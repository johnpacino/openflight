#!/usr/bin/env python3
"""Measure static reflectors as an IWR6843 horizontal aim reference.

Run this with OpenFlight stopped. The script configures the custom L3 firmware,
captures repeated ring dumps, and reads the stationary reflector directly from
the range snapshots without MTI. With two reflector positions, their measured
vector defines the TrackMan target line and diagnoses lateral radar placement.
For a sphere on a reflective holder, the three-stage mode captures the empty
scene, holder, and sphere so the holder can be coherently subtracted. A saved
result is not an RF factory boresight calibration.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from openflight.iwr6843.calibration import DEFAULT_CAL_PATH, Calibration
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dump import parse_dump, range_data
from openflight.iwr6843.lcmf import (
    _phase_to_angle_deg,
    _weighted_circular_mean,
)
from openflight.iwr6843.shot import geometry_from_header

try:
    from openflight.iwr6843.doa import circular_median
except ImportError:  # Pi's validated July 22 branch keeps this helper in lcmf.
    from openflight.iwr6843.lcmf import _circular_median as circular_median

MIN_REFERENCE_BALANCE = 0.30
MIN_RX_COHERENCE = 0.98
MIN_BACKGROUND_ALIGNMENT = 0.98


class UnsafeCalibrationError(RuntimeError):
    """A detected reflector return is unsuitable for aim calibration."""


@dataclass(frozen=True)
class StaticReading:
    """One frame's strongest static reflector response inside the range gate."""

    range_m: float
    phase_rad: float
    raw_bearing_deg: float
    rx_coherence: float
    reference_balance: float
    power_ratio: float
    frame: int
    absolute_bin: int


@dataclass(frozen=True)
class AimEstimate:
    """Robust reflector estimate fused across frames and captures."""

    range_m: float
    phase_rad: float
    raw_bearing_deg: float
    bearing_stability_deg: float
    coherence: float
    median_rx_coherence: float
    median_power_ratio: float
    readings: int
    median_reference_balance: float = 1.0


@dataclass(frozen=True)
class AlignmentReport:
    """Live target displacement relative to raw or corrected azimuth zero."""

    aim_error_deg: float
    lateral_error_cm: float
    direction: str


@dataclass(frozen=True)
class TwoPositionAimEstimate:
    """Target-line geometry inferred from two reflector positions."""

    raw_target_line_bearing_deg: float
    azimuth_offset_deg: float
    lateral_offset_m: float
    target_separation_m: float
    line_bearing_stability_deg: float
    near: AimEstimate
    far: AimEstimate


@dataclass(frozen=True)
class StaticScene:
    """Loop/frame-averaged complex snapshots indexed by absolute range bin."""

    snapshots: dict[int, np.ndarray]
    ranges_m: dict[int, float]


def _phase_delta(value: float, reference: float) -> float:
    return float(np.angle(np.exp(1j * (value - reference))))


def _corrected_bearing_deg(phase_rad: float, reference_phase_rad: float) -> float:
    """Convert phase relative to a saved target-line reference into bearing."""
    return -_phase_to_angle_deg(_phase_delta(phase_rad, reference_phase_rad))


def static_reflector_readings(
    raw: bytes,
    cal: Calibration,
    *,
    expected_range_m: float,
    gate_half_m: float,
    min_power_ratio: float = 3.0,
    min_rx_coherence: float = MIN_RX_COHERENCE,
    min_reference_balance: float = MIN_REFERENCE_BALANCE,
) -> list[StaticReading]:
    """Extract static TX2-vs-TX1/TX3 phase in a tape-measured range gate."""
    meta, cube = parse_dump(raw)
    if meta["n_tx"] != 3:
        raise ValueError(f"horizontal aim calibration requires 3 TX, got {meta['n_tx']}")
    geometry = geometry_from_header(meta)
    loops = meta["chirps_per_frame"] // meta["n_tx"]
    tdm = range_data(meta, cube).reshape(
        meta["n_frames"],
        loops,
        meta["n_tx"],
        meta["n_rx"],
        meta["n_samples"],
    )
    output: list[StaticReading] = []
    cancellation_balances: list[float] = []
    rx_rejected_coherences: list[float] = []
    rx_rejected_references: list[float] = []
    rx_rejected_strengths: list[float] = []
    gate_lo_m = expected_range_m - gate_half_m
    gate_hi_m = expected_range_m + gate_half_m
    for frame in range(meta["n_frames"]):
        start_bin = geometry.frame_bin_start(frame)
        ranges = np.asarray(
            [
                cal.true_range((start_bin + local_bin) * geometry.range_res_m)
                for local_bin in range(meta["n_samples"])
            ]
        )
        gate = np.flatnonzero((ranges >= gate_lo_m) & (ranges <= gate_hi_m))
        if gate.size < 3:
            continue
        snapshots = np.mean(tdm[frame][..., gate], axis=0)
        power = np.sum(np.abs(snapshots) ** 2, axis=(0, 1))
        peak_index = int(np.argmax(power))
        local_bin = int(gate[peak_index])
        peak_power = float(power[peak_index])
        noise_bins = np.ones(len(power), dtype=bool)
        noise_bins[max(0, peak_index - 1) : peak_index + 2] = False
        noise = float(np.median(power[noise_bins])) if np.any(noise_bins) else 0.0
        power_ratio = peak_power / max(noise, 1e-12)
        snapshot = snapshots[:, :, peak_index]
        reference_balance = float(
            np.median(
                np.abs(snapshot[0] + snapshot[2])
                / np.maximum(np.abs(snapshot[0]) + np.abs(snapshot[2]), 1e-12)
            )
        )
        reference = 0.5 * (snapshot[0] + snapshot[2])
        rx_phases = [
            float(value)
            for value in np.angle(np.conj(reference) * snapshot[1])
            if math.isfinite(float(value))
        ]
        if not rx_phases:
            continue
        phase = circular_median(rx_phases)
        rx_coherence = float(abs(np.mean(np.exp(1j * np.asarray(rx_phases)))))
        if power_ratio < min_power_ratio:
            continue
        if reference_balance < min_reference_balance:
            cancellation_balances.append(reference_balance)
            continue
        if rx_coherence < min_rx_coherence:
            rx_rejected_coherences.append(rx_coherence)
            rx_rejected_references.append(reference_balance)
            rx_rejected_strengths.append(power_ratio)
            continue
        output.append(
            StaticReading(
                range_m=float(ranges[local_bin]),
                phase_rad=phase,
                raw_bearing_deg=-_phase_to_angle_deg(phase),
                rx_coherence=rx_coherence,
                reference_balance=reference_balance,
                power_ratio=power_ratio,
                frame=frame,
                absolute_bin=start_bin + local_bin,
            )
        )
    if not output and cancellation_balances:
        raise ValueError(
            "unsafe TX1/TX3 reference cancellation in "
            f"{len(cancellation_balances)} frame(s): median reference="
            f"{statistics.median(cancellation_balances):.3f} "
            f"(requires >={min_reference_balance:.3f}); change reflector elevation "
            "or remove competing clutter, then recapture"
        )
    if not output and rx_rejected_coherences:
        raise ValueError(
            "unsafe RX phase disagreement in "
            f"{len(rx_rejected_coherences)} frame(s): median rx="
            f"{statistics.median(rx_rejected_coherences):.3f} "
            f"(requires >={min_rx_coherence:.3f}), reference="
            f"{statistics.median(rx_rejected_references):.3f}, strength="
            f"{statistics.median(rx_rejected_strengths):.1f}x; re-aim the reflector "
            "toward the antenna center, step away, then recapture"
        )
    return output


def _static_scene(raw: bytes, cal: Calibration) -> StaticScene:
    """Collapse one static dump into a complex TX/RX snapshot per range bin."""
    meta, cube = parse_dump(raw)
    if meta["n_tx"] != 3:
        raise ValueError(f"horizontal aim calibration requires 3 TX, got {meta['n_tx']}")
    if meta["chirps_per_frame"] % meta["n_tx"]:
        raise ValueError("chirps_per_frame must be divisible by n_tx")
    geometry = geometry_from_header(meta)
    loops = meta["chirps_per_frame"] // meta["n_tx"]
    tdm = range_data(meta, cube).reshape(
        meta["n_frames"],
        loops,
        meta["n_tx"],
        meta["n_rx"],
        meta["n_samples"],
    )
    range_bin_counts = meta.get("range_bin_counts")
    snapshots_by_bin: dict[int, list[np.ndarray]] = {}
    ranges_m: dict[int, float] = {}
    for frame in range(meta["n_frames"]):
        start_bin = geometry.frame_bin_start(frame)
        count = meta["n_samples"] if range_bin_counts is None else int(range_bin_counts[frame])
        count = min(count, meta["n_samples"])
        frame_snapshots = np.mean(tdm[frame, ..., :count], axis=0)
        for local_bin in range(count):
            absolute_bin = start_bin + local_bin
            snapshots_by_bin.setdefault(absolute_bin, []).append(frame_snapshots[..., local_bin])
            ranges_m[absolute_bin] = cal.true_range(absolute_bin * geometry.range_res_m)
    return StaticScene(
        snapshots={
            absolute_bin: np.mean(snapshots, axis=0)
            for absolute_bin, snapshots in snapshots_by_bin.items()
        },
        ranges_m=ranges_m,
    )


def _alignment_bins(
    reference: StaticScene,
    observed: StaticScene,
    *,
    expected_range_m: float,
    gate_half_m: float,
) -> list[int]:
    """Stable scene bins used to estimate capture-wide phase/gain drift."""
    common = sorted(set(reference.snapshots) & set(observed.snapshots))
    if not common:
        raise ValueError("background alignment found no common range bins")
    range_steps = [
        abs(reference.ranges_m[right] - reference.ranges_m[left])
        for left, right in zip(common, common[1:])
        if reference.ranges_m[right] != reference.ranges_m[left]
    ]
    if not range_steps:
        raise ValueError("background alignment needs multiple distinct range bins")
    range_res_m = min(range_steps)
    guard_half_m = gate_half_m + 2.0 * range_res_m
    bins = [
        absolute_bin
        for absolute_bin in common
        if abs(reference.ranges_m[absolute_bin] - expected_range_m) > guard_half_m
    ]
    if len(bins) < 3:
        raise ValueError(
            "background alignment needs at least three bins outside the reflector range gate"
        )
    return bins


def _complex_alignment(
    reference: StaticScene,
    observed: StaticScene,
    bins: list[int],
) -> tuple[complex, float]:
    """Fit ``observed ~= scale * reference`` and report normalized coherence."""
    reference_vector = np.concatenate([reference.snapshots[bin_].ravel() for bin_ in bins])
    observed_vector = np.concatenate([observed.snapshots[bin_].ravel() for bin_ in bins])
    reference_power = float(np.vdot(reference_vector, reference_vector).real)
    observed_power = float(np.vdot(observed_vector, observed_vector).real)
    if reference_power <= 0.0 or observed_power <= 0.0:
        raise ValueError("background alignment has no usable static-scene power")
    cross = np.vdot(reference_vector, observed_vector)
    return complex(cross / reference_power), float(
        abs(cross) / math.sqrt(reference_power * observed_power)
    )


def _aligned_scene_average(
    captures: list[bytes],
    cal: Calibration,
    *,
    expected_range_m: float,
    gate_half_m: float,
    min_alignment: float,
) -> StaticScene:
    if not captures:
        raise ValueError("at least one background capture is required")
    scenes = [_static_scene(raw, cal) for raw in captures]
    template = scenes[0]
    aligned: list[dict[int, np.ndarray]] = []
    common_bins = set(template.snapshots)
    for capture, scene in enumerate(scenes, start=1):
        common_bins &= set(scene.snapshots)
        bins = _alignment_bins(
            template,
            scene,
            expected_range_m=expected_range_m,
            gate_half_m=gate_half_m,
        )
        scale, coherence = _complex_alignment(template, scene, bins)
        if coherence < min_alignment:
            raise ValueError(
                f"background alignment for capture {capture} is {coherence:.3f}; "
                f"requires >= {min_alignment:.3f}. Something in the scene moved."
            )
        aligned.append(
            {absolute_bin: snapshot / scale for absolute_bin, snapshot in scene.snapshots.items()}
        )
    return StaticScene(
        snapshots={
            absolute_bin: np.mean(
                [capture[absolute_bin] for capture in aligned],
                axis=0,
            )
            for absolute_bin in sorted(common_bins)
        },
        ranges_m={absolute_bin: template.ranges_m[absolute_bin] for absolute_bin in common_bins},
    )


def differential_reflector_readings(
    background_captures: list[bytes],
    reflector_captures: list[bytes],
    cal: Calibration,
    *,
    expected_range_m: float,
    gate_half_m: float,
    min_power_ratio: float = 3.0,
    min_rx_coherence: float = MIN_RX_COHERENCE,
    min_reference_balance: float = MIN_REFERENCE_BALANCE,
    min_background_alignment: float = MIN_BACKGROUND_ALIGNMENT,
) -> list[StaticReading]:
    """Isolate a reflector by subtracting an unchanged static background.

    A single complex gain/phase term is fitted outside the target range gate
    before subtraction. This removes capture-wide RF drift without changing the
    TX-to-TX phase that contains horizontal bearing.
    """
    if not reflector_captures:
        raise ValueError("at least one reflector capture is required")
    background = _aligned_scene_average(
        background_captures,
        cal,
        expected_range_m=expected_range_m,
        gate_half_m=gate_half_m,
        min_alignment=min_background_alignment,
    )
    output: list[StaticReading] = []
    rejected_quality: list[str] = []
    for capture, raw in enumerate(reflector_captures, start=1):
        reflector = _static_scene(raw, cal)
        alignment_bins = _alignment_bins(
            background,
            reflector,
            expected_range_m=expected_range_m,
            gate_half_m=gate_half_m,
        )
        scale, alignment = _complex_alignment(background, reflector, alignment_bins)
        if alignment < min_background_alignment:
            raise ValueError(
                f"background alignment for reflector capture {capture} is {alignment:.3f}; "
                f"requires >= {min_background_alignment:.3f}. Do not move the holder "
                "between stages."
            )
        gate = [
            absolute_bin
            for absolute_bin in sorted(set(background.snapshots) & set(reflector.snapshots))
            if abs(background.ranges_m[absolute_bin] - expected_range_m) <= gate_half_m
        ]
        if len(gate) < 3:
            raise ValueError("reflector range gate contains fewer than three captured bins")
        differences = {
            absolute_bin: reflector.snapshots[absolute_bin]
            - scale * background.snapshots[absolute_bin]
            for absolute_bin in gate
        }
        powers = np.asarray(
            [np.sum(np.abs(differences[absolute_bin]) ** 2) for absolute_bin in gate]
        )
        peak_index = int(np.argmax(powers))
        absolute_bin = gate[peak_index]
        noise_mask = np.ones(len(gate), dtype=bool)
        noise_mask[max(0, peak_index - 1) : peak_index + 2] = False
        noise = float(np.median(powers[noise_mask])) if np.any(noise_mask) else 0.0
        power_ratio = float(powers[peak_index]) / max(noise, 1e-12)
        snapshot = differences[absolute_bin]
        reference_balance = float(
            np.median(
                np.abs(snapshot[0] + snapshot[2])
                / np.maximum(np.abs(snapshot[0]) + np.abs(snapshot[2]), 1e-12)
            )
        )
        reference = 0.5 * (snapshot[0] + snapshot[2])
        rx_phases = np.angle(np.conj(reference) * snapshot[1])
        phase = circular_median([float(value) for value in rx_phases])
        rx_coherence = float(abs(np.mean(np.exp(1j * rx_phases))))
        failures = []
        if power_ratio < min_power_ratio:
            failures.append(f"strength={power_ratio:.1f}x")
        if reference_balance < min_reference_balance:
            failures.append(f"reference={reference_balance:.3f}")
        if rx_coherence < min_rx_coherence:
            failures.append(f"rx={rx_coherence:.3f}")
        if failures:
            rejected_quality.append(f"capture {capture}: " + ", ".join(failures))
            continue
        output.append(
            StaticReading(
                range_m=background.ranges_m[absolute_bin],
                phase_rad=phase,
                raw_bearing_deg=-_phase_to_angle_deg(phase),
                rx_coherence=rx_coherence,
                reference_balance=reference_balance,
                power_ratio=power_ratio,
                frame=capture - 1,
                absolute_bin=absolute_bin,
            )
        )
    if not output:
        detail = "; ".join(rejected_quality) or "no differential peak passed quality gates"
        raise ValueError(f"isolated reflector is unsafe: {detail}")
    return output


def fuse_readings(readings: list[StaticReading]) -> AimEstimate:
    """Fuse static readings while retaining an honest stability statistic."""
    if not readings:
        raise ValueError("no reflector readings passed range/power/coherence gates")
    weights = [reading.power_ratio * reading.rx_coherence for reading in readings]
    phase, coherence = _weighted_circular_mean(
        [reading.phase_rad for reading in readings],
        weights,
    )
    raw_bearing = -_phase_to_angle_deg(phase)
    bearing_samples = [
        raw_bearing + _corrected_bearing_deg(reading.phase_rad, phase) for reading in readings
    ]
    return AimEstimate(
        range_m=float(np.average([reading.range_m for reading in readings], weights=weights)),
        phase_rad=phase,
        raw_bearing_deg=raw_bearing,
        bearing_stability_deg=float(np.std(bearing_samples)),
        coherence=coherence,
        median_rx_coherence=float(statistics.median(reading.rx_coherence for reading in readings)),
        median_power_ratio=float(statistics.median(reading.power_ratio for reading in readings)),
        readings=len(readings),
        median_reference_balance=float(
            statistics.median(reading.reference_balance for reading in readings)
        ),
    )


def _polar_xy(estimate: AimEstimate) -> tuple[float, float]:
    bearing_rad = math.radians(estimate.raw_bearing_deg)
    return (
        estimate.range_m * math.cos(bearing_rad),
        estimate.range_m * math.sin(bearing_rad),
    )


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def solve_two_position_target_line(
    first: AimEstimate,
    second: AimEstimate,
    *,
    min_separation_m: float = 0.50,
) -> TwoPositionAimEstimate:
    """Solve target-line yaw and lateral displacement from two static targets.

    The nearer-to-farther reflector vector defines the target-line direction in
    the radar's raw horizontal frame. Its negative is the additive azimuth
    correction that maps that direction to 0 degrees. The signed perpendicular
    distance from the radar origin to the same line is retained as a placement
    diagnostic; it is not folded into the angle correction.
    """
    if min_separation_m <= 0:
        raise ValueError("min_separation_m must be positive")
    near, far = sorted((first, second), key=lambda estimate: estimate.range_m)
    near_x, near_y = _polar_xy(near)
    far_x, far_y = _polar_xy(far)
    delta_x = far_x - near_x
    delta_y = far_y - near_y
    separation = math.hypot(delta_x, delta_y)
    if separation < min_separation_m:
        raise ValueError(
            "reflector positions must be at least "
            f"{min_separation_m:.2f} m apart; measured {separation:.2f} m"
        )

    line_bearing = _wrap_degrees(math.degrees(math.atan2(delta_y, delta_x)))
    lateral_offset = (delta_x * near_y - delta_y * near_x) / separation
    near_cross_range_sigma = near.range_m * math.radians(near.bearing_stability_deg)
    far_cross_range_sigma = far.range_m * math.radians(far.bearing_stability_deg)
    line_stability = math.degrees(
        math.hypot(near_cross_range_sigma, far_cross_range_sigma) / separation
    )
    return TwoPositionAimEstimate(
        raw_target_line_bearing_deg=line_bearing,
        azimuth_offset_deg=_wrap_degrees(-line_bearing),
        lateral_offset_m=lateral_offset,
        target_separation_m=separation,
        line_bearing_stability_deg=line_stability,
        near=near,
        far=far,
    )


def _load_reference(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        reference = json.load(handle)
    if "target_line_phase_rad" not in reference and "azimuth_offset_deg" not in reference:
        raise ValueError(f"{path} has neither target_line_phase_rad nor azimuth_offset_deg")
    return reference


def _reference_corrected_bearing(
    estimate: AimEstimate,
    reference: dict[str, Any] | None,
) -> float | None:
    if reference is None:
        return None
    if "azimuth_offset_deg" in reference:
        return _wrap_degrees(estimate.raw_bearing_deg + float(reference["azimuth_offset_deg"]))
    return _corrected_bearing_deg(
        estimate.phase_rad,
        float(reference["target_line_phase_rad"]),
    )


def _alignment_report(
    estimate: AimEstimate,
    *,
    azimuth_offset_deg: float | None,
    reference: dict[str, Any] | None,
) -> AlignmentReport:
    """Convert a reflector bearing into target miss at the measured range."""
    if azimuth_offset_deg is not None and reference is not None:
        raise ValueError("use either azimuth_offset_deg or reference, not both")
    if azimuth_offset_deg is not None:
        aim_error_deg = _wrap_degrees(estimate.raw_bearing_deg + azimuth_offset_deg)
    else:
        corrected = _reference_corrected_bearing(estimate, reference)
        aim_error_deg = estimate.raw_bearing_deg if corrected is None else corrected
    lateral_error_cm = 100.0 * estimate.range_m * math.sin(math.radians(aim_error_deg))
    if abs(aim_error_deg) < 0.05:
        direction = "CENTERED"
    else:
        direction = "RIGHT" if aim_error_deg > 0 else "LEFT"
    return AlignmentReport(
        aim_error_deg=aim_error_deg,
        lateral_error_cm=lateral_error_cm,
        direction=direction,
    )


def _print_estimate(
    label: str,
    estimate: AimEstimate,
    reference: dict[str, Any] | None,
) -> None:
    corrected = _reference_corrected_bearing(estimate, reference)
    corrected_text = f"  corrected={corrected:+6.2f} deg" if corrected is not None else ""
    print(
        f"{label}: range={estimate.range_m:5.2f} m  "
        f"raw={estimate.raw_bearing_deg:+6.2f} deg{corrected_text}  "
        f"stability=+/-{estimate.bearing_stability_deg:4.2f} deg  "
        f"temporal={estimate.coherence:.3f}  "
        f"rx={estimate.median_rx_coherence:.3f}  "
        f"reference={estimate.median_reference_balance:.3f}  "
        f"strength={estimate.median_power_ratio:.1f}x  n={estimate.readings}"
    )


def _capture_position(
    radar: IWR6843Radar,
    cal: Calibration,
    *,
    label: str,
    expected_range_m: float,
    gate_half_m: float,
    captures: int,
    interval_s: float,
    outdir: Path | None,
    reference: dict[str, Any] | None,
) -> AimEstimate:
    readings_for_position: list[StaticReading] = []
    for capture in range(1, captures + 1):
        raw = radar.read_dump()
        if outdir:
            (outdir / f"aim_{label}_{capture:02d}.l3dump").write_bytes(raw)
        try:
            readings = static_reflector_readings(
                raw,
                cal,
                expected_range_m=expected_range_m,
                gate_half_m=gate_half_m,
            )
        except ValueError as error:
            raise UnsafeCalibrationError(f"{label} capture {capture}: {error}") from error
        if readings:
            estimate = fuse_readings(readings)
            _print_estimate(f"{label} capture {capture}", estimate, reference)
            readings_for_position.extend(readings)
        else:
            print(f"{label} capture {capture}: no strong reflector near {expected_range_m:.2f} m")
        time.sleep(interval_s)
    if not readings_for_position:
        raise RuntimeError(
            f"no reflector detected for {label} position near {expected_range_m:.2f} m"
        )
    return fuse_readings(readings_for_position)


def _capture_raw_stage(
    radar: IWR6843Radar,
    cal: Calibration,
    *,
    label: str,
    captures: int,
    interval_s: float,
    outdir: Path | None,
) -> list[bytes]:
    """Capture and validate raw dumps without selecting a static target yet."""
    output: list[bytes] = []
    for capture in range(1, captures + 1):
        raw = radar.read_dump()
        try:
            _static_scene(raw, cal)
        except ValueError as error:
            raise UnsafeCalibrationError(f"{label} capture {capture}: {error}") from error
        if outdir:
            path = outdir / f"aim_{label}_{capture:02d}.l3dump"
            path.write_bytes(raw)
            print(f"{label} capture {capture}: saved {path.name}")
        else:
            print(f"{label} capture {capture}: captured")
        output.append(raw)
        time.sleep(interval_s)
    return output


def _run_live_alignment(
    radar: IWR6843Radar,
    cal: Calibration,
    *,
    expected_range_m: float,
    gate_half_m: float,
    interval_s: float,
    azimuth_offset_deg: float | None,
    reference: dict[str, Any] | None,
) -> None:
    """Continuously print reflector aim error until interrupted."""
    zero_label = "CORRECTED ZERO" if azimuth_offset_deg is not None or reference else "RAW RF ZERO"
    print(f"\nLive alignment relative to {zero_label}; press Ctrl-C to stop.")
    if zero_label == "RAW RF ZERO":
        print(
            "WARNING: raw zero includes board-specific electrical phase and is not "
            "proven mechanical boresight."
        )
    capture = 0
    while True:
        capture += 1
        try:
            raw = radar.read_dump()
            readings = static_reflector_readings(
                raw,
                cal,
                expected_range_m=expected_range_m,
                gate_half_m=gate_half_m,
            )
            estimate = fuse_readings(readings)
            report = _alignment_report(
                estimate,
                azimuth_offset_deg=azimuth_offset_deg,
                reference=reference,
            )
            if report.direction == "CENTERED":
                instruction = "CENTERED"
            else:
                instruction = f"turn enclosure {report.direction}"
            print(
                f"{capture:04d}  range={estimate.range_m:5.2f} m  "
                f"raw={estimate.raw_bearing_deg:+6.2f} deg  "
                f"aim={report.aim_error_deg:+6.2f} deg  "
                f"miss={abs(report.lateral_error_cm):6.1f} cm {report.direction:<8}  "
                f"rx={estimate.median_rx_coherence:.3f}  "
                f"reference={estimate.median_reference_balance:.3f}  "
                f"strength={estimate.median_power_ratio:.1f}x  {instruction}"
            )
        except ValueError as error:
            print(f"{capture:04d}  REJECTED: {error}")
        time.sleep(interval_s)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="TI CLI port; auto-detected by default")
    parser.add_argument(
        "--cfg",
        default="config/iwr6843_l3dump_vTX2_window53_12l18f.cfg",
    )
    parser.add_argument("--cal", default=DEFAULT_CAL_PATH)
    parser.add_argument("--reflector-range-m", type=float, required=True)
    parser.add_argument(
        "--second-reflector-range-m",
        type=float,
        default=None,
        help=(
            "Enable a two-position target-line solve. Move the same reflector "
            "to this second approximate range when prompted."
        ),
    )
    parser.add_argument("--gate-half-m", type=float, default=0.20)
    parser.add_argument("--captures", type=int, default=3)
    parser.add_argument("--interval-s", type=float, default=0.4)
    parser.add_argument(
        "--live-align",
        action="store_true",
        help="Continuously report target bearing and left/right miss until Ctrl-C",
    )
    parser.add_argument(
        "--sphere-three-stage",
        action="store_true",
        help=(
            "Capture empty scene, holder only, then sphere on the unchanged holder; "
            "subtract the holder to isolate the sphere"
        ),
    )
    parser.add_argument(
        "--azimuth-offset-deg",
        type=float,
        default=None,
        help="Known additive azimuth correction for live alignment",
    )
    parser.add_argument("--reference", type=Path, default=None)
    parser.add_argument("--save-reference", type=Path, default=None)
    parser.add_argument("--outdir", type=Path, default=None)
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="Do not pause before each target position (automation/testing only)",
    )
    args = parser.parse_args()

    if args.captures < 1:
        parser.error("--captures must be at least 1")
    if args.reflector_range_m <= 0:
        parser.error("--reflector-range-m must be positive")
    if args.second_reflector_range_m is not None and args.second_reflector_range_m <= 0:
        parser.error("--second-reflector-range-m must be positive")
    if args.gate_half_m <= 0:
        parser.error("--gate-half-m must be positive")
    if args.interval_s < 0:
        parser.error("--interval-s cannot be negative")
    if args.reference and args.azimuth_offset_deg is not None:
        parser.error("use either --reference or --azimuth-offset-deg, not both")
    if args.live_align and args.second_reflector_range_m is not None:
        parser.error("--live-align supports one reflector range")
    if args.sphere_three_stage and args.live_align:
        parser.error("--sphere-three-stage cannot be combined with --live-align")
    if args.sphere_three_stage and args.second_reflector_range_m is not None:
        parser.error("--sphere-three-stage supports one sphere position")
    if args.live_align and args.save_reference:
        parser.error("--live-align cannot save a calibration reference")
    if args.live_align and args.outdir:
        parser.error("--live-align does not save raw dumps; omit --outdir")
    if (
        args.second_reflector_range_m is not None
        and abs(args.reflector_range_m - args.second_reflector_range_m) < 0.50
    ):
        parser.error("approximate reflector ranges must differ by at least 0.50 m")
    cal = Calibration.load(args.cal)
    reference = _load_reference(args.reference) if args.reference else None
    if args.outdir:
        args.outdir.mkdir(parents=True, exist_ok=True)

    estimates: list[AimEstimate] = []
    ranges = [args.reflector_range_m]
    if args.second_reflector_range_m is not None:
        ranges.append(args.second_reflector_range_m)
    print("Configuring IWR6843; OpenFlight must be stopped...")
    with IWR6843Radar(port=args.port) as radar:
        radar.send_config(args.cfg)
        print(f"Radar running on {radar.port}.")
        if args.sphere_three_stage:
            prompts = (
                (
                    "empty_scene",
                    "\nStage 1/3 — EMPTY SCENE: Remove the box, cup, sphere, and any "
                    "other objects from the target area. Do not move the radar. Step "
                    "away, then press Enter.",
                ),
                (
                    "holder_only",
                    "\nStage 2/3 — HOLDER ONLY: Place the box corner-facing and cup in "
                    "their final marked positions. Leave the sphere off. Step away, "
                    "then press Enter.",
                ),
                (
                    "sphere_present",
                    "\nStage 3/3 — SPHERE: Add only the sphere to the unchanged holder, "
                    f"centered on the target line near {args.reflector_range_m:.2f} m. "
                    "Step away, then press Enter.",
                ),
            )
            stage_captures: dict[str, list[bytes]] = {}
            try:
                for label, prompt in prompts:
                    if not args.no_prompt:
                        input(prompt)
                    print(f"Capturing {label}...")
                    stage_captures[label] = _capture_raw_stage(
                        radar,
                        cal,
                        label=label,
                        captures=args.captures,
                        interval_s=args.interval_s,
                        outdir=args.outdir,
                    )
            except UnsafeCalibrationError as error:
                raise SystemExit(str(error)) from error

            print("\nThree-stage subtraction")
            try:
                holder_readings = differential_reflector_readings(
                    stage_captures["empty_scene"],
                    stage_captures["holder_only"],
                    cal,
                    expected_range_m=args.reflector_range_m,
                    gate_half_m=args.gate_half_m,
                )
                _print_estimate(
                    "Holder contribution",
                    fuse_readings(holder_readings),
                    None,
                )
            except ValueError as error:
                print(f"Holder diagnostic unavailable: {error}")
            try:
                sphere_readings = differential_reflector_readings(
                    stage_captures["holder_only"],
                    stage_captures["sphere_present"],
                    cal,
                    expected_range_m=args.reflector_range_m,
                    gate_half_m=args.gate_half_m,
                )
            except ValueError as error:
                raise SystemExit(f"Sphere isolation failed: {error}") from error
            for capture, reading in enumerate(sphere_readings, start=1):
                _print_estimate(
                    f"Isolated sphere capture {capture}",
                    fuse_readings([reading]),
                    reference,
                )
            estimates.append(fuse_readings(sphere_readings))
        if args.live_align:
            if not args.no_prompt:
                input(
                    "\nPlace the reflector approximately "
                    f"{args.reflector_range_m:.2f} m from the antenna center on "
                    "the target line. Step away, then press Enter."
                )
            try:
                _run_live_alignment(
                    radar,
                    cal,
                    expected_range_m=args.reflector_range_m,
                    gate_half_m=args.gate_half_m,
                    interval_s=args.interval_s,
                    azimuth_offset_deg=args.azimuth_offset_deg,
                    reference=reference,
                )
            except KeyboardInterrupt:
                print("\nLive alignment stopped.")
            return 0
        for index, expected_range_m in enumerate(ranges, start=1):
            if args.sphere_three_stage:
                break
            label = f"position_{index}"
            if not args.no_prompt:
                input(
                    f"\nPlace the reflector at position {index}, approximately "
                    f"{expected_range_m:.2f} m from the antenna center, centered "
                    "on the target line. Step away, then press Enter."
                )
            print(f"Capturing {label}...")
            try:
                estimates.append(
                    _capture_position(
                        radar,
                        cal,
                        label=label,
                        expected_range_m=expected_range_m,
                        gate_half_m=args.gate_half_m,
                        captures=args.captures,
                        interval_s=args.interval_s,
                        outdir=args.outdir,
                        reference=reference,
                    )
                )
            except UnsafeCalibrationError as error:
                raise SystemExit(str(error)) from error
            except RuntimeError as error:
                raise SystemExit(
                    f"{error}. Verify the approximate range, widen --gate-half-m, "
                    "and confirm the current firmware window stores that range."
                ) from error

    print("\nCombined")
    for index, estimate in enumerate(estimates, start=1):
        label = "Isolated sphere" if args.sphere_three_stage else f"Position {index}"
        _print_estimate(label, estimate, reference)

    if len(estimates) == 2:
        solved = solve_two_position_target_line(*estimates)
        print("\nTwo-position target-line solve")
        print(f"Raw target-line bearing: {solved.raw_target_line_bearing_deg:+.2f} deg")
        print(f"Additive azimuth offset: {solved.azimuth_offset_deg:+.2f} deg")
        print(f"Radar lateral offset: {solved.lateral_offset_m:+.3f} m")
        print(f"Measured target separation: {solved.target_separation_m:.2f} m")
        print(f"Estimated line-bearing stability: +/-{solved.line_bearing_stability_deg:.2f} deg")
        print(
            "Test with: scripts/start-kiosk.sh --iwr6843 "
            f"--iwr6843-azimuth-offset-deg {solved.azimuth_offset_deg:.3f}"
        )
    else:
        solved = None
        estimate = estimates[0]
        print(f"Raw target-line phase: {estimate.phase_rad:+.6f} rad")

    if args.save_reference and solved is not None:
        payload = {
            "type": "iwr6843_horizontal_two_position_target_line",
            "method": "static_two_position",
            "created_unix": time.time(),
            "azimuth_offset_deg": solved.azimuth_offset_deg,
            "raw_target_line_bearing_deg": solved.raw_target_line_bearing_deg,
            "lateral_offset_m": solved.lateral_offset_m,
            "target_separation_m": solved.target_separation_m,
            "line_bearing_stability_deg": solved.line_bearing_stability_deg,
            "calibration": args.cal,
            "config": args.cfg,
            "positions": [asdict(solved.near), asdict(solved.far)],
        }
        args.save_reference.parent.mkdir(parents=True, exist_ok=True)
        args.save_reference.write_text(
            json.dumps(payload, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"Saved two-position target-line calibration: {args.save_reference}")
    elif args.save_reference:
        estimate = estimates[0]
        payload = {
            "type": "iwr6843_horizontal_target_line_reference",
            "method": (
                "three_stage_complex_background_subtraction"
                if args.sphere_three_stage
                else "static_single_position"
            ),
            "created_unix": time.time(),
            "target_line_phase_rad": estimate.phase_rad,
            "raw_bearing_deg": estimate.raw_bearing_deg,
            "reflector_range_m": estimate.range_m,
            "bearing_stability_deg": estimate.bearing_stability_deg,
            "coherence": estimate.coherence,
            "readings": estimate.readings,
            "calibration": args.cal,
            "config": args.cfg,
            "estimate": asdict(estimate),
        }
        args.save_reference.parent.mkdir(parents=True, exist_ok=True)
        args.save_reference.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        print(f"Saved target-line reference: {args.save_reference}")
        print(
            "This file defines the reflector line as 0 deg; it does not prove "
            "that raw RF boresight is 0 deg."
        )
    elif reference is None and solved is None:
        print(
            "No reference applied. Raw bearing includes fixed TX electrical phase. "
            "Use --save-reference with the reflector on TrackMan's target line."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
