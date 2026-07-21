#!/usr/bin/env python3
"""OPS-guided clubhead tracker for IWR6843 L3 dumps.

This diagnostic tries to solve the target-identity problem seen in the simple
club-path / attack-angle probes. Instead of selecting the brightest near-tee
return independently in each frame, it:

1. Uses OPS impact and last-club timing to choose the relevant TI time window.
2. Extracts multiple near-tee candidate peaks per loop from the TI cube.
3. Fits a range-time ridge constrained by a broad OPS club-speed prior.
4. Computes horizontal and vertical motion from that one selected ridge.

Horizontal output requires 3TX firmware because it uses TX2 phase against the
TX1/TX3 vertical reference. Vertical/attack diagnostics can also run on the
earlier 2TX TrackMan-session dumps.

The outputs are still experimental proxies, but the selected reflector should
be more club-like than a per-frame max-power picker.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from openflight.iwr6843 import doa
from openflight.iwr6843.calibration import DEFAULT_CAL_PATH, Calibration
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.lcmf import _circular_median, _phase_to_angle_deg, _weighted_circular_mean
from openflight.iwr6843.music import GRID, _grid_steer, est_bartlett, est_music_fbss_high
from openflight.iwr6843.shot import TX2_LOOP_PERIOD_S, TX2_VERTICAL_TDM_TAU_S
from openflight.iwr6843.tracking import Geometry

MPH_PER_MS = 2.23694


@dataclass(frozen=True)
class ShotJoin:
    """Session rows needed to align OPS and IWR."""

    shot: int
    dump: Path
    iwr_trigger_delta_ms: float
    ops_club_mph: float | None
    ops_ball_mph: float | None
    ops_impact_ms: float | None
    ops_last_club_center_ms: float | None
    ops_last_club_mph: float | None
    ops_first_ball_center_ms: float | None
    live_h_deg: float | None
    live_v_deg: float | None


@dataclass(frozen=True)
class Candidate:
    """One near-tee candidate peak at one loop time."""

    frame: int
    loop: int
    time_ms: float
    range_m: float
    range_bin: int
    snr: float
    weight: float


def _float(value: Any) -> float | None:
    try:
        if value in (None, ""):
            return None
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _fmt(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return "" if not math.isfinite(value) else f"{value:.3f}"
    return str(value)


def _load_session(path: Path) -> list[ShotJoin]:
    rolling: dict[int, dict[str, Any]] = {}
    shots: dict[int, dict[str, Any]] = {}
    captures: dict[int, dict[str, Any]] = {}
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            if not line.strip():
                continue
            entry = json.loads(line)
            event = entry.get("event") or entry.get("type")
            shot = entry.get("shot_number")
            if shot is None:
                continue
            shot = int(shot)
            if event == "rolling_buffer_capture":
                rolling[shot] = entry
            elif event == "shot_detected":
                shots[shot] = entry
            elif event == "iwr6843_capture" and entry.get("capture_path"):
                captures[shot] = entry

    rows: list[ShotJoin] = []
    for shot, capture in sorted(captures.items()):
        roll = rolling.get(shot, {})
        shot_row = shots.get(shot, {})
        measurement = capture.get("measurement") or {}
        rows.append(
            ShotJoin(
                shot=shot,
                dump=Path(capture["capture_path"]),
                iwr_trigger_delta_ms=float(capture.get("trigger_delta_ms") or 0.0),
                ops_club_mph=_float(roll.get("club_speed_mph"))
                or _float(shot_row.get("club_speed_mph")),
                ops_ball_mph=_float(roll.get("ball_speed_mph"))
                or _float(shot_row.get("ball_speed_mph")),
                ops_impact_ms=_float(roll.get("impact_timestamp_ms")),
                ops_last_club_center_ms=_float(roll.get("impact_last_club_center_ms"))
                or _float(roll.get("impact_last_club_timestamp_ms")),
                ops_last_club_mph=_float(roll.get("impact_last_club_speed_mph"))
                or _float(roll.get("club_speed_mph")),
                ops_first_ball_center_ms=_float(roll.get("impact_first_ball_center_ms"))
                or _float(roll.get("impact_first_ball_timestamp_ms")),
                live_h_deg=_float(shot_row.get("launch_angle_horizontal"))
                or _float(measurement.get("horizontal_deg")),
                live_v_deg=_float(shot_row.get("launch_angle_vertical"))
                or _float(measurement.get("launch_angle_deg")),
            )
        )
    return rows


def _geometry(meta: dict) -> Geometry:
    loop_period_s = TX2_LOOP_PERIOD_S if meta["n_tx"] == 3 else doa.TDM_TAU_S * meta["n_tx"]
    return Geometry(
        n_frames=meta["n_frames"],
        chirps_per_frame=meta["chirps_per_frame"],
        n_tx=meta["n_tx"],
        n_rx=meta["n_rx"],
        n_samples=meta["n_samples"],
        frame_period_s=(meta.get("frame_period_us", 0) / 1e6) or 0.012,
        trigger_frame=meta["trigger_frame"],
        loop_period_s=loop_period_s,
    )


def _vertical_tx_indices_and_tau(meta: dict) -> tuple[int, int, float]:
    """Return the vertical TX pair and their TDM separation for a dump."""
    if meta["n_tx"] == 2:
        return 0, 1, doa.TDM_TAU_S
    if meta["n_tx"] == 3:
        return 0, 2, TX2_VERTICAL_TDM_TAU_S
    raise ValueError(f"unsupported n_tx={meta['n_tx']}")


def _local_peak_bins(gate: np.ndarray, *, top_k: int) -> list[int]:
    peaks = [
        idx
        for idx in range(1, len(gate) - 1)
        if gate[idx] >= gate[idx - 1] and gate[idx] >= gate[idx + 1]
    ]
    if not peaks:
        peaks = list(range(len(gate)))
    return sorted(peaks, key=lambda idx: float(gate[idx]), reverse=True)[:top_k]


def _subbin(power: np.ndarray, idx: int, lo: int, hi: int) -> float:
    peak = float(idx)
    if lo < idx < hi - 1:
        y0, y1, y2 = power[idx - 1], power[idx], power[idx + 1]
        den = y0 - 2 * y1 + y2
        if den < 0:
            off = (y0 - y2) / (2 * den)
            if abs(off) < 1.0:
                peak += float(off)
    return peak


def _horizontal_at(mti: np.ndarray, cand: Candidate, velocity_ms: float) -> float | None:
    tx1 = mti[cand.frame, cand.loop, 0, :, cand.range_bin]
    tx2 = mti[cand.frame, cand.loop, 1, :, cand.range_bin] * np.exp(
        -1j * 4.0 * np.pi * velocity_ms * doa.TDM_TAU_S / doa.LAM
    )
    tx3 = mti[cand.frame, cand.loop, 2, :, cand.range_bin] * np.exp(
        -1j * 4.0 * np.pi * velocity_ms * TX2_VERTICAL_TDM_TAU_S / doa.LAM
    )
    reference = 0.5 * (tx1 + tx3)
    phases = [
        float(np.angle(np.conj(reference[rx]) * tx2[rx]))
        for rx in range(reference.size)
        if abs(reference[rx]) * abs(tx2[rx]) > 0
    ]
    if not phases:
        return None
    return -_phase_to_angle_deg(_circular_median(phases))


def _vertical_at(
    meta: dict,
    mti: np.ndarray,
    cal: Calibration,
    cand: Candidate,
    velocity_ms: float,
    *,
    agreement_deg: float,
) -> float | None:
    early_tx, late_tx, tau_s = _vertical_tx_indices_and_tau(meta)
    early = mti[cand.frame, cand.loop, early_tx, :, cand.range_bin]
    late = mti[cand.frame, cand.loop, late_tx, :, cand.range_bin] * np.exp(
        -1j * 4.0 * np.pi * velocity_ms * tau_s / doa.LAM
    )
    snap = np.concatenate((early, late), axis=0)[::-1]
    corrected = cal.apply(snap)
    noise = float(np.median(np.abs(mti) ** 2))
    theta = est_music_fbss_high(corrected, noise)
    if abs(theta - est_bartlett(corrected)) > math.radians(agreement_deg):
        return None
    return float(math.degrees(theta + cal.tilt_rad))


def _vertical_variants(
    meta: dict,
    mti: np.ndarray,
    cal: Calibration,
    cand: Candidate,
    velocity_ms: float,
) -> dict[str, float | None]:
    """Return several vertical DOA choices for club multipath diagnosis."""
    early_tx, late_tx, tau_s = _vertical_tx_indices_and_tau(meta)
    early = mti[cand.frame, cand.loop, early_tx, :, cand.range_bin]
    late = mti[cand.frame, cand.loop, late_tx, :, cand.range_bin] * np.exp(
        -1j * 4.0 * np.pi * velocity_ms * tau_s / doa.LAM
    )
    snap = cal.apply(np.concatenate((early, late), axis=0)[::-1])
    noise = float(np.median(np.abs(mti) ** 2))
    n = len(snap)
    m = n // 2 + 1
    p = n - m + 1
    covariance = np.zeros((m, m), dtype=complex)
    for i in range(p):
        xs = snap[i : i + m]
        covariance += np.outer(xs, xs.conj())
    covariance /= p
    exchange = np.eye(m)[::-1]
    covariance = 0.5 * (covariance + exchange @ covariance.conj() @ exchange)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    source_count = int(np.clip(np.sum(eigenvalues > 6.0 * noise), 1, m - 1))
    source_count = min(source_count, 2)
    noise_space = eigenvectors[:, : m - source_count]
    steering = _grid_steer(m)
    denom = np.sum(np.abs(noise_space.conj().T @ steering) ** 2, axis=0)
    spectrum = 1.0 / np.maximum(denom, 1e-12)
    interior = np.where((spectrum[1:-1] > spectrum[:-2]) & (spectrum[1:-1] > spectrum[2:]))[0] + 1
    if len(interior) == 0:
        peak_indices = np.asarray([int(np.argmax(spectrum))])
    else:
        peak_indices = interior[np.argsort(spectrum[interior])[::-1][: max(1, source_count)]]
    peak_angles = [float(math.degrees(GRID[idx] + cal.tilt_rad)) for idx in peak_indices]
    bartlett = float(math.degrees(est_bartlett(snap) + cal.tilt_rad))
    high = max(peak_angles)
    low = min(peak_angles)
    strong = peak_angles[0]
    # For a clubhead near the ball, the physically direct bearing should sit
    # near/slightly below zero in ground coordinates; this is a diagnostic pick.
    physical = min(peak_angles, key=lambda angle: abs(angle + 4.0))
    return {
        "v_high": high,
        "v_low": low,
        "v_strong": strong,
        "v_physical": physical,
        "v_bartlett": bartlett,
    }


def _fit_ridge(
    candidates: list[Candidate],
    *,
    expected_ms: float,
    tolerance_ms: float,
    iterations: int,
) -> tuple[float, float, list[Candidate], float, float] | None:
    if len(candidates) < 5:
        return None
    rng = random.Random(17)
    best: tuple[float, float, float, list[Candidate]] | None = None
    for _ in range(iterations):
        a, b = rng.sample(candidates, 2)
        dt = (b.time_ms - a.time_ms) / 1000.0
        if abs(dt) < 0.003:
            continue
        slope = (b.range_m - a.range_m) / dt
        if not -8.0 <= slope <= 55.0:
            continue
        intercept = a.range_m - slope * (a.time_ms / 1000.0)
        inliers = [
            c
            for c in candidates
            if abs(c.range_m - (slope * (c.time_ms / 1000.0) + intercept)) < 0.075
        ]
        if len(inliers) < 5:
            continue
        support = sum(math.log(max(c.snr, 1.0)) for c in inliers)
        prior = ((slope - expected_ms) / tolerance_ms) ** 2
        score = support - 1.5 * prior
        if best is None or score > best[0]:
            best = (score, slope, intercept, inliers)
    if best is None:
        return None
    _score, slope, intercept, inliers = best
    t = np.asarray([c.time_ms / 1000.0 for c in inliers])
    r = np.asarray([c.range_m for c in inliers])
    design = np.vstack([t, np.ones_like(t)]).T
    coeff, *_ = np.linalg.lstsq(design, r, rcond=None)
    slope, intercept = float(coeff[0]), float(coeff[1])
    residual = r - (design @ coeff)
    rms_m = float(np.sqrt(np.mean(residual**2)))
    prior_ratio = slope / expected_ms if abs(expected_ms) > 1e-6 else 0.0
    return slope, intercept, inliers, rms_m, prior_ratio


def _linear_delta(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 2:
        return None
    x = np.asarray(xs)
    y = np.asarray(ys)
    design = np.vstack([x, np.ones_like(x)]).T
    coeff, *_ = np.linalg.lstsq(design, y, rcond=None)
    return float(coeff[0] * (x.max() - x.min()))


def analyze_shot(
    row: ShotJoin,
    cal: Calibration,
    *,
    tee_m: float,
    freeze_delay_ms: float,
    pre_ms: float,
    post_ms: float,
    gate_half_m: float,
    top_k: int,
    snr_min: float,
    radial_fraction: float,
    radial_tolerance_ms: float,
    agreement_deg: float,
) -> dict[str, Any]:
    raw = row.dump.read_bytes()
    meta, cube = parse_dump(raw)
    if meta["n_tx"] not in (2, 3):
        return {"shot": row.shot, "status": f"unsupported_tx_count_{meta['n_tx']}"}
    geo = _geometry(meta)
    total_ms = geo.n_frames * geo.frame_period_s * 1000.0
    impact_t_ms = total_ms - freeze_delay_ms - row.iwr_trigger_delta_ms
    if row.ops_impact_ms is not None and row.ops_last_club_center_ms is not None:
        club_center_t_ms = impact_t_ms + (row.ops_last_club_center_ms - row.ops_impact_ms)
    else:
        club_center_t_ms = impact_t_ms - 2.0
    lo_t = club_center_t_ms - pre_ms
    hi_t = impact_t_ms + post_ms

    n_frames, chirps_per_frame, n_rx, n_samples = cube.shape
    loops = chirps_per_frame // meta["n_tx"]
    tdm = cube.reshape(n_frames, loops, meta["n_tx"], n_rx, n_samples)
    rfft = np.fft.fft(tdm, axis=-1)
    mti = rfft - rfft.mean(axis=1, keepdims=True)

    lo = max(2, int((tee_m - gate_half_m) / geo.range_res_m))
    hi = min(n_samples // 2 - 2, int((tee_m + gate_half_m) / geo.range_res_m))
    candidates: list[Candidate] = []
    for frame in range(geo.n_frames):
        for loop in range(geo.n_loops):
            time_ms = geo.loop_time(frame, loop) * 1000.0
            if not lo_t <= time_ms <= hi_t:
                continue
            power = np.sum(np.abs(mti[frame, loop, :, :, :]) ** 2, axis=(0, 1))
            gate = power[lo:hi]
            baseline = float(np.median(gate) + 1e-12)
            for rel in _local_peak_bins(gate, top_k=top_k):
                snr = float(gate[rel] / baseline)
                if snr < snr_min:
                    continue
                bin_float = _subbin(power, lo + rel, lo, hi)
                candidates.append(
                    Candidate(
                        frame=frame,
                        loop=loop,
                        time_ms=time_ms,
                        range_m=cal.true_range(bin_float * geo.range_res_m),
                        range_bin=int(round(bin_float)),
                        snr=snr,
                        weight=float(gate[rel]),
                    )
                )
    club_mph = row.ops_last_club_mph or row.ops_club_mph
    expected_ms = (club_mph or 75.0) / MPH_PER_MS * radial_fraction
    fit = _fit_ridge(
        candidates,
        expected_ms=expected_ms,
        tolerance_ms=radial_tolerance_ms,
        iterations=1200,
    )
    base = {
        "shot": row.shot,
        "ops_club_mph": row.ops_club_mph,
        "ops_last_club_mph": row.ops_last_club_mph,
        "ops_ball_mph": row.ops_ball_mph,
        "impact_t_ms": impact_t_ms,
        "club_center_t_ms": club_center_t_ms,
        "candidate_count": len(candidates),
        "live_h_deg": row.live_h_deg,
        "live_v_deg": row.live_v_deg,
        "n_tx": meta["n_tx"],
    }
    if fit is None:
        return {**base, "status": "ridge_fit_failed"}
    slope, _intercept, inliers, rms_m, prior_ratio = fit
    inliers = sorted(inliers, key=lambda c: c.time_ms)
    h_points: list[tuple[float, float]] = []
    v_points: list[tuple[float, float]] = []
    v_variant_points: dict[str, list[tuple[float, float]]] = {
        "v_high": [],
        "v_low": [],
        "v_strong": [],
        "v_physical": [],
        "v_bartlett": [],
    }
    for cand in inliers:
        if meta["n_tx"] == 3:
            h = _horizontal_at(mti, cand, slope)
            if h is not None:
                h_points.append((cand.time_ms, h))
        variants = _vertical_variants(meta, mti, cal, cand, slope)
        for key, value in variants.items():
            if value is not None:
                v_variant_points[key].append((cand.time_ms, value))
        v = _vertical_at(meta, mti, cal, cand, slope, agreement_deg=agreement_deg)
        if v is not None:
            v_points.append((cand.time_ms, v))

    h_delta = _linear_delta([p[0] for p in h_points], [p[1] for p in h_points])
    v_delta = _linear_delta([p[0] for p in v_points], [p[1] for p in v_points])
    variant_deltas = {
        f"{key}_delta_deg": _linear_delta([p[0] for p in points], [p[1] for p in points])
        for key, points in v_variant_points.items()
    }
    variant_medians = {
        f"{key}_median_deg": float(np.median([p[1] for p in points])) if points else None
        for key, points in v_variant_points.items()
    }
    h_phase = None
    h_coherence = None
    if h_points:
        h_phase, h_coherence = _weighted_circular_mean(
            [math.radians(p[1]) for p in h_points],
            [c.weight for c in inliers[: len(h_points)]],
        )
    status = "accepted"
    reasons: list[str] = []
    if len(inliers) < 6:
        reasons.append("few_range_inliers")
    if rms_m > 0.08:
        reasons.append("high_range_rms")
    if meta["n_tx"] == 3 and h_delta is None:
        reasons.append("missing_horizontal")
    if v_delta is None:
        reasons.append("missing_vertical")
    if reasons:
        status = "low_quality"

    return {
        **base,
        "status": status,
        "reasons": ";".join(reasons),
        "range_inliers": len(inliers),
        "range_rms_m": rms_m,
        "club_radial_mph": slope * MPH_PER_MS,
        "expected_radial_mph": expected_ms * MPH_PER_MS,
        "prior_ratio": prior_ratio,
        "ridge_t0_ms": inliers[0].time_ms,
        "ridge_t1_ms": inliers[-1].time_ms,
        "ridge_range0_m": inliers[0].range_m,
        "ridge_range1_m": inliers[-1].range_m,
        "ridge_range_delta_m": inliers[-1].range_m - inliers[0].range_m,
        "h_points": len(h_points),
        "h_first_deg": h_points[0][1] if h_points else None,
        "h_last_deg": h_points[-1][1] if h_points else None,
        "h_delta_deg": h_delta,
        "h_median_deg": float(np.median([p[1] for p in h_points])) if h_points else None,
        "h_coherence": h_coherence,
        "h_phase_rad": h_phase,
        "club_path_proxy_deg": (
            max(-12.0, min(12.0, h_delta * 0.55)) if h_delta is not None else None
        ),
        "v_points": len(v_points),
        "v_variant_points": len(v_variant_points["v_high"]),
        "v_first_deg": v_points[0][1] if v_points else None,
        "v_last_deg": v_points[-1][1] if v_points else None,
        "v_delta_deg": v_delta,
        "v_median_deg": float(np.median([p[1] for p in v_points])) if v_points else None,
        **variant_deltas,
        **variant_medians,
        "attack_proxy_deg": (
            max(-16.0, min(8.0, v_delta * 0.42)) if v_delta is not None else None
        ),
        "attack_high_proxy_deg": (
            max(-16.0, min(8.0, variant_deltas["v_high_delta_deg"] * 0.42))
            if variant_deltas["v_high_delta_deg"] is not None
            else None
        ),
        "attack_low_proxy_deg": (
            max(-16.0, min(8.0, variant_deltas["v_low_delta_deg"] * 0.42))
            if variant_deltas["v_low_delta_deg"] is not None
            else None
        ),
        "attack_physical_proxy_deg": (
            max(-16.0, min(8.0, variant_deltas["v_physical_delta_deg"] * 0.42))
            if variant_deltas["v_physical_delta_deg"] is not None
            else None
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--cal", default=DEFAULT_CAL_PATH)
    parser.add_argument("--tee-m", type=float, required=True)
    parser.add_argument("--freeze-delay-ms", type=float, default=50.0)
    parser.add_argument("--pre-ms", type=float, default=12.0)
    parser.add_argument("--post-ms", type=float, default=4.0)
    parser.add_argument("--gate-half-m", type=float, default=0.90)
    parser.add_argument("--top-k", type=int, default=4)
    parser.add_argument("--snr-min", type=float, default=8.0)
    parser.add_argument("--radial-fraction", type=float, default=0.45)
    parser.add_argument("--radial-tolerance-ms", type=float, default=14.0)
    parser.add_argument("--vertical-agreement-deg", type=float, default=16.0)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    cal = Calibration.load(args.cal)
    rows = _load_session(args.session.expanduser())
    output = [
        analyze_shot(
            row,
            cal,
            tee_m=args.tee_m,
            freeze_delay_ms=args.freeze_delay_ms,
            pre_ms=args.pre_ms,
            post_ms=args.post_ms,
            gate_half_m=args.gate_half_m,
            top_k=args.top_k,
            snr_min=args.snr_min,
            radial_fraction=args.radial_fraction,
            radial_tolerance_ms=args.radial_tolerance_ms,
            agreement_deg=args.vertical_agreement_deg,
        )
        for row in rows
    ]
    fields = [
        "shot",
        "status",
        "reasons",
        "n_tx",
        "ops_club_mph",
        "ops_last_club_mph",
        "ops_ball_mph",
        "club_radial_mph",
        "expected_radial_mph",
        "prior_ratio",
        "club_path_proxy_deg",
        "attack_proxy_deg",
        "attack_high_proxy_deg",
        "attack_low_proxy_deg",
        "attack_physical_proxy_deg",
        "h_delta_deg",
        "v_delta_deg",
        "v_high_delta_deg",
        "v_low_delta_deg",
        "v_strong_delta_deg",
        "v_physical_delta_deg",
        "v_bartlett_delta_deg",
        "h_median_deg",
        "v_median_deg",
        "v_high_median_deg",
        "v_low_median_deg",
        "v_strong_median_deg",
        "v_physical_median_deg",
        "v_bartlett_median_deg",
        "h_first_deg",
        "h_last_deg",
        "v_first_deg",
        "v_last_deg",
        "h_points",
        "v_points",
        "v_variant_points",
        "h_coherence",
        "range_inliers",
        "candidate_count",
        "range_rms_m",
        "ridge_t0_ms",
        "ridge_t1_ms",
        "impact_t_ms",
        "club_center_t_ms",
        "ridge_range0_m",
        "ridge_range1_m",
        "ridge_range_delta_m",
        "live_h_deg",
        "live_v_deg",
    ]
    target = args.out.open("w", newline="", encoding="utf-8") if args.out else sys.stdout
    try:
        writer = csv.DictWriter(target, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in output:
            writer.writerow({field: _fmt(row.get(field)) for field in fields})
    finally:
        if args.out:
            target.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
