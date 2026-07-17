#!/usr/bin/env python3
"""Measure moving-ball range asymmetry at predicted floor-path delays.

This audit does not estimate launch angle. It asks a narrower physics
question: after burst MTI, is there systematically more energy on the far
side of the direct ball return where the DG/GD and GG paths must appear?
The comparison uses the symmetric near-side sample as a local control for
the direct return's FFT main lobe and sidelobes.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import tm0714

from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.multipath import ground_path_geometry
from openflight.iwr6843.shot import geometry_from_header

HERE = Path(__file__).parent
CACHE_PATH = HERE / "cache_tm0714.npz"
TABLE_PATH = HERE / "cache_tm0714.json"
DEFAULT_OUTPUT = HERE / "range_path_audit.csv"


def _oversampled_mti(
    path: Path,
    *,
    n_fft: int,
    window: str,
) -> tuple[np.ndarray, object]:
    """Return burst-MTI range FFT as [frame, TX, loop, RX, range]."""
    meta, cube = parse_dump(path.read_bytes())
    geometry = geometry_from_header(meta)
    tdm = cube.reshape(
        geometry.n_frames,
        geometry.n_loops,
        2,
        geometry.n_rx,
        geometry.n_samples,
    ).transpose(0, 2, 1, 3, 4)
    tdm = tdm - tdm.mean(axis=2, keepdims=True)
    if window != "rectangular":
        windows = {
            "hann": np.hanning,
            "blackman": np.blackman,
        }
        win = windows[window](geometry.n_samples)
        tdm = tdm * win[None, None, None, None, :]
    return np.fft.fft(tdm, n=n_fft, axis=-1), geometry


def _sample(profile: np.ndarray, index: float) -> float:
    """Linearly sample a nonnegative range-power profile."""
    lo = int(np.floor(index))
    if lo < 0 or lo + 1 >= len(profile):
        return float("nan")
    fraction = index - lo
    return float((1.0 - fraction) * profile[lo] + fraction * profile[lo + 1])


def _asymmetry(profile: np.ndarray, center: float, offset: float) -> tuple[float, float]:
    farther = _sample(profile, center + offset)
    nearer = _sample(profile, center - offset)
    peak = _sample(profile, center)
    if not np.isfinite(farther + nearer + peak):
        return float("nan"), float("nan")
    epsilon = max(peak, 1.0) * 1e-12
    ratio_db = 10.0 * np.log10((farther + epsilon) / (nearer + epsilon))
    signed_peak_fraction = (farther - nearer) / (peak + epsilon)
    return float(ratio_db), float(signed_peak_fraction)


def _half_profile_asymmetry(
    profile: np.ndarray,
    center: float,
    *,
    oversample: float,
    max_offset: float,
) -> float:
    """Compare integrated far and near shoulders outside the peak center."""
    offsets = np.arange(0.35 * oversample, max_offset + 0.5, 0.5)
    farther = np.asarray([_sample(profile, center + value) for value in offsets])
    nearer = np.asarray([_sample(profile, center - value) for value in offsets])
    valid = np.isfinite(farther) & np.isfinite(nearer)
    if not np.any(valid):
        return float("nan")
    epsilon = max(_sample(profile, center), 1.0) * 1e-12
    return float(
        10.0 * np.log10((np.sum(farther[valid]) + epsilon) / (np.sum(nearer[valid]) + epsilon))
    )


def _median(values: list[float]) -> float:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    return float(np.median(finite)) if len(finite) else float("nan")


def _shot_metrics(
    cache: np.lib.npyio.NpzFile,
    indices: np.ndarray,
    shot: tm0714.TruthShot,
    rfft: np.ndarray,
    geometry: object,
    range_bias_m: float,
) -> dict[str, float]:
    oversample = rfft.shape[-1] / geometry.n_samples
    profiles = {
        "all": np.sum(np.abs(rfft) ** 2, axis=(1, 3)),
        "early": np.sum(np.abs(rfft[:, 0]) ** 2, axis=2),
        "late": np.sum(np.abs(rfft[:, 1]) ** 2, axis=2),
    }
    values: dict[str, list[float]] = {}
    for name in profiles:
        for metric in (
            "cross_asym_db",
            "cross_excess_fraction",
            "double_asym_db",
            "double_excess_fraction",
            "shoulder_asym_db",
            "peak_shift_cm",
        ):
            values[f"{name}_{metric}"] = []

    geometry_values: dict[str, list[float]] = {
        "cross_excess_cm": [],
        "double_excess_cm": [],
        "angle_separation_deg": [],
        "reflection_point_m": [],
    }
    native_res_m = geometry.range_res_m
    for index in indices:
        frame = int(cache["frame"][index])
        loop = int(cache["loop"][index])
        x_m = float(cache["x"][index])
        path = ground_path_geometry(
            x_m,
            shot.y_at(x_m),
            shot.z_at(x_m),
            shot.rh,
            np.radians(shot.tilt_deg),
        )
        center = (float(cache["r"][index]) + range_bias_m) / native_res_m * oversample
        cross_offset = path.cross_range_excess_m / native_res_m * oversample
        double_offset = path.double_range_excess_m / native_res_m * oversample
        geometry_values["cross_excess_cm"].append(100.0 * path.cross_range_excess_m)
        geometry_values["double_excess_cm"].append(100.0 * path.double_range_excess_m)
        geometry_values["angle_separation_deg"].append(
            np.degrees(path.direct_angle_rad - path.image_angle_rad)
        )
        geometry_values["reflection_point_m"].append(path.reflection_point_m)

        for name, power in profiles.items():
            profile = power[frame, loop]
            cross_db, cross_fraction = _asymmetry(profile, center, cross_offset)
            double_db, double_fraction = _asymmetry(profile, center, double_offset)
            shoulder_db = _half_profile_asymmetry(
                profile,
                center,
                oversample=oversample,
                max_offset=max(double_offset + 0.75 * oversample, 1.5 * oversample),
            )
            search_radius = max(int(np.ceil(double_offset + oversample)), int(oversample))
            center_bin = int(round(center))
            lo = max(center_bin - search_radius, 0)
            hi = min(center_bin + search_radius + 1, len(profile))
            peak_bin = lo + int(np.argmax(profile[lo:hi]))
            peak_shift_cm = (peak_bin - center) / oversample * native_res_m * 100.0
            values[f"{name}_cross_asym_db"].append(cross_db)
            values[f"{name}_cross_excess_fraction"].append(cross_fraction)
            values[f"{name}_double_asym_db"].append(double_db)
            values[f"{name}_double_excess_fraction"].append(double_fraction)
            values[f"{name}_shoulder_asym_db"].append(shoulder_db)
            values[f"{name}_peak_shift_cm"].append(peak_shift_cm)

    result = {f"median_{name}": _median(samples) for name, samples in values.items()}
    result.update({f"median_{name}": _median(samples) for name, samples in geometry_values.items()})
    return result


def _print_summary(rows: list[dict[str, object]]) -> None:
    print("\nRange-path asymmetry by block; positive means excess on far side")
    print("group       n  cross dB  GG dB  shoulder dB  peak shift  GG delay")
    groups = [("all", rows)]
    groups.extend(
        (f"block {block}", [row for row in rows if row["block"] == block]) for block in "ABC"
    )
    groups.extend(
        (
            f"9i {block}",
            [row for row in rows if row["block"] == block and str(row["club"]).startswith("9Iron")],
        )
        for block in "ABC"
    )
    for label, selected in groups:
        if not selected:
            continue
        print(
            f"{label:10s} {len(selected):2d} "
            f"{np.mean([row['median_all_cross_asym_db'] for row in selected]):+8.2f} "
            f"{np.mean([row['median_all_double_asym_db'] for row in selected]):+6.2f} "
            f"{np.mean([row['median_all_shoulder_asym_db'] for row in selected]):+11.2f} "
            f"{np.mean([row['median_all_peak_shift_cm'] for row in selected]):+9.2f}cm "
            f"{np.mean([row['median_double_excess_cm'] for row in selected]):7.2f}cm"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--n-fft", type=int, default=1024)
    parser.add_argument(
        "--window",
        choices=("rectangular", "hann", "blackman"),
        default="rectangular",
    )
    parser.add_argument("--max-range-m", type=float, default=4.70)
    parser.add_argument("--max-per-frame", type=int, default=4)
    args = parser.parse_args()

    cache = np.load(CACHE_PATH)
    table = {row["num"]: row for row in json.loads(TABLE_PATH.read_text())}
    shots = {shot.num: shot for shot in tm0714.load_shots()}
    calibration = tm0714.load_cal()
    rows: list[dict[str, object]] = []
    for shot_number, rec in sorted(table.items()):
        if rec.get("guided_ms") is None or shot_number not in shots:
            continue
        indices = tm0714.balanced_snapshot_indices(
            cache,
            shot_number,
            max_range_m=args.max_range_m,
            max_per_frame=args.max_per_frame,
        )
        frames = cache["frame"][indices]
        if len(indices) < 12 or len(np.unique(frames)) < 3:
            continue
        shot = shots[shot_number]
        rfft, geometry = _oversampled_mti(
            shot.file,
            n_fft=args.n_fft,
            window=args.window,
        )
        metrics = _shot_metrics(
            cache,
            indices,
            shot,
            rfft,
            geometry,
            calibration.range_bias_m,
        )
        rows.append(
            {
                "shot": shot_number,
                "club": rec["club"],
                "block": rec["block"],
                "n_snapshots": len(indices),
                "n_frames": len(np.unique(frames)),
                "window": args.window,
                "n_fft": args.n_fft,
                "trackman_launch_deg": rec["tm_la"],
                "trackman_ball_ms": shot.tm_ball_ms,
                "radar_height_m": rec["rh"],
                "tilt_deg": rec["tilt"],
                **metrics,
            }
        )
        print(f"shot {shot_number:03d} {rec['block']} {rec['club']}: {len(indices)} snapshots")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    _print_summary(rows)
    print(f"\nwrote {args.output} ({len(rows)} shots)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
