#!/usr/bin/env python3
"""Jointly fit moving-ball antenna phase and floor-path range delay.

The channel audit samples only the tracked direct range bin. This audit keeps
the calibrated complex FFT bins around the ball and gives DD, DG, GD, and GG
their physically predicted apparent ranges. It therefore tests whether range
delay resolves coherent floor multipath better than angle-only processing.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import multipath_channel_audit as channel_audit
import numpy as np
import tm0714

from openflight.iwr6843.doa import TDM_TAU_S, canonicalize_tx_blocks, validate_tx_order
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.music import LAM
from openflight.iwr6843.shot import geometry_from_header

HERE = Path(__file__).parent
CACHE_PATH = HERE / "cache_tm0714.npz"
TABLE_PATH = HERE / "cache_tm0714.json"
DEFAULT_OUTPUT = HERE / "fast_time_multipath_audit.csv"
MODELS = ("direct1", "two2", "four3", "four4")


def _prepared_fft(
    path: Path,
    indices: np.ndarray,
    cache: np.lib.npyio.NpzFile,
    element_correction: np.ndarray,
    *,
    n_fft: int,
    window: str,
    tx_order: str = "normal",
    tdm_sign: int = 1,
) -> tuple[np.ndarray, object, np.ndarray]:
    """Return calibrated fixed-positive FFT snapshots and window samples."""
    tx_order = validate_tx_order(tx_order)
    if tdm_sign not in (-1, 1):
        raise ValueError("tdm_sign must be -1 or +1")
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
    if window == "rectangular":
        win = np.ones(geometry.n_samples)
    else:
        windows = {"hann": np.hanning, "blackman": np.blackman}
        win = windows[window](geometry.n_samples)

    snapshots = []
    phase_per_ms = 4.0 * np.pi * TDM_TAU_S / LAM
    for index in indices:
        frame = int(cache["frame"][index])
        loop = int(cache["loop"][index])
        snapshot = canonicalize_tx_blocks(
            tdm[frame, 0, loop],
            tdm[frame, 1, loop],
            tdm_phase=tdm_sign * phase_per_ms * cache["vr"][index],
            tx_order=tx_order,
        )
        snapshot *= element_correction[:, None]
        snapshots.append(np.fft.fft(snapshot * win[None, :], n=n_fft, axis=-1))
    return np.asarray(snapshots), geometry, win


def _quadratic_peak(profile: np.ndarray, center: float, radius: int) -> float:
    center_bin = int(round(center))
    lo = max(center_bin - radius, 1)
    hi = min(center_bin + radius + 1, len(profile) - 1)
    peak = lo + int(np.argmax(profile[lo:hi]))
    y0, y1, y2 = profile[peak - 1 : peak + 2]
    denominator = y0 - 2.0 * y1 + y2
    offset = 0.5 * (y0 - y2) / denominator if denominator < 0 else 0.0
    return float(peak + np.clip(offset, -1.0, 1.0))


def _center_bins(
    data_fft: np.ndarray,
    track_bins: np.ndarray,
    *,
    center_source: str,
    oversample: float,
) -> np.ndarray:
    if center_source == "track":
        return track_bins
    power = np.sum(np.abs(data_fft) ** 2, axis=1)
    return np.asarray(
        [
            _quadratic_peak(profile, center, int(np.ceil(oversample)))
            for profile, center in zip(power, track_bins, strict=True)
        ]
    )


def _local_data(
    data_fft: np.ndarray,
    center_bins: np.ndarray,
    offsets: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    bins = np.rint(center_bins).astype(int)[:, None] + offsets[None, :]
    if np.any(bins < 0) or np.any(bins >= data_fft.shape[-1]):
        raise ValueError("local range window exceeds FFT bounds")
    selected = np.take_along_axis(data_fft, bins[:, None, :], axis=2)
    return selected, bins


def _design(
    model: str,
    launch_rad: float,
    range_m: np.ndarray,
    center_native_bins: np.ndarray,
    selected_bins: np.ndarray,
    rec: dict,
    *,
    n_samples: int,
    n_fft: int,
    range_res_m: float,
    window: np.ndarray,
    tdm_residual: str,
) -> np.ndarray:
    tdm_model = {
        "none": "four4",
        "positive": "four4_path_tdm",
        "negative": "four4_path_tdm_neg",
    }[tdm_residual]
    spatial = channel_audit._dictionary(tdm_model, launch_rad, range_m, rec)
    x_m, height_m, _direct_vr, _image_vr = tm0714.candidate_trajectory_from_range(
        launch_rad,
        range_m,
        rec,
    )
    direct_distance = np.hypot(x_m, height_m - rec["rh"])
    image_distance = np.hypot(x_m, height_m + rec["rh"])
    delta_bins = (image_distance - direct_distance) / range_res_m
    path_bins = center_native_bins[:, None] + np.stack(
        [
            np.zeros_like(delta_bins),
            0.5 * delta_bins,
            0.5 * delta_bins,
            delta_bins,
        ],
        axis=1,
    )
    sample = np.arange(n_samples)
    tones = np.exp(2j * np.pi * path_bins[:, :, None] * sample / n_samples)
    response = np.fft.fft(tones * window[None, None, :], n=n_fft, axis=-1)
    response = np.take_along_axis(response, selected_bins[:, None, :], axis=2)
    design = spatial[:, :, None, :] * response.transpose(0, 2, 1)[:, None, :, :]
    if model == "direct1":
        return design[..., [0]]
    if model == "two2":
        return design[..., [0, 3]]
    if model == "four3":
        return np.stack(
            [design[..., 0], design[..., 1] + design[..., 2], design[..., 3]],
            axis=-1,
        )
    return design


def _fit_error(data: np.ndarray, design: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    flattened_data = data.reshape(len(data), -1)
    flattened_design = design.reshape(len(data), -1, design.shape[-1])
    pseudo = np.linalg.pinv(flattened_design)
    coefficients = np.einsum("...kn,...n->...k", pseudo, flattened_data)
    prediction = np.einsum("...nk,...k->...n", flattened_design, coefficients)
    residual = np.sum(np.abs(flattened_data - prediction) ** 2, axis=1)
    power = np.sum(np.abs(flattened_data) ** 2, axis=1) + 1e-12
    return residual / power, coefficients


def _objective(errors: np.ndarray, frames: np.ndarray) -> float:
    frame_errors = [
        np.median(np.clip(errors[frames == frame], 1e-8, 1.0)) for frame in np.unique(frames)
    ]
    return float(np.mean(np.log(frame_errors)))


def _path_ratios(coefficients: np.ndarray, model: str) -> tuple[float, float]:
    direct = np.maximum(np.abs(coefficients[:, 0]), 1e-12)
    if model == "direct1":
        return float("nan"), float("nan")
    if model == "two2":
        return float("nan"), float(np.median(np.abs(coefficients[:, 1]) / direct))
    if model == "four3":
        return (
            float(np.median(np.abs(coefficients[:, 1]) / direct)),
            float(np.median(np.abs(coefficients[:, 2]) / direct)),
        )
    cross = 0.5 * (np.abs(coefficients[:, 1]) + np.abs(coefficients[:, 2]))
    return (
        float(np.median(cross / direct)),
        float(np.median(np.abs(coefficients[:, 3]) / direct)),
    )


def _finite_mean(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.mean(finite)) if len(finite) else float("nan")


def _print_summary(rows: list[dict[str, object]]) -> None:
    print("\nJoint channel/range launch error; no club or block offsets")
    print("model    block   n    MAE    bias  medianAE  truth-fit")
    models = list(dict.fromkeys(str(row["model"]) for row in rows))
    for model in models:
        for block in "ABC":
            selected = [row for row in rows if row["model"] == model and row["block"] == block]
            if not selected:
                continue
            errors = np.asarray([row["launch_error_deg"] for row in selected], dtype=float)
            print(
                f"{model:8s} {block:>3s} {len(selected):3d} "
                f"{np.mean(np.abs(errors)):6.2f} {np.mean(errors):+7.2f} "
                f"{np.median(np.abs(errors)):9.2f} "
                f"{np.mean([row['truth_log_error'] for row in selected]):+9.3f}"
            )

    print("\nSame-club geometry comparison: 9i only")
    print("model    block   n    MAE    bias  cross/DD   GG/DD")
    for model in models:
        for block in "ABC":
            selected = [
                row
                for row in rows
                if row["model"] == model
                and row["block"] == block
                and str(row["club"]).startswith("9Iron")
            ]
            if not selected:
                continue
            errors = np.asarray([row["launch_error_deg"] for row in selected], dtype=float)
            cross = np.asarray([row["median_cross_to_direct"] for row in selected], dtype=float)
            double = np.asarray([row["median_double_to_direct"] for row in selected], dtype=float)
            print(
                f"{model:8s} {block:>3s} {len(selected):3d} "
                f"{np.mean(np.abs(errors)):6.2f} {np.mean(errors):+7.2f} "
                f"{_finite_mean(cross):9.2f} {_finite_mean(double):7.2f}"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--table", type=Path, default=TABLE_PATH)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--n-fft", type=int, default=512)
    parser.add_argument(
        "--window",
        choices=("rectangular", "hann", "blackman"),
        default="hann",
    )
    parser.add_argument("--center-source", choices=("track", "peak"), default="track")
    parser.add_argument(
        "--tdm-residual",
        choices=("none", "positive", "negative"),
        default="none",
    )
    parser.add_argument(
        "--speed-source",
        choices=("ops", "radar"),
        default="ops",
        help="truth-independent total ball speed used by candidate trajectories",
    )
    parser.add_argument("--max-range-m", type=float, default=4.70)
    parser.add_argument("--max-per-frame", type=int, default=4)
    parser.add_argument("--grid-step-deg", type=float, default=0.5)
    parser.add_argument("--models", nargs="+", choices=MODELS, default=list(MODELS))
    parser.add_argument(
        "--snapshot-half",
        choices=("all", "first", "second"),
        default="all",
    )
    args = parser.parse_args()

    cache = np.load(CACHE_PATH)
    table = {row["num"]: row for row in json.loads(args.table.read_text())}
    shots = {shot.num: shot for shot in tm0714.load_shots()}
    calibration = tm0714.load_cal()
    grid_deg = np.arange(-5.0, 45.0 + args.grid_step_deg / 2.0, args.grid_step_deg)
    rows: list[dict[str, object]] = []
    for shot_number, rec in sorted(table.items()):
        if rec.get("guided_ms") is None or shot_number not in shots:
            continue
        rec = dict(rec)
        rec["candidate_speed_ms"] = tm0714.independent_ball_speed_ms(
            rec,
            args.speed_source,
        )
        indices = tm0714.balanced_snapshot_indices(
            cache,
            shot_number,
            max_range_m=args.max_range_m,
            max_per_frame=args.max_per_frame,
        )
        indices = indices[np.argsort(cache["t"][indices])]
        if args.snapshot_half != "all":
            midpoint = len(indices) // 2
            indices = indices[:midpoint] if args.snapshot_half == "first" else indices[midpoint:]
        frames = cache["frame"][indices]
        min_snapshots = 12 if args.snapshot_half == "all" else 6
        min_frames = 3 if args.snapshot_half == "all" else 2
        if len(indices) < min_snapshots or len(np.unique(frames)) < min_frames:
            continue
        data_fft, geometry, window = _prepared_fft(
            shots[shot_number].file,
            indices,
            cache,
            calibration.elem_correction,
            n_fft=args.n_fft,
            window=args.window,
            tx_order=str(rec.get("tx_order", "normal")),
        )
        oversample = args.n_fft / geometry.n_samples
        range_m = cache["r"][indices]
        track_bins = (range_m + calibration.range_bias_m) / geometry.range_res_m * oversample
        centers = _center_bins(
            data_fft,
            track_bins,
            center_source=args.center_source,
            oversample=oversample,
        )
        local_offsets = np.arange(
            -int(np.ceil(2.0 * oversample)),
            int(np.ceil(6.0 * oversample)) + 1,
        )
        local_data, selected_bins = _local_data(data_fft, centers, local_offsets)
        center_native = centers / oversample
        for model in args.models:
            objective = []
            for angle_deg in grid_deg:
                design = _design(
                    model,
                    np.radians(angle_deg),
                    range_m,
                    center_native,
                    selected_bins,
                    rec,
                    n_samples=geometry.n_samples,
                    n_fft=args.n_fft,
                    range_res_m=geometry.range_res_m,
                    window=window,
                    tdm_residual=args.tdm_residual,
                )
                errors, _coefficients = _fit_error(local_data, design)
                objective.append(_objective(errors, frames))
            objective = np.asarray(objective)
            estimate = channel_audit._refine_grid(grid_deg, objective)
            best_design = _design(
                model,
                np.radians(estimate),
                range_m,
                center_native,
                selected_bins,
                rec,
                n_samples=geometry.n_samples,
                n_fft=args.n_fft,
                range_res_m=geometry.range_res_m,
                window=window,
                tdm_residual=args.tdm_residual,
            )
            _best_errors, coefficients = _fit_error(local_data, best_design)
            cross_ratio, double_ratio = _path_ratios(coefficients, model)
            truth_design = _design(
                model,
                np.radians(rec["tm_la"]),
                range_m,
                center_native,
                selected_bins,
                rec,
                n_samples=geometry.n_samples,
                n_fft=args.n_fft,
                range_res_m=geometry.range_res_m,
                window=window,
                tdm_residual=args.tdm_residual,
            )
            truth_errors, _truth_coefficients = _fit_error(local_data, truth_design)
            rows.append(
                {
                    "shot": shot_number,
                    "club": rec["club"],
                    "block": rec["block"],
                    "model": model,
                    "window": args.window,
                    "center_source": args.center_source,
                    "tdm_residual": args.tdm_residual,
                    "speed_source": args.speed_source,
                    "snapshot_half": args.snapshot_half,
                    "max_range_m": args.max_range_m,
                    "n_snapshots": len(indices),
                    "n_frames": len(np.unique(frames)),
                    "launch_angle_deg": estimate,
                    "trackman_launch_deg": rec["tm_la"],
                    "launch_error_deg": estimate - rec["tm_la"],
                    "best_log_error": float(np.min(objective)),
                    "truth_log_error": _objective(truth_errors, frames),
                    "truth_minus_best": _objective(truth_errors, frames) - float(np.min(objective)),
                    "median_cross_to_direct": cross_ratio,
                    "median_double_to_direct": double_ratio,
                    "trackman_ball_ms": shots[shot_number].tm_ball_ms,
                    "radar_ball_ms": rec["guided_ms"],
                    "radar_height_m": rec["rh"],
                    "tilt_deg": rec["tilt"],
                }
            )
        print(f"shot {shot_number:03d} {rec['block']} {rec['club']}: {len(indices)} snapshots")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    _print_summary(rows)
    print(f"\nwrote {args.output} ({len(rows)} model-shot rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
