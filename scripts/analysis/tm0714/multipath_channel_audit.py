#!/usr/bin/env python3
"""Compare per-TX and four-path MIMO models on the TrackMan session.

Each candidate launch trajectory fixes the direct and floor-image angles for
every snapshot. Complex path amplitudes remain nuisance parameters. Models are
scored with leave-one-channel-out PRESS error, not in-sample explained energy,
so adding cross-path columns must improve prediction of unseen antenna channels
rather than merely consume more degrees of freedom.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import tm0714

from openflight.iwr6843.doa import TDM_TAU_S, later_physical_tx_index, validate_tx_order
from openflight.iwr6843.multipath import leave_one_channel_out_error
from openflight.iwr6843.music import LAM

HERE = Path(__file__).parent
CACHE_PATH = HERE / "cache_tm0714.npz"
TABLE_PATH = HERE / "cache_tm0714.json"
DEFAULT_OUTPUT = HERE / "multipath_channel_audit.csv"
MODELS = (
    "two8",
    "two8_path_tdm",
    "two8_path_tdm_neg",
    "four3",
    "four3_path_tdm",
    "four3_path_tdm_neg",
    "four4_path_tdm",
    "four4_path_tdm_neg",
    "tx_later4",
    "tx_earlier4",
)


def _trajectory(
    launch_rad: float,
    range_m: np.ndarray,
    rec: dict,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Truth-free candidate x, height, direct rate, and image rate."""
    return tm0714.candidate_trajectory_from_range(launch_rad, range_m, rec)


def _dictionary(
    model: str,
    launch_rad: float,
    range_m: np.ndarray,
    rec: dict,
) -> np.ndarray:
    tx_order = validate_tx_order(str(rec.get("tx_order", "normal")))
    x_m, height_m, direct_vr, image_vr = _trajectory(launch_rad, range_m, rec)
    tilt = np.radians(rec["tilt"])
    direct = np.arctan2(height_m - rec["rh"], x_m) - tilt
    image = np.arctan2(-(height_m + rec["rh"]), x_m) - tilt

    if model.startswith("tx_"):
        rx = np.arange(4, dtype=float)[None, :]
        return np.stack(
            [
                np.exp(1j * np.pi * np.sin(direct)[:, None] * rx),
                np.exp(1j * np.pi * np.sin(image)[:, None] * rx),
            ],
            axis=-1,
        )

    tx = np.repeat(np.array([0.0, 4.0]), 4)[None, :]
    rx = np.tile(np.arange(4, dtype=float), 2)[None, :]
    sin_direct = np.sin(direct)[:, None]
    sin_image = np.sin(image)[:, None]
    dd = np.exp(1j * np.pi * (tx * sin_direct + rx * sin_direct))
    dg = np.exp(1j * np.pi * (tx * sin_direct + rx * sin_image))
    gd = np.exp(1j * np.pi * (tx * sin_image + rx * sin_direct))
    gg = np.exp(1j * np.pi * (tx * sin_image + rx * sin_image))

    if "path_tdm" in model:
        polarity = -1.0 if model.endswith("_neg") else 1.0
        cross_phase = 2.0 * np.pi * (image_vr - direct_vr) * TDM_TAU_S / LAM
        later = later_physical_tx_index(tx_order)
        block = slice(4 * later, 4 * (later + 1))
        dg[:, block] *= np.exp(1j * polarity * cross_phase)[:, None]
        gd[:, block] *= np.exp(1j * polarity * cross_phase)[:, None]
        gg[:, block] *= np.exp(2j * polarity * cross_phase)[:, None]

    if model.startswith("two8"):
        return np.stack([dd, gg], axis=-1)
    if model.startswith("four3"):
        return np.stack([dd, dg + gd, gg], axis=-1)
    if model == "four4" or model.startswith("four4_path_tdm"):
        return np.stack([dd, dg, gd, gg], axis=-1)
    raise ValueError(f"unknown model: {model}")


def _model_snapshot(model: str, vectors: np.ndarray) -> np.ndarray:
    if model == "tx_later4":
        return vectors[:, :4]
    if model == "tx_earlier4":
        return vectors[:, 4:]
    return vectors


def _objective(
    model: str,
    launch_rad: float,
    range_m: np.ndarray,
    vectors: np.ndarray,
    frames: np.ndarray,
    rec: dict,
) -> float:
    dictionary = _dictionary(model, launch_rad, range_m, rec)
    errors = leave_one_channel_out_error(_model_snapshot(model, vectors), dictionary)
    # Give each frame one vote and limit a single pathological loop's leverage.
    frame_errors = [
        np.median(np.clip(errors[frames == frame], 1e-6, 1e3)) for frame in np.unique(frames)
    ]
    return float(np.mean(np.log(frame_errors)))


def _refine_grid(grid_deg: np.ndarray, objective: np.ndarray) -> float:
    index = int(np.argmin(objective))
    if not 0 < index < len(grid_deg) - 1:
        return float(grid_deg[index])
    y0, y1, y2 = objective[index - 1 : index + 2]
    denominator = y0 - 2.0 * y1 + y2
    offset = 0.5 * (y0 - y2) / denominator if denominator > 0 else 0.0
    return float(grid_deg[index] + np.clip(offset, -1.0, 1.0) * (grid_deg[1] - grid_deg[0]))


def _geometry_features(x_m: np.ndarray, shot: tm0714.TruthShot) -> dict[str, float]:
    z_m = np.asarray([shot.z_at(x) for x in x_m])
    y_m = np.asarray([shot.y_at(x) for x in x_m])
    horizontal = np.hypot(x_m, y_m)
    direct_range = np.hypot(horizontal, z_m - shot.rh)
    image_range = np.hypot(horizontal, z_m + shot.rh)
    angle_separation = np.degrees(
        np.arctan2(z_m - shot.rh, horizontal) - np.arctan2(-(z_m + shot.rh), horizontal)
    )
    reflection = horizontal * shot.rh / np.maximum(shot.rh + z_m, 1e-9)
    return {
        "median_cross_excess_cm": 50.0 * float(np.median(image_range - direct_range)),
        "median_double_excess_cm": 100.0 * float(np.median(image_range - direct_range)),
        "median_angle_separation_deg": float(np.median(angle_separation)),
        "median_reflection_point_m": float(np.median(reflection)),
    }


def _print_summary(rows: list[dict[str, object]]) -> None:
    print("\nRaw launch error by block; no club or block offsets")
    print("model               block   n    MAE    bias  medianAE  truth-CV")
    for model in MODELS:
        for block in "ABC":
            selected = [row for row in rows if row["model"] == model and row["block"] == block]
            if not selected:
                continue
            errors = np.asarray([row["launch_error_deg"] for row in selected], dtype=float)
            truth_cv = np.asarray([row["truth_cv_log_error"] for row in selected], dtype=float)
            print(
                f"{model:19s} {block:>3s} {len(selected):3d} "
                f"{np.mean(np.abs(errors)):6.2f} {np.mean(errors):+7.2f} "
                f"{np.median(np.abs(errors)):9.2f} {np.mean(truth_cv):+9.3f}"
            )

    print("\nSame-club geometry comparison: 9i only")
    print("model               block   n    MAE    bias  angle-sep  GG-delay")
    for model in MODELS:
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
            print(
                f"{model:19s} {block:>3s} {len(selected):3d} "
                f"{np.mean(np.abs(errors)):6.2f} {np.mean(errors):+7.2f} "
                f"{np.mean([row['median_angle_separation_deg'] for row in selected]):9.2f} "
                f"{np.mean([row['median_double_excess_cm'] for row in selected]):7.2f}cm"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--table", type=Path, default=TABLE_PATH)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--max-range-m", type=float, default=4.70)
    parser.add_argument("--max-per-frame", type=int, default=4)
    parser.add_argument("--grid-step-deg", type=float, default=0.5)
    parser.add_argument(
        "--velocity-source",
        choices=("truth", "radar"),
        default="truth",
        help="velocity used only for direct-path TDM phase correction",
    )
    parser.add_argument(
        "--speed-source",
        choices=("ops", "radar"),
        default="ops",
        help="truth-independent total ball speed used by candidate trajectories",
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
        frames = cache["frame"][indices]
        if len(indices) < 12 or len(np.unique(frames)) < 3:
            continue
        vectors = tm0714.fixed_positive_vectors(
            cache,
            indices,
            table,
            shots,
            calibration.elem_correction,
            truth_velocity=args.velocity_source == "truth",
        )
        range_m = cache["r"][indices]
        features = _geometry_features(cache["x"][indices], shots[shot_number])
        for model in MODELS:
            objective = np.asarray(
                [
                    _objective(model, np.radians(angle), range_m, vectors, frames, rec)
                    for angle in grid_deg
                ]
            )
            estimate = _refine_grid(grid_deg, objective)
            truth_objective = _objective(
                model,
                np.radians(rec["tm_la"]),
                range_m,
                vectors,
                frames,
                rec,
            )
            rows.append(
                {
                    "shot": shot_number,
                    "club": rec["club"],
                    "block": rec["block"],
                    "model": model,
                    "velocity_source": args.velocity_source,
                    "speed_source": args.speed_source,
                    "n_snapshots": len(indices),
                    "n_frames": len(np.unique(frames)),
                    "launch_angle_deg": estimate,
                    "trackman_launch_deg": rec["tm_la"],
                    "trackman_ball_ms": shots[shot_number].tm_ball_ms,
                    "radar_ball_ms": rec["guided_ms"],
                    "launch_error_deg": estimate - rec["tm_la"],
                    "best_cv_log_error": float(np.min(objective)),
                    "truth_cv_log_error": truth_objective,
                    "truth_minus_best_cv": truth_objective - float(np.min(objective)),
                    "radar_height_m": rec["rh"],
                    "tilt_deg": rec["tilt"],
                    "tee_distance_m": rec["x_tee"],
                    **features,
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
