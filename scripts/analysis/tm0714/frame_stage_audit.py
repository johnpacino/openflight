#!/usr/bin/env python3
"""Audit early/middle/late IWR6843 frames against TrackMan truth.

This is a mechanism diagnostic, not a production estimator. It converts the
historical auto-sign snapshot cache to the fixed-positive convention and runs
two TDM corrections: range-walk local velocity and TrackMan truth velocity.
For each shot, anchored launch fits from the first, middle, and last three
valid chronological frames are compared with a truth-height fit over the same
samples. The latter isolates array/two-ray model bias from gravity and range
coverage.

Run ``extract_tm0714.py`` first to create ``cache_tm0714.npz`` and JSON.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import tm0714

from openflight.iwr6843.music import est_bartlett, steer
from openflight.iwr6843.trajectory import _two_ray_solve

HERE = Path(__file__).parent
CACHE_PATH = HERE / "cache_tm0714.npz"
TABLE_PATH = HERE / "cache_tm0714.json"
GRID_M = np.arange(-0.02, 1.30, 0.01)


def _stage_indices(
    cache: np.lib.npyio.NpzFile,
    shot_number: int,
    *,
    max_range_m: float,
) -> dict[str, np.ndarray]:
    """First/middle/last three valid frames in chronological order."""
    shot_indices = np.nonzero(cache["shot"] == shot_number)[0]
    frames: list[tuple[float, np.ndarray]] = []
    for frame in np.unique(cache["frame"][shot_indices]):
        indices = shot_indices[cache["frame"][shot_indices] == frame]
        valid = indices[(cache["snr"][indices] >= 8.0) & (cache["r"][indices] <= max_range_m)]
        if len(valid) >= 4:
            frames.append((float(np.median(cache["t"][valid])), valid))
    frames.sort(key=lambda item: item[0])
    if len(frames) < 3:
        return {}
    middle = max(0, min(len(frames) - 3, len(frames) // 2 - 1))
    selected = {
        "first3": frames[:3],
        "middle3": frames[middle : middle + 3],
        "last3": frames[-3:],
    }
    return {
        stage: np.concatenate([indices for _time, indices in group])
        for stage, group in selected.items()
    }


def _anchored_angle(
    x_m: np.ndarray,
    height_m: np.ndarray,
    weights: np.ndarray,
    *,
    x_tee_m: float,
    ball_height_m: float,
) -> float:
    dx = x_m - x_tee_m
    denominator = float(np.sum(weights * dx * dx))
    slope = float(np.sum(weights * dx * (height_m - ball_height_m)) / denominator)
    return float(np.degrees(np.arctan(slope)))


def _group_metrics(
    cache: np.lib.npyio.NpzFile,
    indices: np.ndarray,
    table: dict[int, dict],
    shots: dict[int, tm0714.TruthShot],
    element_correction: np.ndarray,
    *,
    truth_velocity: bool,
    max_range_m: float,
) -> dict[str, float] | None:
    vectors = tm0714.fixed_positive_vectors(
        cache,
        indices,
        table,
        shots,
        element_correction,
        truth_velocity=truth_velocity,
    )
    number = int(cache["shot"][indices[0]])
    record = table[number]
    values: list[dict[str, float]] = []
    for index, vector in zip(indices, vectors):
        if cache["snr"][index] < 8.0 or cache["r"][index] > max_range_m:
            continue
        height, explained, _image_fraction = _two_ray_solve(
            vector,
            float(cache["r"][index]),
            record["rh"],
            np.radians(record["tilt"]),
            GRID_M,
        )
        if explained < 0.70:
            continue
        truth_basis = np.column_stack(
            [steer(cache["th_d"][index], 8), steer(cache["th_i"][index], 8)]
        )
        coefficients, *_ = np.linalg.lstsq(truth_basis, vector, rcond=None)
        prediction = truth_basis @ coefficients
        truth_explained = 1.0 - float(
            np.vdot(vector - prediction, vector - prediction).real / np.vdot(vector, vector).real
        )
        values.append(
            {
                "range_m": float(cache["r"][index]),
                "height_m": height,
                "truth_height_m": float(cache["z"][index]),
                "weight": explained - 0.70 + 1e-3,
                "truth_direct_deg": float(np.degrees(cache["th_d"][index])),
                "direct_image_sep_deg": float(
                    np.degrees(cache["th_d"][index] - cache["th_i"][index])
                ),
                "truth_explained": truth_explained,
                "best_explained": explained,
                "bartlett_deg": float(np.degrees(est_bartlett(vector))),
            }
        )
    if len(values) < 6:
        return None
    x_m = np.asarray([value["range_m"] for value in values])
    solved_height = np.asarray([value["height_m"] for value in values])
    truth_height = np.asarray([value["truth_height_m"] for value in values])
    weights = np.asarray([value["weight"] for value in values])
    solve_angle = _anchored_angle(
        x_m,
        solved_height,
        weights,
        x_tee_m=record["x_tee"],
        ball_height_m=record["z0"],
    )
    truth_angle = _anchored_angle(
        x_m,
        truth_height,
        weights,
        x_tee_m=record["x_tee"],
        ball_height_m=record["z0"],
    )
    direct_angles = np.asarray([value["truth_direct_deg"] for value in values])
    return {
        "n_snapshots": len(values),
        "launch_angle_deg": solve_angle,
        "trackman_launch_deg": float(record["tm_la"]),
        "launch_error_deg": solve_angle - float(record["tm_la"]),
        "sampled_truth_angle_deg": truth_angle,
        "model_bias_deg": solve_angle - truth_angle,
        "height_error_cm": 100.0 * float(np.median(solved_height - truth_height)),
        "truth_direct_deg": float(np.median(direct_angles)),
        "direct_image_sep_deg": float(
            np.median([value["direct_image_sep_deg"] for value in values])
        ),
        "truth_explained": float(np.median([value["truth_explained"] for value in values])),
        "best_explained": float(np.median([value["best_explained"] for value in values])),
        "truth_corrupt_zone_pct": 100.0 * float(np.mean(direct_angles > -2.5)),
        "bartlett_gate_pct": 100.0
        * float(np.mean([value["bartlett_deg"] > -2.5 for value in values])),
    }


def _print_summary(rows: list[dict[str, object]], min_trackman_angle: float) -> None:
    print(f"\nEqual-shot summary (TrackMan launch >= {min_trackman_angle:g} deg)")
    print("mode  club stage     n  LA error mean/median  model bias  height err  truth theta")
    for mode in ("local", "truth"):
        for club in ("9Iron", "7Iron"):
            for stage in ("first3", "middle3", "last3"):
                selected = [
                    row
                    for row in rows
                    if row["mode"] == mode
                    and str(row["club"]).startswith(club)
                    and row["stage"] == stage
                    and float(row["trackman_launch_deg"]) >= min_trackman_angle
                ]
                if not selected:
                    continue
                errors = np.asarray([row["launch_error_deg"] for row in selected], dtype=float)
                print(
                    f"{mode:5s} {club[:2]:4s} {stage:8s} {len(selected):2d} "
                    f"{errors.mean():+7.2f}/{np.median(errors):+6.2f} "
                    f"{np.mean([row['model_bias_deg'] for row in selected]):+10.2f} "
                    f"{np.mean([row['height_error_cm'] for row in selected]):+9.1f} cm "
                    f"{np.mean([row['truth_direct_deg'] for row in selected]):+8.1f} deg"
                )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--max-range-m", type=float, default=4.70)
    parser.add_argument("--min-trackman-angle", type=float, default=10.0)
    parser.add_argument("--output", type=Path, default=HERE / "frame_stage_audit.csv")
    args = parser.parse_args()

    if not CACHE_PATH.exists() or not TABLE_PATH.exists():
        parser.error("run extract_tm0714.py first")
    cache = np.load(CACHE_PATH)
    table = {row["num"]: row for row in json.loads(TABLE_PATH.read_text())}
    calibration = tm0714.load_cal()
    shots = {shot.num: shot for shot in tm0714.load_shots()}
    rows: list[dict[str, object]] = []
    for number, record in sorted(table.items()):
        if record["block"] != "A" or not str(record["club"]).startswith(("9Iron", "7Iron")):
            continue
        for stage, indices in _stage_indices(cache, number, max_range_m=args.max_range_m).items():
            for mode, truth_velocity in (("local", False), ("truth", True)):
                metrics = _group_metrics(
                    cache,
                    indices,
                    table,
                    shots,
                    calibration.elem_correction,
                    truth_velocity=truth_velocity,
                    max_range_m=args.max_range_m,
                )
                if metrics is not None:
                    rows.append(
                        {
                            "shot": number,
                            "club": record["club"],
                            "stage": stage,
                            "mode": mode,
                            **metrics,
                        }
                    )
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    _print_summary(rows, args.min_trackman_angle)
    print(f"\nwrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
