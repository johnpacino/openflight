#!/usr/bin/env python3
"""Quantify what Blocks A/B/C can and cannot say about mount geometry."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.stats import spearmanr

HERE = Path(__file__).parent
DEFAULT_CHANNEL = HERE / "multipath_channel_audit_production.csv"
DEFAULT_RANGE = HERE / "range_path_audit_hann.csv"
DEFAULT_OUTPUT = HERE / "block_geometry_audit.json"
MODELS = ("two8", "four3", "four4_path_tdm")


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def _selected(
    rows: list[dict[str, str]],
    model: str,
    block: str,
) -> list[dict[str, str]]:
    return [
        row
        for row in rows
        if row["model"] == model and row["block"] == block and row["club"].startswith("9Iron")
    ]


def _summary(rows: list[dict[str, str]]) -> dict[str, float | int]:
    errors = np.asarray([float(row["launch_error_deg"]) for row in rows])
    return {
        "n": len(errors),
        "mae_deg": float(np.mean(np.abs(errors))),
        "bias_deg": float(np.mean(errors)),
        "median_ae_deg": float(np.median(np.abs(errors))),
        "radar_height_m": float(np.mean([float(row["radar_height_m"]) for row in rows])),
        "tilt_deg": float(np.mean([float(row["tilt_deg"]) for row in rows])),
        "gg_delay_cm": float(np.mean([float(row["median_double_excess_cm"]) for row in rows])),
        "angle_separation_deg": float(
            np.mean([float(row["median_angle_separation_deg"]) for row in rows])
        ),
    }


def _exact_permutation(
    first: np.ndarray,
    second: np.ndarray,
) -> float:
    """Two-sided exact randomization p-value for a difference in MAE."""
    values = np.concatenate([first, second])
    n_first = len(first)
    observed = abs(np.mean(np.abs(first)) - np.mean(np.abs(second)))
    extreme = 0
    total = 0
    all_indices = np.arange(len(values))
    for chosen in itertools.combinations(all_indices, n_first):
        mask = np.zeros(len(values), dtype=bool)
        mask[list(chosen)] = True
        difference = abs(np.mean(np.abs(values[mask])) - np.mean(np.abs(values[~mask])))
        extreme += difference >= observed - 1e-12
        total += 1
    return extreme / total


def _bootstrap_difference(
    first: np.ndarray,
    second: np.ndarray,
    *,
    iterations: int = 50_000,
) -> tuple[float, float]:
    rng = np.random.default_rng(20260716)
    first_samples = rng.choice(first, size=(iterations, len(first)), replace=True)
    second_samples = rng.choice(second, size=(iterations, len(second)), replace=True)
    differences = np.mean(np.abs(first_samples), axis=1) - np.mean(np.abs(second_samples), axis=1)
    low, high = np.quantile(differences, [0.025, 0.975])
    return float(low), float(high)


def _matched_difference(
    first: list[dict[str, str]],
    second: list[dict[str, str]],
) -> dict[str, float | int]:
    combined = first + second
    features = np.asarray(
        [[float(row["trackman_launch_deg"]), float(row["trackman_ball_ms"])] for row in combined]
    )
    scale = np.std(features, axis=0, ddof=1)
    scale[scale < 1e-9] = 1.0
    first_features = features[: len(first)] / scale
    second_features = features[len(first) :] / scale
    cost = np.sum(
        (first_features[:, None, :] - second_features[None, :, :]) ** 2,
        axis=2,
    )
    first_indices, second_indices = linear_sum_assignment(cost)
    first_error = np.asarray([float(first[index]["launch_error_deg"]) for index in first_indices])
    second_error = np.asarray(
        [float(second[index]["launch_error_deg"]) for index in second_indices]
    )
    return {
        "n_pairs": len(first_indices),
        "first_minus_second_mae_deg": float(
            np.mean(np.abs(first_error)) - np.mean(np.abs(second_error))
        ),
        "first_minus_second_bias_deg": float(np.mean(first_error - second_error)),
        "mean_standardized_match_distance": float(
            np.mean(np.sqrt(cost[first_indices, second_indices]))
        ),
    }


def _comparison(
    first: list[dict[str, str]],
    second: list[dict[str, str]],
) -> dict[str, object]:
    first_errors = np.asarray([float(row["launch_error_deg"]) for row in first])
    second_errors = np.asarray([float(row["launch_error_deg"]) for row in second])
    low, high = _bootstrap_difference(first_errors, second_errors)
    return {
        "first_minus_second_mae_deg": float(
            np.mean(np.abs(first_errors)) - np.mean(np.abs(second_errors))
        ),
        "first_minus_second_bias_deg": float(np.mean(first_errors) - np.mean(second_errors)),
        "mae_difference_bootstrap_95_deg": [low, high],
        "mae_difference_exact_two_sided_p": _exact_permutation(
            first_errors,
            second_errors,
        ),
        "matched_on_tm_launch_and_speed": _matched_difference(first, second),
    }


def _geometry_correlation(rows: list[dict[str, str]]) -> dict[str, float]:
    delay = np.asarray([float(row["median_double_excess_cm"]) for row in rows])
    separation = np.asarray([float(row["median_angle_separation_deg"]) for row in rows])
    absolute_error = np.abs([float(row["launch_error_deg"]) for row in rows])
    delay_result = spearmanr(delay, absolute_error)
    separation_result = spearmanr(separation, absolute_error)
    return {
        "gg_delay_vs_absolute_error_rho": float(delay_result.statistic),
        "gg_delay_vs_absolute_error_p": float(delay_result.pvalue),
        "angle_separation_vs_absolute_error_rho": float(separation_result.statistic),
        "angle_separation_vs_absolute_error_p": float(separation_result.pvalue),
    }


def _range_summary(rows: list[dict[str, str]], block: str) -> dict[str, float | int]:
    selected = [row for row in rows if row["block"] == block and row["club"].startswith("9Iron")]
    return {
        "n": len(selected),
        "cross_asym_db": float(
            np.mean([float(row["median_all_cross_asym_db"]) for row in selected])
        ),
        "gg_asym_db": float(np.mean([float(row["median_all_double_asym_db"]) for row in selected])),
        "shoulder_asym_db": float(
            np.mean([float(row["median_all_shoulder_asym_db"]) for row in selected])
        ),
        "peak_shift_cm": float(
            np.mean([float(row["median_all_peak_shift_cm"]) for row in selected])
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--channel", type=Path, default=DEFAULT_CHANNEL)
    parser.add_argument("--range", dest="range_path", type=Path, default=DEFAULT_RANGE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    channel_rows = _read_csv(args.channel)
    range_rows = _read_csv(args.range_path)
    result: dict[str, object] = {
        "scope": "9-iron only; radar-velocity TDM correction; no fitted offsets",
        "confounding": "height, tilt, tee distance, and shot populations changed together",
        "models": {},
        "range_asymmetry": {block: _range_summary(range_rows, block) for block in "ABC"},
    }
    models: dict[str, object] = {}
    for model in MODELS:
        by_block = {block: _selected(channel_rows, model, block) for block in "ABC"}
        all_rows = [row for block_rows in by_block.values() for row in block_rows]
        models[model] = {
            "blocks": {block: _summary(rows) for block, rows in by_block.items()},
            "comparisons": {
                "A_minus_B": _comparison(by_block["A"], by_block["B"]),
                "A_minus_C": _comparison(by_block["A"], by_block["C"]),
                "C_minus_B": _comparison(by_block["C"], by_block["B"]),
            },
            "geometry_correlations": _geometry_correlation(all_rows),
        }
    result["models"] = models
    args.output.write_text(json.dumps(result, indent=2) + "\n")

    print("9-iron geometry audit; positive A-B means Block B has lower MAE")
    print("model             A MAE  B MAE  C MAE   A-B   p(A/B)  A-C")
    for model in MODELS:
        model_result = models[model]
        blocks = model_result["blocks"]
        comparisons = model_result["comparisons"]
        print(
            f"{model:16s} "
            f"{blocks['A']['mae_deg']:6.2f} {blocks['B']['mae_deg']:6.2f} "
            f"{blocks['C']['mae_deg']:6.2f} "
            f"{comparisons['A_minus_B']['first_minus_second_mae_deg']:+6.2f} "
            f"{comparisons['A_minus_B']['mae_difference_exact_two_sided_p']:8.3f} "
            f"{comparisons['A_minus_C']['first_minus_second_mae_deg']:+6.2f}"
        )
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
