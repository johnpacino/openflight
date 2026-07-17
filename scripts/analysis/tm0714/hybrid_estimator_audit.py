#!/usr/bin/env python3
"""Score frozen equal-weight research ensembles without per-club offsets."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
DEFAULT_CHANNEL = HERE / "multipath_channel_audit_production.csv"
DEFAULT_FAST = HERE / "fast_time_multipath_second_half.csv"
DEFAULT_TABLE = HERE / "cache_tm0714.json"
DEFAULT_OUTPUT = HERE / "hybrid_estimator_audit.csv"
DEFAULT_SUMMARY = HERE / "hybrid_estimator_audit.json"
DEFAULT_BLOCK_A_OUTPUT = HERE / "lcmf_v1_block_a_shots.csv"
MPH = 2.23694
VARIANTS = {
    "channel_two": (("channel", "two8"), ("channel", "four4_path_tdm")),
    "fast_late_three": (
        ("fast", "direct1"),
        ("fast", "two2"),
        ("fast", "four4"),
    ),
    "hybrid_three": (
        ("channel", "two8"),
        ("channel", "four4_path_tdm"),
        ("fast", "four4"),
    ),
    "lcmf_v1": (
        ("channel", "two8"),
        ("channel", "four4_path_tdm"),
        ("fast", "direct1"),
        ("fast", "two2"),
        ("fast", "four4"),
    ),
}


def _load(path: Path) -> dict[tuple[int, str], dict[str, str]]:
    with path.open(newline="") as handle:
        return {(int(row["shot"]), row["model"]): row for row in csv.DictReader(handle)}


def _crossfit(errors: np.ndarray) -> tuple[np.ndarray, list[float]]:
    corrected: list[float] = []
    biases: list[float] = []
    for parity in (0, 1):
        bias = float(np.mean(errors[parity::2]))
        biases.append(bias)
        corrected.extend(errors[1 - parity :: 2] - bias)
    return np.asarray(corrected), biases


def _bootstrap_mae_interval(errors: np.ndarray) -> list[float]:
    rng = np.random.default_rng(20260716)
    samples = rng.choice(errors, size=(50_000, len(errors)), replace=True)
    return [float(value) for value in np.quantile(np.mean(np.abs(samples), axis=1), [0.025, 0.975])]


def _random_split_crossfit(errors: np.ndarray) -> list[float]:
    rng = np.random.default_rng(7142026)
    values: list[float] = []
    for _ in range(5_000):
        order = rng.permutation(len(errors))
        midpoint = len(errors) // 2
        first, second = errors[order[:midpoint]], errors[order[midpoint:]]
        corrected = np.concatenate([second - np.mean(first), first - np.mean(second)])
        values.append(float(np.mean(np.abs(corrected))))
    return [float(value) for value in np.quantile(values, [0.05, 0.5, 0.95])]


def _club_transfer(rows: list[dict[str, object]]) -> tuple[float, dict[str, float]]:
    errors = np.asarray([row["raw_error_deg"] for row in rows], dtype=float)
    clubs = np.asarray([str(row["club"]).split()[0] for row in rows])
    corrected: list[float] = []
    biases: dict[str, float] = {}
    for club in np.unique(clubs):
        held_out = clubs == club
        bias = float(np.mean(errors[~held_out]))
        biases[str(club)] = bias
        corrected.extend(errors[held_out] - bias)
    return float(np.mean(np.abs(corrected))), biases


def _metrics(errors: np.ndarray) -> dict[str, float | int]:
    return {
        "n": len(errors),
        "mae_deg": float(np.mean(np.abs(errors))),
        "bias_deg": float(np.mean(errors)),
        "sd_deg": float(np.std(errors, ddof=1)),
        "median_ae_deg": float(np.median(np.abs(errors))),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--channel", type=Path, default=DEFAULT_CHANNEL)
    parser.add_argument("--fast", type=Path, default=DEFAULT_FAST)
    parser.add_argument("--table", type=Path, default=DEFAULT_TABLE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--block-a-output", type=Path, default=DEFAULT_BLOCK_A_OUTPUT)
    args = parser.parse_args()

    sources = {"channel": _load(args.channel), "fast": _load(args.fast)}
    records = {row["num"]: row for row in json.loads(args.table.read_text())}
    all_shots = sorted({shot for source in sources.values() for shot, _model in source})
    rows: list[dict[str, object]] = []
    summary: dict[str, object] = {
        "calibration_policy": "one Block-A-wide constant; no club-specific offsets",
        "selection_warning": (
            "exploratory model selected on this session; freeze before independent validation"
        ),
        "variants": {},
    }
    print("Equal-weight production-input ensembles")
    print("variant          A raw  A bias  A global-CV  club-transfer  B(A-cal)  C(A-cal)")
    for variant, components in VARIANTS.items():
        variant_rows: list[dict[str, object]] = []
        for shot in all_shots:
            if not all((shot, model) in sources[source] for source, model in components):
                continue
            component_rows = [sources[source][(shot, model)] for source, model in components]
            estimate = float(np.mean([float(row["launch_angle_deg"]) for row in component_rows]))
            truth = float(component_rows[0]["trackman_launch_deg"])
            variant_rows.append(
                {
                    "variant": variant,
                    "shot": shot,
                    "club": component_rows[0]["club"],
                    "block": component_rows[0]["block"],
                    "n_components": len(components),
                    "components": "+".join(f"{source}:{model}" for source, model in components),
                    "launch_angle_deg": estimate,
                    "trackman_launch_deg": truth,
                    "trackman_ball_mph": float(records[shot]["tm_ball_ms"]) * MPH,
                    "ops_ball_mph": records[shot]["of_ball_mph"],
                    "raw_error_deg": estimate - truth,
                }
            )

        block_a = sorted(
            [row for row in variant_rows if row["block"] == "A"],
            key=lambda row: int(row["shot"]),
        )
        a_errors = np.asarray([row["raw_error_deg"] for row in block_a], dtype=float)
        block_a_bias = float(np.mean(a_errors))
        global_cv_errors, fold_biases = _crossfit(a_errors)
        global_cv_mae = float(np.mean(np.abs(global_cv_errors)))
        club_transfer_mae, club_biases = _club_transfer(block_a)
        by_block: dict[str, object] = {}
        for block in "ABC":
            selected = [row for row in variant_rows if row["block"] == block]
            raw_errors = np.asarray([row["raw_error_deg"] for row in selected], dtype=float)
            fixed_errors = raw_errors - block_a_bias
            by_block[block] = {
                "raw": _metrics(raw_errors),
                "fixed_block_a_offset": _metrics(fixed_errors),
            }
            for row in selected:
                row["block_a_offset_deg"] = block_a_bias
                row["fixed_error_deg"] = float(row["raw_error_deg"]) - block_a_bias
                row["corrected_launch_angle_deg"] = float(row["launch_angle_deg"]) - block_a_bias
        summary["variants"][variant] = {
            "components": [f"{source}:{model}" for source, model in components],
            "blocks": by_block,
            "block_a_global_crossfit_mae_deg": global_cv_mae,
            "block_a_global_crossfit_bootstrap_95_deg": _bootstrap_mae_interval(global_cv_errors),
            "block_a_random_split_crossfit_5_50_95_deg": _random_split_crossfit(a_errors),
            "block_a_global_crossfit_fold_biases_deg": fold_biases,
            "block_a_leave_one_club_out_mae_deg": club_transfer_mae,
            "block_a_leave_one_club_out_biases_deg": club_biases,
        }
        rows.extend(variant_rows)
        print(
            f"{variant:16s} "
            f"{by_block['A']['raw']['mae_deg']:5.2f} "
            f"{by_block['A']['raw']['bias_deg']:+7.2f} "
            f"{global_cv_mae:11.2f} {club_transfer_mae:14.2f} "
            f"{by_block['B']['fixed_block_a_offset']['mae_deg']:8.2f} "
            f"{by_block['C']['fixed_block_a_offset']['mae_deg']:8.2f}"
        )

    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    args.summary.write_text(json.dumps(summary, indent=2) + "\n")
    accepted = {
        int(row["shot"]): row for row in rows if row["variant"] == "lcmf_v1" and row["block"] == "A"
    }
    block_a_rows: list[dict[str, object]] = []
    for shot, record in sorted(records.items()):
        if record["block"] != "A":
            continue
        result = accepted.get(shot)
        block_a_rows.append(
            {
                "shot": shot,
                "club": record["club"],
                "trackman_launch_deg": record["tm_la"],
                "trackman_ball_mph": float(record["tm_ball_ms"]) * MPH,
                "ops_ball_mph": record["of_ball_mph"],
                "lcmf_v1_angle_deg": (
                    result["corrected_launch_angle_deg"] if result is not None else ""
                ),
                "status": (
                    "accepted"
                    if result is not None
                    else "no_measurement_insufficient_strict_coverage"
                ),
            }
        )
    with args.block_a_output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(block_a_rows[0]))
        writer.writeheader()
        writer.writerows(block_a_rows)
    print(f"wrote {args.output}, {args.summary}, and {args.block_a_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
