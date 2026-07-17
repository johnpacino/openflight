#!/usr/bin/env python3
"""Score angle estimators without per-club calibration leakage."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
DEFAULT_ESTIMATES = HERE / "est_v2_results.json"
DEFAULT_TABLE = HERE / "cache_tm0714.json"
DEFAULT_OUTPUT = HERE / "honest_offset_scorecard.csv"
ESTIMATORS = (
    "music_tee",
    "music_tee_th",
    "tworay_w",
    "tworay_dom_xlim",
    "tworay_tee",
    "tworay_tee_th",
    "golden_k2",
)


def _track_ok(record: dict) -> bool:
    return bool(
        record.get("guided_ms") is not None
        and record["guided_rms"] < 0.50
        and record["guided_n"] >= 30
        and record["n_snaps"] >= 56
    )


def _crossfit_global(errors: np.ndarray) -> tuple[np.ndarray, tuple[float, float]]:
    corrected: list[float] = []
    biases: list[float] = []
    for parity in (0, 1):
        calibration = errors[parity::2]
        score = errors[1 - parity :: 2]
        bias = float(np.mean(calibration))
        corrected.extend(score - bias)
        biases.append(bias)
    return np.asarray(corrected), (biases[0], biases[1])


def _leave_one_club_out(
    errors: np.ndarray,
    clubs: np.ndarray,
) -> tuple[np.ndarray, dict[str, float]]:
    corrected: list[float] = []
    biases: dict[str, float] = {}
    for club in np.unique(clubs):
        score = clubs == club
        calibration = ~score
        bias = float(np.mean(errors[calibration]))
        biases[str(club)] = bias
        corrected.extend(errors[score] - bias)
    return np.asarray(corrected), biases


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--estimates", type=Path, default=DEFAULT_ESTIMATES)
    parser.add_argument("--table", type=Path, default=DEFAULT_TABLE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    estimates = {
        int(number): values for number, values in json.loads(args.estimates.read_text()).items()
    }
    table = {record["num"]: record for record in json.loads(args.table.read_text())}
    eligible = [
        number
        for number, record in sorted(table.items())
        if record["block"] == "A" and _track_ok(record)
    ]
    rows: list[dict[str, object]] = []
    print("Block A scorecard; no per-club offsets")
    print("estimator          n  cover  raw MAE   bias  global-CV  club-transfer")
    for estimator in ESTIMATORS:
        numbers = [
            number for number in eligible if estimates.get(number, {}).get(estimator) is not None
        ]
        errors = np.asarray(
            [estimates[number][estimator] - table[number]["tm_la"] for number in numbers]
        )
        clubs = np.asarray([table[number]["club"].split()[0] for number in numbers])
        global_errors, fold_biases = _crossfit_global(errors)
        transfer_errors, transfer_biases = _leave_one_club_out(errors, clubs)
        by_club: dict[str, list[float]] = defaultdict(list)
        for club, error in zip(clubs, errors, strict=True):
            by_club[str(club)].append(float(error))
        row: dict[str, object] = {
            "estimator": estimator,
            "n": len(errors),
            "coverage_pct": 100.0 * len(errors) / len(eligible),
            "raw_mae_deg": float(np.mean(np.abs(errors))),
            "raw_bias_deg": float(np.mean(errors)),
            "raw_sd_deg": float(np.std(errors, ddof=1)),
            "global_offset_crossfit_mae_deg": float(np.mean(np.abs(global_errors))),
            "global_offset_fold_0_deg": fold_biases[0],
            "global_offset_fold_1_deg": fold_biases[1],
            "leave_one_club_out_mae_deg": float(np.mean(np.abs(transfer_errors))),
            "leave_one_club_out_biases_json": json.dumps(transfer_biases, sort_keys=True),
            "raw_club_metrics_json": json.dumps(
                {
                    club: {
                        "n": len(values),
                        "mae_deg": float(np.mean(np.abs(values))),
                        "bias_deg": float(np.mean(values)),
                    }
                    for club, values in sorted(by_club.items())
                },
                sort_keys=True,
            ),
        }
        rows.append(row)
        print(
            f"{estimator:18s} {len(errors):2d} {row['coverage_pct']:5.1f}% "
            f"{row['raw_mae_deg']:8.2f} {row['raw_bias_deg']:+6.2f} "
            f"{row['global_offset_crossfit_mae_deg']:10.2f} "
            f"{row['leave_one_club_out_mae_deg']:13.2f}"
        )

    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
