#!/usr/bin/env python3
"""Build one LCMF-angle OPS cosine-speed iteration for offline scoring."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from openflight.speed_correction import correct_ball_speed

HERE = Path(__file__).parent
DEFAULT_TABLE = HERE / "cache_tm0714.json"
DEFAULT_HYBRID = HERE / "hybrid_estimator_audit.csv"
DEFAULT_OUTPUT = HERE / "cache_tm0714_lcmf_speed.json"
DEFAULT_AUDIT = HERE / "lcmf_speed_iteration.csv"
MPH = 2.23694
M_TO_FT = 3.280839895


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--table", type=Path, default=DEFAULT_TABLE)
    parser.add_argument("--hybrid", type=Path, default=DEFAULT_HYBRID)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--audit", type=Path, default=DEFAULT_AUDIT)
    args = parser.parse_args()

    table = json.loads(args.table.read_text())
    records = {row["num"]: row for row in table}
    with args.hybrid.open(newline="") as handle:
        estimates = {
            int(row["shot"]): row for row in csv.DictReader(handle) if row["variant"] == "lcmf_v1"
        }

    audit_rows: list[dict[str, object]] = []
    for shot, estimate in estimates.items():
        record = records[shot]
        raw_speed = float(record["of_ball_mph"])
        launch_angle = float(estimate["corrected_launch_angle_deg"])
        corrected_speed = correct_ball_speed(
            raw_speed,
            launch_angle,
            float(record["x_tee"]) * M_TO_FT,
            (float(record["z0"]) - float(record["rh"])) * M_TO_FT,
        )
        record["of_ball_mph"] = corrected_speed
        audit_rows.append(
            {
                "shot": shot,
                "club": record["club"],
                "block": record["block"],
                "lcmf_v1_angle_deg": launch_angle,
                "trackman_ball_mph": float(record["tm_ball_ms"]) * MPH,
                "ops_raw_mph": raw_speed,
                "ops_corrected_mph": corrected_speed,
                "raw_error_mph": raw_speed - float(record["tm_ball_ms"]) * MPH,
                "corrected_error_mph": corrected_speed - float(record["tm_ball_ms"]) * MPH,
            }
        )

    args.output.write_text(json.dumps(table, indent=1) + "\n")
    with args.audit.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(audit_rows[0]))
        writer.writeheader()
        writer.writerows(audit_rows)

    print("LCMF-v1 cosine speed correction")
    for block in "ABC":
        selected = [row for row in audit_rows if row["block"] == block]
        if not selected:
            continue
        raw = np.asarray([row["raw_error_mph"] for row in selected], dtype=float)
        corrected = np.asarray([row["corrected_error_mph"] for row in selected], dtype=float)
        print(
            f"block {block}: n={len(selected)} "
            f"raw MAE/bias {np.mean(np.abs(raw)):.2f}/{np.mean(raw):+.2f} mph, "
            f"corrected {np.mean(np.abs(corrected)):.2f}/{np.mean(corrected):+.2f} mph"
        )
    print(f"wrote {args.output} and {args.audit}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
