#!/usr/bin/env python3
"""Verify OPS speed selects the same TI range walk as TrackMan guidance."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import tm0714

HERE = Path(__file__).parent
TABLE_PATH = HERE / "cache_tm0714.json"
CACHE_PATH = HERE / "cache_tm0714.npz"
DEFAULT_OUTPUT = HERE / "ops_guide_audit.csv"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    table = {row["num"]: row for row in json.loads(TABLE_PATH.read_text())}
    shots = {shot.num: shot for shot in tm0714.load_shots()}
    cache = np.load(CACHE_PATH)
    strict_shots = {
        number for number in table if len(tm0714.balanced_snapshot_indices(cache, number)) >= 12
    }
    rows: list[dict[str, object]] = []
    for number, record in sorted(table.items()):
        if (
            record.get("guided_ms") is None
            or record.get("of_ball_mph") is None
            or number not in shots
        ):
            continue
        mti, geometry = tm0714.load_mti(shots[number].file)
        ops_ball_ms = float(record["of_ball_mph"]) / tm0714.MPH
        track = tm0714.find_guided(mti, geometry, ops_ball_ms)
        if track is None:
            rows.append(
                {
                    "shot": number,
                    "club": record["club"],
                    "block": record["block"],
                    "strict_angle_pool": number in strict_shots,
                    "ops_track_found": False,
                    "same_track": False,
                    "ops_ball_ms": ops_ball_ms,
                    "trackman_ball_ms": record["tm_ball_ms"],
                    "tm_guided_radial_ms": record["guided_ms"],
                }
            )
            continue
        speed_delta = track.speed_ms - float(record["guided_ms"])
        first_delta_ms = 1000.0 * (track.t_first - float(record["t_first"]))
        last_delta_ms = 1000.0 * (track.t_last - float(record["t_last"]))
        same_track = bool(
            abs(speed_delta) < 1.0 and abs(first_delta_ms) < 3.0 and abs(last_delta_ms) < 3.0
        )
        rows.append(
            {
                "shot": number,
                "club": record["club"],
                "block": record["block"],
                "strict_angle_pool": number in strict_shots,
                "ops_track_found": True,
                "same_track": same_track,
                "ops_ball_ms": ops_ball_ms,
                "trackman_ball_ms": record["tm_ball_ms"],
                "tm_guided_radial_ms": record["guided_ms"],
                "ops_guided_radial_ms": track.speed_ms,
                "radial_speed_delta_ms": speed_delta,
                "first_time_delta_ms": first_delta_ms,
                "last_time_delta_ms": last_delta_ms,
                "ops_track_rms_bins": track.rms_bins,
                "ops_track_inliers": track.n_inliers,
            }
        )
        print(
            f"shot {number:03d}: OPS track {track.speed_ms:5.1f} m/s, "
            f"delta {speed_delta:+5.1f}, same={same_track}"
        )

    fields = list(dict.fromkeys(key for row in rows for key in row))
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print("\nOPS-guided track equivalence")
    for label, selected in (
        ("all", rows),
        ("strict angle pool", [row for row in rows if row["strict_angle_pool"]]),
    ):
        found = sum(bool(row["ops_track_found"]) for row in selected)
        same = sum(bool(row["same_track"]) for row in selected)
        print(f"{label:17s}: found {found}/{len(selected)}, same {same}/{len(selected)}")
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
