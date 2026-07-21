#!/usr/bin/env python3
"""Convert early-frame IWR6843 club motion into a signed club-path proxy.

This intentionally produces an experimental proxy, not a TrackMan-equivalent
club path. The stable signal in the 2026-07-19 3TX data is frame-to-frame TX2
horizontal motion near impact; this script compresses that evidence to one row
per shot with quality flags.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Any


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


def _quality(row: dict[str, float | int | None]) -> tuple[str, float, list[str]]:
    reasons: list[str] = []
    confidence = 0.25
    h0 = row["h0_deg"]
    h1 = row["h1_deg"]
    h2 = row["h2_deg"]
    dh02 = row["delta_h_0_to_2_deg"]
    if None in (h0, h1, h2, dh02):
        return "reject", 0.0, ["missing_frames"]
    assert h0 is not None and h1 is not None and h2 is not None and dh02 is not None
    if not -18.0 <= h0 <= 8.0:
        reasons.append("frame0_bearing_outlier")
    if abs(dh02) > 15.0:
        reasons.append("delta_outlier")
    if h2 <= h0:
        reasons.append("not_rightward_0_to_2")
    if h2 <= min(h0, h1):
        reasons.append("frame2_not_rightmost")
    if abs(h1 - h0) > 12.0:
        reasons.append("frame0_to_1_jump")
    if abs(h2 - h1) > 12.0:
        reasons.append("frame1_to_2_jump")
    if not reasons:
        confidence = 0.75
        if h2 > h1 >= h0:
            confidence = 0.88
        elif h2 > h0 and dh02 > 3.0:
            confidence = 0.82
        return "accepted", confidence, reasons
    if reasons == ["not_rightward_0_to_2"]:
        return "opposite_sign", 0.45, reasons
    return "low_quality", 0.35, reasons


def summarize(rows: list[dict[str, str]], *, scale: float) -> list[dict[str, Any]]:
    by_shot: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        shot = int(row["shot"])
        by_shot.setdefault(shot, []).append(
            {
                "frame": int(row["frame"]),
                "time_ms": _float(row.get("time_ms")),
                "range_m": _float(row.get("range_m")),
                "h_deg": _float(row.get("h_deg")),
                "snr_med": _float(row.get("snr_med")),
                "ops_club_mph": _float(row.get("ops_club_mph")),
                "ops_ball_mph": _float(row.get("ops_ball_mph")),
                "live_h_deg": _float(row.get("live_h_deg")),
            }
        )

    out: list[dict[str, Any]] = []
    for shot, points in sorted(by_shot.items()):
        points = sorted(points, key=lambda point: point["time_ms"] if point["time_ms"] is not None else 999.0)
        if len(points) < 3:
            out.append({"shot": shot, "status": "reject", "confidence": 0.0, "reasons": "missing_frames"})
            continue
        h0, h1, h2 = points[0]["h_deg"], points[1]["h_deg"], points[2]["h_deg"]
        r0, r1, r2 = points[0]["range_m"], points[1]["range_m"], points[2]["range_m"]
        dh02 = (h2 - h0) if h0 is not None and h2 is not None else None
        dr02 = (r2 - r0) if r0 is not None and r2 is not None else None
        row = {
            "h0_deg": h0,
            "h1_deg": h1,
            "h2_deg": h2,
            "delta_h_0_to_2_deg": dh02,
        }
        status, confidence, reasons = _quality(row)
        proxy = None if dh02 is None else max(-12.0, min(12.0, dh02 * scale))
        out.append(
            {
                "shot": shot,
                "status": status,
                "confidence": confidence,
                "club_path_proxy_deg": proxy,
                "delta_h_0_to_2_deg": dh02,
                "delta_range_0_to_2_m": dr02,
                "h0_deg": h0,
                "h1_deg": h1,
                "h2_deg": h2,
                "r0_m": r0,
                "r1_m": r1,
                "r2_m": r2,
                "snr_median": statistics.median(
                    [p["snr_med"] for p in points[:3] if p["snr_med"] is not None]
                ),
                "ops_club_mph": points[0]["ops_club_mph"],
                "ops_ball_mph": points[0]["ops_ball_mph"],
                "live_ball_h_deg": points[0]["live_h_deg"],
                "reasons": ";".join(reasons),
            }
        )
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument(
        "--scale",
        type=float,
        default=0.55,
        help=(
            "Experimental mapping from early-frame horizontal bearing shift to "
            "club-path degrees. 0.55 makes the 2026-07-19 9i median land near "
            "the TrackMan 9i path baseline; revalidate before product use."
        ),
    )
    args = parser.parse_args()

    rows = list(csv.DictReader(args.motion_csv.expanduser().open(encoding="utf-8")))
    output = summarize(rows, scale=args.scale)
    fields = [
        "shot",
        "status",
        "confidence",
        "club_path_proxy_deg",
        "delta_h_0_to_2_deg",
        "delta_range_0_to_2_m",
        "h0_deg",
        "h1_deg",
        "h2_deg",
        "r0_m",
        "r1_m",
        "r2_m",
        "snr_median",
        "ops_club_mph",
        "ops_ball_mph",
        "live_ball_h_deg",
        "reasons",
    ]
    target = args.out.open("w", newline="", encoding="utf-8") if args.out else sys.stdout
    try:
        writer = csv.DictWriter(target, fields, extrasaction="ignore")
        writer.writeheader()
        for row in output:
            writer.writerow({field: _fmt(row.get(field)) for field in fields})
    finally:
        if args.out:
            target.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
