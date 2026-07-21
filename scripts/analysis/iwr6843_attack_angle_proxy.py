#!/usr/bin/env python3
"""Experimental attack-angle proxy from early IWR6843 club vertical motion."""

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


def _status(v0: float | None, v1: float | None, v2: float | None) -> tuple[str, float, list[str]]:
    reasons: list[str] = []
    if None in (v0, v1, v2):
        return "reject", 0.0, ["missing_vertical_frame"]
    assert v0 is not None and v1 is not None and v2 is not None
    dv12 = v2 - v1
    dv02 = v2 - v0
    if abs(dv12) > 22.0:
        reasons.append("frame1_to_2_vertical_jump")
    if not -25.0 <= dv12 <= 4.0:
        reasons.append("implausible_attack_sign_or_size")
    if dv12 >= 0:
        reasons.append("not_downward_1_to_2")
    if abs(v1) > 35.0 or abs(v2) > 35.0:
        reasons.append("vertical_bearing_outlier")
    if not reasons:
        confidence = 0.78
        if dv12 < 0 and dv02 < 0:
            confidence = 0.84
        return "accepted", confidence, reasons
    return "low_quality", 0.35, reasons


def summarize(rows: list[dict[str, str]], *, scale: float) -> list[dict[str, Any]]:
    by_shot: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        shot = int(row["shot"])
        by_shot.setdefault(shot, []).append(
            {
                "time_ms": _float(row.get("time_ms")),
                "v_deg": _float(row.get("v_deg")),
                "h_deg": _float(row.get("h_deg")),
                "range_m": _float(row.get("range_m")),
                "snr_med": _float(row.get("snr_med")),
                "ops_club_mph": _float(row.get("ops_club_mph")),
                "ops_ball_mph": _float(row.get("ops_ball_mph")),
            }
        )
    output: list[dict[str, Any]] = []
    for shot, points in sorted(by_shot.items()):
        points = sorted(points, key=lambda p: p["time_ms"] if p["time_ms"] is not None else 999.0)
        if len(points) < 3:
            output.append({"shot": shot, "status": "reject", "confidence": 0.0, "reasons": "missing_frames"})
            continue
        v0, v1, v2 = points[0]["v_deg"], points[1]["v_deg"], points[2]["v_deg"]
        status, confidence, reasons = _status(v0, v1, v2)
        dv12 = (v2 - v1) if v1 is not None and v2 is not None else None
        dv02 = (v2 - v0) if v0 is not None and v2 is not None else None
        # The vertical bearing swing is not one-to-one attack angle. Scale it
        # down to a conservative first proxy; TrackMan validation must set this.
        attack = None if dv12 is None else max(-16.0, min(8.0, dv12 * scale))
        output.append(
            {
                "shot": shot,
                "status": status,
                "confidence": confidence,
                "attack_proxy_deg": attack,
                "delta_v_1_to_2_deg": dv12,
                "delta_v_0_to_2_deg": dv02,
                "v0_deg": v0,
                "v1_deg": v1,
                "v2_deg": v2,
                "h0_deg": points[0]["h_deg"],
                "h1_deg": points[1]["h_deg"],
                "h2_deg": points[2]["h_deg"],
                "r0_m": points[0]["range_m"],
                "r1_m": points[1]["range_m"],
                "r2_m": points[2]["range_m"],
                "snr_median": statistics.median(
                    [p["snr_med"] for p in points[:3] if p["snr_med"] is not None]
                ),
                "ops_club_mph": points[0]["ops_club_mph"],
                "ops_ball_mph": points[0]["ops_ball_mph"],
                "reasons": ";".join(reasons),
            }
        )
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument(
        "--scale",
        type=float,
        default=0.42,
        help="Experimental scale from frame-1-to-2 vertical bearing change to attack angle.",
    )
    args = parser.parse_args()

    rows = list(csv.DictReader(args.motion_csv.expanduser().open(encoding="utf-8")))
    output = summarize(rows, scale=args.scale)
    fields = [
        "shot",
        "status",
        "confidence",
        "attack_proxy_deg",
        "delta_v_1_to_2_deg",
        "delta_v_0_to_2_deg",
        "v0_deg",
        "v1_deg",
        "v2_deg",
        "h0_deg",
        "h1_deg",
        "h2_deg",
        "r0_m",
        "r1_m",
        "r2_m",
        "snr_median",
        "ops_club_mph",
        "ops_ball_mph",
        "reasons",
    ]
    target = args.out.open("w", newline="", encoding="utf-8") if args.out else sys.stdout
    try:
        writer = csv.DictWriter(target, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in output:
            writer.writerow({field: _fmt(row.get(field)) for field in fields})
    finally:
        if args.out:
            target.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
