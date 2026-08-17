#!/usr/bin/env python3
"""Analyze OV9281 clap-buffer captures for pre-impact club motion."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))

from openflight.camera.club_motion import (  # noqa: E402
    detect_reference_ball,
    image_plane_motion,
    track_bright_shaft_endpoint,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="Camera clap-buffer session directory")
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output directory (default: <session>/club_motion_analysis)",
    )
    parser.add_argument("--ball-roi", type=int, nargs=4, metavar=("X0", "Y0", "X1", "Y1"))
    return parser.parse_args()


def annotated_strip(frames: np.ndarray, ball, track, output: Path) -> None:
    """Write a compact PNG showing the selected shaft endpoint in each frame."""
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("run with --extra analysis to generate overlays") from exc

    tiles = []
    points_by_frame = {point.frame_index: point for point in track.points}
    for frame_index in range(min(points_by_frame), max(points_by_frame) + 1):
        image = cv2.cvtColor(frames[frame_index], cv2.COLOR_GRAY2BGR)
        cv2.circle(image, (round(ball.x), round(ball.y)), 7, (0, 255, 0), 1)
        point = points_by_frame.get(frame_index)
        if point:
            cv2.circle(image, (round(point.x), round(point.y)), 6, (0, 0, 255), 2)
        cv2.putText(
            image,
            f"frame {frame_index}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        x0 = max(0, round(ball.x) - 170)
        x1 = min(image.shape[1], round(ball.x) + 100)
        y0 = max(0, round(ball.y) - 170)
        y1 = min(image.shape[0], round(ball.y) + 70)
        tiles.append(image[y0:y1, x0:x1])
    height = min(tile.shape[0] for tile in tiles)
    strip = np.concatenate([tile[:height] for tile in tiles], axis=1)
    cv2.imwrite(str(output), strip)


def main() -> int:
    args = parse_args()
    session = args.session.expanduser().resolve()
    output_dir = args.output_dir or session / "club_motion_analysis"
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []

    for capture_dir in sorted(session.glob("capture_*")):
        archive = np.load(capture_dir / "frames.npz")
        frames = archive["frames"]
        timestamps_ns = archive["host_timestamp_ns"]
        trigger_index = int(archive["pre_trigger_count"]) - 1
        ball = detect_reference_ball(frames, roi=tuple(args.ball_roi) if args.ball_roi else None)
        track = track_bright_shaft_endpoint(
            frames,
            trigger_frame_index=trigger_index,
            ball=ball,
        )
        result = {
            "capture": capture_dir.name,
            "ball_x": ball.x,
            "ball_y": ball.y,
            "ball_diameter_px": ball.diameter_px,
            "track_confidence": track.confidence,
            "track_reason": track.reason,
            "tracked_frames": [point.frame_index for point in track.points],
            "points": [asdict(point) for point in track.points],
        }
        if len(track.points) >= 2:
            motion = image_plane_motion(
                track.points,
                timestamps_ns,
                ball_diameter_px=ball.diameter_px,
            )
            result.update(asdict(motion))
            annotated_strip(frames, ball, track, output_dir / f"{capture_dir.name}.png")
        rows.append(result)

    (output_dir / "results.json").write_text(json.dumps(rows, indent=2) + "\n")
    scalar_keys = sorted(
        {key for row in rows for key, value in row.items() if not isinstance(value, (list, dict))}
    )
    with (output_dir / "results.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=scalar_keys)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in scalar_keys} for row in rows)

    accepted = sum(row["track_reason"] == "ok" for row in rows)
    print(f"Analyzed {len(rows)} captures; complete tracks: {accepted}/{len(rows)}")
    print(f"Results: {output_dir}")
    print("Motion values are image-plane measurements, not final club path or AoA.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
