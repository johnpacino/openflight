#!/usr/bin/env python3
"""Replay radar-only and camera-primary horizontal estimators against truth."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from openflight.camera.ball_flight import (
    CameraBallGeometry,
    estimate_camera_ball_flight,
    select_camera_assisted_horizontal,
)
from openflight.camera.club_delivery import ReferenceBallTracker
from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.lcmf import estimate_lcmf_v1


def _event_data(session_path: str, shot_number: int) -> tuple[dict, dict]:
    start = None
    capture = None
    with Path(session_path).open(encoding="utf-8") as handle:
        for line in handle:
            event = json.loads(line)
            if event.get("type") == "session_start":
                start = event
            elif (
                event.get("type") == "iwr6843_capture"
                and int(event.get("shot_number", -1)) == shot_number
            ):
                capture = event
    if start is None or capture is None:
        raise ValueError(f"missing session metadata for shot {shot_number}")
    return start, capture


def _replay(
    row: dict[str, str],
    *,
    horizontal_offset_deg: float,
    horizontal_phase_reference_rad: float | None,
    ball_tracker: ReferenceBallTracker | None,
) -> dict[str, object]:
    shot_number = int(row["of_shot_number"])
    start, capture = _event_data(row["session_log_path"], shot_number)
    radar_config = start["config"]["iwr6843"]
    camera_config = start["config"]["camera_capture"]

    calibration = Calibration.load("config/iwr6843_calibration_reference.json")
    calibration.tee_range_m = float(radar_config["tee_slant_range_m"])
    calibration.tee_ball_height_m = float(radar_config["ball_height_m"])
    calibration.meta["radar_height_m"] = float(radar_config["radar_height_m"])
    calibration.tilt_rad = math.radians(float(row["of_effective_tilt_deg"]))
    ball_speed_mph = float(capture["ball_speed_mph"])
    radar = estimate_lcmf_v1(
        Path(row["iwr_dump_path"]).read_bytes(),
        calibration,
        ball_speed_mph=ball_speed_mph,
        club=row["of_club"],
        net_range_m=float(radar_config["net_range_m"]),
        tx_order=radar_config.get("tx_order", "normal"),
        horizontal_phase_reference_rad=(
            horizontal_phase_reference_rad
            if horizontal_phase_reference_rad is not None
            else radar_config.get("horizontal_phase_reference_rad")
        ),
    )

    archive = np.load(Path(row["camera_directory"]) / "frames.npz")
    frames = archive["frames"]
    camera_geometry = CameraBallGeometry(
        camera_height_m=float(camera_config["mount_height_m"]),
        radar_height_m=float(radar_config["radar_height_m"]),
        tee_range_m=float(radar_config["tee_slant_range_m"]),
        ball_height_m=float(radar_config["ball_height_m"]),
        camera_lateral_offset_m=float(camera_config.get("lateral_offset_m", 0.0)),
        horizontal_offset_deg=horizontal_offset_deg,
        horizontal_pixel_sign=-1.0 if camera_config.get("mirror_horizontal") else 1.0,
        image_width_px=frames.shape[2],
        image_height_px=frames.shape[1],
    )
    camera = estimate_camera_ball_flight(
        frames,
        archive["host_timestamp_ns"],
        trigger_ns=int(archive["trigger_host_timestamp_ns"]),
        range_evidence=radar.range_evidence,
        geometry=camera_geometry,
        ops_ball_speed_mph=ball_speed_mph,
        iwr_vertical_deg=radar.angle_deg,
        ball_tracker=ball_tracker,
    )
    selected = select_camera_assisted_horizontal(
        camera,
        iwr_horizontal_deg=radar.horizontal_deg,
        iwr_confidence=radar.horizontal_confidence,
    )
    return {
        "tm_sequence": int(row["tm_sequence"]),
        "club": row["tm_club"],
        "capture_profile": "dense_iq8" if "dense" in row["iwr_config"] else "wide_iq16",
        "truth_deg": float(row["tm_launch_direction_deg"]),
        "radar_deg": radar.horizontal_deg,
        "radar_confidence": radar.horizontal_confidence,
        "radar_status": radar.horizontal_status,
        "camera_deg": camera.horizontal_deg,
        "camera_tier": camera.confidence_tier,
        "camera_status": camera.status,
        "camera_support": camera.support,
        "camera_parameter_mad_deg": camera.parameter_mad_deg,
        "camera_window_mad_deg": camera.window_mad_deg,
        "camera_speed_error_mph": camera.speed_error_mph,
        "selected_deg": selected.selected_deg,
        "selected_source": selected.source,
        "selected_status": selected.status,
    }


def _delta(estimate: float, truth: float) -> float:
    return (estimate - truth + 180.0) % 360.0 - 180.0


def _metrics(rows: list[dict[str, object]], field: str) -> str:
    available = [row for row in rows if row[field] is not None]
    errors = np.asarray([_delta(float(row[field]), float(row["truth_deg"])) for row in available])
    if not errors.size:
        return f"coverage=0/{len(rows)}"
    absolute = np.abs(errors)
    return (
        f"coverage={len(available)}/{len(rows)} ({100 * len(available) / len(rows):.1f}%) "
        f"MAE={absolute.mean():.3f} bias={errors.mean():+.3f} "
        f"p50={np.percentile(absolute, 50):.3f} "
        f"p75={np.percentile(absolute, 75):.3f} "
        f"p90={np.percentile(absolute, 90):.3f}"
    )


def main() -> int:
    """Replay every matched shot and write per-shot diagnostics."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("aligned_csv", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--camera-horizontal-offset-deg",
        type=float,
        default=0.0,
        help="Measured setup correction added to camera horizontal launch.",
    )
    parser.add_argument(
        "--horizontal-phase-reference-rad",
        type=float,
        help="Override the IWR horizontal phase reference recorded in the session.",
    )
    parser.add_argument(
        "--stable-anchor",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use the live rolling reference-ball anchor (default: enabled).",
    )
    args = parser.parse_args()

    with args.aligned_csv.open(newline="", encoding="utf-8") as handle:
        source_rows = [
            row
            for row in csv.DictReader(handle)
            if row["match_status"] == "matched" and row["iwr_dump_path"] and row["camera_directory"]
        ]
    trackers: dict[str, ReferenceBallTracker] = {}
    rows = []
    for row in source_rows:
        tracker = None
        if args.stable_anchor:
            tracker = trackers.setdefault(row["session_log_path"], ReferenceBallTracker())
        rows.append(
            _replay(
                row,
                horizontal_offset_deg=args.camera_horizontal_offset_deg,
                horizontal_phase_reference_rad=args.horizontal_phase_reference_rad,
                ball_tracker=tracker,
            )
        )

    for profile in ("wide_iq16", "dense_iq8", "all"):
        cohort = (
            rows if profile == "all" else [row for row in rows if row["capture_profile"] == profile]
        )
        print(f"\n{profile} ({len(cohort)} shots)")
        print("  radar-only:    ", _metrics(cohort, "radar_deg"))
        print("  camera-only:   ", _metrics(cohort, "camera_deg"))
        print("  camera/fallback", _metrics(cohort, "selected_deg"))

    output = args.output or args.aligned_csv.with_name("horizontal_estimator_replay.csv")
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nSaved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
