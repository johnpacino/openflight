#!/usr/bin/env python3
"""Sweep camera/IWR ball-flight fusion without fitting to launch-monitor truth.

The rear camera and IWR6843 share the same lateral origin. Camera centroids
therefore supply ball bearing while the radar track supplies metric range. The
shared sound-trigger timestamp aligns both sensors at impact.
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

from openflight.camera.club_motion import (
    BALL_DIAMETER_MM,
    ReferenceBall,
    detect_reference_ball,
)
from openflight.iwr6843 import tracking
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.shot import (
    TX2_LOOP_PERIOD_S,
    geometry_from_header,
    is_range_snapshot,
    project_tx_pair,
)

MPH_PER_MS = 2.23694


@dataclass(frozen=True)
class Candidate:
    x: float
    y: float
    area: int
    width: int
    height: int
    fill: float
    circularity: float
    mean_intensity: float


@dataclass(frozen=True)
class Estimate:
    shot: int
    bright_threshold: int
    difference_threshold: int
    min_area: int
    horizontal_deg: float
    vertical_deg: float
    speed_mph: float
    speed_error_mph: float
    fit_median_m: float
    step_speed_mad_mph: float
    window_mad_deg: float
    n_points: int
    first_frame: int
    last_frame: int


def _events(session_file: Path) -> tuple[dict, dict, dict, dict]:
    start: dict = {}
    camera: dict[int, dict] = {}
    radar: dict[int, dict] = {}
    shots: dict[int, dict] = {}
    with session_file.open(encoding="utf-8") as handle:
        for line in handle:
            event = json.loads(line)
            event_type = event.get("type")
            if event_type == "session_start":
                start = event
            elif event_type == "camera_capture":
                camera[int(event["shot_number"])] = event
            elif event_type == "iwr6843_capture":
                radar[int(event["shot_number"])] = event
            elif event_type == "shot_detected":
                shots[int(event["shot_number"])] = event
    return start, camera, radar, shots


def _local_capture(root: Path, event: dict, kind: str) -> Path:
    name = Path(event["capture_path"]).name
    if kind == "camera":
        return root / "openflight_sessions" / "trackman" / "camera" / name
    return root / "openflight_sessions" / "iwr6843" / name


def _session_anchor(root: Path, camera_events: dict[int, dict]) -> ReferenceBall:
    candidates: list[ReferenceBall] = []
    for event in camera_events.values():
        frames = np.load(_local_capture(root, event, "camera") / "frames.npz")["frames"]
        try:
            ball = detect_reference_ball(frames)
        except ValueError:
            continue
        if 9.0 <= ball.diameter_px <= 30.0:
            candidates.append(ball)
    if len(candidates) < 3:
        raise RuntimeError("fewer than three plausible stationary-ball observations")
    return ReferenceBall(
        x=float(np.median([ball.x for ball in candidates])),
        y=float(np.median([ball.y for ball in candidates])),
        diameter_px=float(np.median([ball.diameter_px for ball in candidates])),
        area_px=int(round(np.median([ball.area_px for ball in candidates]))),
    )


def _camera_model(
    anchor: ReferenceBall,
    *,
    tee_range_m: float,
    ball_height_m: float,
    camera_height_m: float,
    radar_height_m: float,
    width: int,
    height: int,
) -> tuple[float, float, float, np.ndarray]:
    center_x = width / 2.0
    center_y = height / 2.0
    camera_ball_range = math.hypot(tee_range_m, ball_height_m - camera_height_m)
    focal_px = anchor.diameter_px * camera_ball_range / (BALL_DIAMETER_MM / 1000.0)
    pitch = math.atan2(ball_height_m - camera_height_m, tee_range_m) - math.atan2(
        -(anchor.y - center_y) / focal_px, 1.0
    )
    target_yaw = math.atan2((anchor.x - center_x) / focal_px, 1.0)
    radar_from_camera = np.array([0.0, 0.0, camera_height_m - radar_height_m])
    return focal_px, pitch, target_yaw, radar_from_camera


def _project(
    candidate: Candidate,
    radar_range_m: float,
    *,
    focal_px: float,
    pitch: float,
    radar_from_camera: np.ndarray,
    camera_height_m: float,
    width: int,
    height: int,
) -> np.ndarray | None:
    image_z = -(candidate.y - height / 2.0) / focal_px
    ray = np.array(
        [
            (candidate.x - width / 2.0) / focal_px,
            math.cos(pitch) - image_z * math.sin(pitch),
            math.sin(pitch) + image_z * math.cos(pitch),
        ]
    )
    ray /= np.linalg.norm(ray)
    ray_offset = float(ray @ radar_from_camera)
    discriminant = ray_offset**2 - (float(radar_from_camera @ radar_from_camera) - radar_range_m**2)
    if discriminant < 0.0:
        return None
    distance = -ray_offset + math.sqrt(discriminant)
    if distance <= 0.0:
        return None
    return np.array([0.0, 0.0, camera_height_m]) + distance * ray


def _candidates(
    frame: np.ndarray,
    background: np.ndarray,
    anchor: ReferenceBall,
    *,
    bright_threshold: int,
    difference_threshold: int,
    min_area: int,
) -> list[Candidate]:
    difference = cv2.subtract(frame, background)
    mask = ((frame > bright_threshold) & (difference > difference_threshold)).astype(np.uint8)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    found: list[Candidate] = []
    for label in range(1, count):
        _, _, width, height, area = stats[label]
        x, y = centroids[label]
        aspect = width / max(height, 1)
        fill = area / max(width * height, 1)
        if not (
            min_area <= area <= 400
            and 0.35 <= aspect <= 2.8
            and fill >= 0.18
            and abs(x - anchor.x) < 160
            and 10 < y < anchor.y + 15
        ):
            continue
        left, top = int(stats[label][cv2.CC_STAT_LEFT]), int(stats[label][cv2.CC_STAT_TOP])
        roi = (labels[top : top + height, left : left + width] == label).astype(np.uint8)
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        perimeter = sum(cv2.arcLength(contour, True) for contour in contours)
        circularity = 4.0 * math.pi * area / perimeter**2 if perimeter > 0.0 else 0.0
        pixel_values = frame[top : top + height, left : left + width][roi > 0]
        found.append(
            Candidate(
                float(x),
                float(y),
                int(area),
                int(width),
                int(height),
                float(fill),
                float(circularity),
                float(np.mean(pixel_values)),
            )
        )
    return found


def _rough_path_score(path: list[tuple[int, Candidate]]) -> float:
    if len(path) < 3:
        return 20.0 * len(path)
    steps = np.array(
        [
            ((second.x - first.x) / (j - i), (second.y - first.y) / (j - i))
            for (i, first), (j, second) in zip(path, path[1:])
        ]
    )
    dispersion = np.median(np.linalg.norm(steps - np.median(steps, axis=0), axis=1))
    return 20.0 * len(path) - 2.0 * float(dispersion)


def _pixel_paths(nodes: list[list[Candidate]], anchor: ReferenceBall) -> list[list]:
    all_paths: list[list[tuple[int, Candidate]]] = []
    frontier: list[list[tuple[int, Candidate]]] = []
    for frame in range(min(5, len(nodes))):
        for candidate in nodes[frame]:
            if math.hypot(candidate.x - anchor.x, candidate.y - anchor.y) <= 70.0:
                frontier.append([(frame, candidate)])
    all_paths.extend(frontier)
    for _ in range(len(nodes)):
        extended: list[list[tuple[int, Candidate]]] = []
        for path in frontier:
            previous_frame, previous = path[-1]
            for frame in range(previous_frame + 1, min(len(nodes), previous_frame + 3)):
                gap = frame - previous_frame
                for candidate in nodes[frame]:
                    delta_x = candidate.x - previous.x
                    delta_y = candidate.y - previous.y
                    if abs(delta_x) <= 30.0 * gap and -38.0 * gap <= delta_y <= -0.5 * gap:
                        extended.append([*path, (frame, candidate)])
        if not extended:
            break
        extended.sort(key=_rough_path_score, reverse=True)
        frontier = extended[:150]
        all_paths.extend(frontier)
    viable = [path for path in all_paths if len(path) >= 4]
    viable.sort(key=_rough_path_score, reverse=True)
    return viable[:120]


def _robust_velocity(times: np.ndarray, positions: np.ndarray) -> tuple[np.ndarray, float]:
    slopes = []
    for first in range(len(times)):
        for second in range(first + 1, len(times)):
            delta = times[second] - times[first]
            if delta > 0.0:
                slopes.append((positions[second] - positions[first]) / delta)
    velocity = np.median(slopes, axis=0)
    intercept = np.median(positions - times[:, None] * velocity, axis=0)
    residual = np.linalg.norm(positions - (intercept + times[:, None] * velocity), axis=1)
    return velocity, float(np.median(residual))


def _horizontal(velocity: np.ndarray, target_yaw: float) -> float:
    angle = math.atan2(float(velocity[0]), float(velocity[1])) - target_yaw
    return (math.degrees(angle) + 180.0) % 360.0 - 180.0


def _path_estimate(
    *,
    shot_number: int,
    path: list[tuple[int, Candidate]],
    frame_indices: list[int],
    timestamps_ns: np.ndarray,
    trigger_ns: int,
    ball_track,
    impact_t_s: float,
    range_resolution_m: float,
    ball_speed_mph: float,
    iwr_vertical_deg: float,
    model: tuple,
    camera_height_m: float,
    width: int,
    height: int,
    thresholds: tuple[int, int, int],
) -> tuple[float, Estimate] | None:
    focal_px, pitch, target_yaw, radar_from_camera = model
    times: list[float] = []
    positions: list[np.ndarray] = []
    actual_frames: list[int] = []
    used_candidates: list[Candidate] = []
    for relative_frame, candidate in path:
        frame = frame_indices[relative_frame]
        relative_time = (int(timestamps_ns[frame]) - trigger_ns) / 1e9
        radar_range = float(ball_track.range_at(impact_t_s + relative_time, range_resolution_m))
        position = _project(
            candidate,
            radar_range,
            focal_px=focal_px,
            pitch=pitch,
            radar_from_camera=radar_from_camera,
            camera_height_m=camera_height_m,
            width=width,
            height=height,
        )
        if position is not None:
            times.append(relative_time)
            positions.append(position)
            actual_frames.append(frame)
            used_candidates.append(candidate)
    if len(positions) < 4:
        return None
    times_array = np.asarray(times)
    positions_array = np.stack(positions)
    velocity, fit_median = _robust_velocity(times_array, positions_array)
    horizontal = _horizontal(velocity, target_yaw)
    vertical = math.degrees(
        math.atan2(float(velocity[2]), math.hypot(float(velocity[0]), float(velocity[1])))
    )
    speed = float(np.linalg.norm(velocity) * MPH_PER_MS)
    step_velocity = np.diff(positions_array, axis=0) / np.diff(times_array)[:, None]
    step_speeds = np.linalg.norm(step_velocity, axis=1) * MPH_PER_MS
    step_speed_mad = float(np.median(np.abs(step_speeds - np.median(step_speeds))))
    # Computing every contiguous Theil-Sen subwindow for every candidate path
    # is combinatorial. Adjacent-step direction dispersion is the same local
    # stability test and keeps the threshold sweep practical.
    step_angles = np.asarray([_horizontal(step, target_yaw) for step in step_velocity], dtype=float)
    window_mad = float(np.median(np.abs(step_angles - horizontal)))
    candidate_shapes = np.asarray(
        [
            abs(math.log(candidate.width / max(candidate.height, 1)))
            for candidate in used_candidates
        ],
        dtype=float,
    )
    shape_median = float(np.median(candidate_shapes))
    fill_median = float(np.median([candidate.fill for candidate in used_candidates]))
    circularity_median = float(np.median([candidate.circularity for candidate in used_candidates]))
    intensity_median = float(np.median([candidate.mean_intensity for candidate in used_candidates]))
    camera_origin = np.array([0.0, 0.0, camera_height_m])
    camera_ranges = np.linalg.norm(positions_array - camera_origin, axis=1)
    expected_diameter = focal_px * (BALL_DIAMETER_MM / 1000.0) / camera_ranges
    measured_diameter = np.asarray(
        [math.sqrt(4.0 * candidate.area / math.pi) for candidate in used_candidates],
        dtype=float,
    )
    size_ratio = measured_diameter / expected_diameter
    size_ratio_median = float(np.median(size_ratio))
    size_ratio_mad = float(np.median(np.abs(size_ratio - size_ratio_median)))
    if not (
        -30.0 <= horizontal <= 30.0
        and -5.0 <= vertical <= 55.0
        and 0.5 * ball_speed_mph <= speed <= 1.5 * ball_speed_mph
        and shape_median <= 0.55
        and fill_median >= 0.5
        and circularity_median >= 0.5
        and intensity_median >= 195.0
        and 0.45 <= size_ratio_median <= 2.5
        and size_ratio_mad <= 0.75
    ):
        return None

    # IWR vertical is a target-identity prior only. IWR horizontal is never
    # used to choose or tune the camera estimate.
    score = (
        10.0 * len(positions_array)
        - 400.0 * fit_median
        - 0.45 * abs(speed - ball_speed_mph)
        - 0.35 * step_speed_mad
        - 0.5 * abs(vertical - iwr_vertical_deg)
        - 6.0 * window_mad
        - 8.0 * shape_median
        + 6.0 * fill_median
        + 4.0 * circularity_median
        - 4.0 * size_ratio_mad
        - 2.0 * (actual_frames[0] - frame_indices[0])
    )
    bright, difference, min_area = thresholds
    return score, Estimate(
        shot=shot_number,
        bright_threshold=bright,
        difference_threshold=difference,
        min_area=min_area,
        horizontal_deg=horizontal,
        vertical_deg=vertical,
        speed_mph=speed,
        speed_error_mph=speed - ball_speed_mph,
        fit_median_m=fit_median,
        step_speed_mad_mph=step_speed_mad,
        window_mad_deg=window_mad,
        n_points=len(positions_array),
        first_frame=actual_frames[0],
        last_frame=actual_frames[-1],
    )


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_root", type=Path)
    parser.add_argument("--camera-height-m", type=float, default=0.20955)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    root = args.session_root.expanduser().resolve()
    session_files = sorted((root / "openflight_sessions").glob("session_*.jsonl"))
    if len(session_files) != 1:
        raise RuntimeError(f"expected one session JSONL under {root}, found {len(session_files)}")
    start, camera_events, radar_events, shot_events = _events(session_files[0])
    common = sorted(set(camera_events) & set(radar_events) & set(shot_events))
    anchor = _session_anchor(root, camera_events)
    config = start["config"]["iwr6843"]
    tee_range_m = float(config["tee_slant_range_m"])
    radar_height_m = float(config["radar_height_m"])
    ball_height_m = float(config["ball_height_m"])
    width = int(start["config"]["camera_capture"]["width"])
    height = int(start["config"]["camera_capture"]["height"])
    model = _camera_model(
        anchor,
        tee_range_m=tee_range_m,
        ball_height_m=ball_height_m,
        camera_height_m=args.camera_height_m,
        radar_height_m=radar_height_m,
        width=width,
        height=height,
    )

    logging.disable(logging.CRITICAL)
    estimates: list[Estimate] = []
    iwr_horizontal: dict[int, float] = {}
    for shot_number in common:
        camera_event = camera_events[shot_number]
        radar_event = radar_events[shot_number]
        archive = np.load(_local_capture(root, camera_event, "camera") / "frames.npz")
        frames = archive["frames"]
        timestamps_ns = archive["host_timestamp_ns"]
        trigger_ns = int(camera_event["metadata"]["trigger_host_timestamp_ns"])
        trigger_frame = int(np.argmin(np.abs(timestamps_ns.astype(np.int64) - trigger_ns)))
        frame_indices = list(range(trigger_frame, min(len(frames), trigger_frame + 15)))
        background = np.median(frames[:20], axis=0).astype(np.uint8)

        projected = project_tx_pair(_local_capture(root, radar_event, "radar").read_bytes(), (0, 2))
        metadata, cube = parse_dump(projected)
        geometry = geometry_from_header(metadata, loop_period_s=TX2_LOOP_PERIOD_S)
        mti = tracking.mti_filter(cube, range_domain=is_range_snapshot(metadata), geometry=geometry)
        ball_track = tracking.find_ball(
            mti,
            geometry,
            max_range_m=float(config["net_range_m"]),
            min_ball_ms=20,
        )
        if ball_track is None:
            continue
        impact_t_s = float(radar_event["measurement"]["impact_t_s"])
        ball_speed_mph = float(shot_events[shot_number]["ball_speed_mph"])
        iwr_vertical = float(radar_event["measurement"]["launch_angle_deg"])
        iwr_horizontal[shot_number] = float(radar_event["measurement"]["horizontal_deg"])

        for bright in (100, 115, 130):
            for difference in (12, 18, 24):
                for min_area in (5, 10, 20):
                    nodes = [
                        _candidates(
                            frames[frame],
                            background,
                            anchor,
                            bright_threshold=bright,
                            difference_threshold=difference,
                            min_area=min_area,
                        )
                        for frame in frame_indices
                    ]
                    options = []
                    for path in _pixel_paths(nodes, anchor):
                        estimate = _path_estimate(
                            shot_number=shot_number,
                            path=path,
                            frame_indices=frame_indices,
                            timestamps_ns=timestamps_ns,
                            trigger_ns=trigger_ns,
                            ball_track=ball_track,
                            impact_t_s=impact_t_s,
                            range_resolution_m=geometry.range_res_m,
                            ball_speed_mph=ball_speed_mph,
                            iwr_vertical_deg=iwr_vertical,
                            model=model,
                            camera_height_m=args.camera_height_m,
                            width=width,
                            height=height,
                            thresholds=(bright, difference, min_area),
                        )
                        if estimate is not None:
                            options.append(estimate)
                    if options:
                        estimates.append(max(options, key=lambda item: item[0])[1])

    output = args.output_dir or root / "camera_iwr_ball_sweep"
    output.mkdir(parents=True, exist_ok=True)
    _write_csv(output / "all_estimates.csv", [asdict(row) for row in estimates])

    aggregate = []
    for shot_number in common:
        rows = [row for row in estimates if row.shot == shot_number]
        if not rows:
            aggregate.append(
                {
                    "shot": shot_number,
                    "support": 0,
                    "support_pct": 0.0,
                    "camera_horizontal_deg": "",
                    "parameter_mad_deg": "",
                    "window_mad_deg": "",
                    "speed_error_mph": "",
                    "iwr_horizontal_deg": iwr_horizontal.get(shot_number, ""),
                    "camera_iwr_delta_deg": "",
                    "status": "withheld",
                }
            )
            continue
        horizontal = np.asarray([row.horizontal_deg for row in rows])
        median_horizontal = float(np.median(horizontal))
        parameter_mad = float(np.median(np.abs(horizontal - median_horizontal)))
        window_mad = float(np.median([row.window_mad_deg for row in rows]))
        speed_error = float(np.median([row.speed_error_mph for row in rows]))
        support_pct = 100.0 * len(rows) / 27.0
        stable = parameter_mad <= 1.0 and window_mad <= 1.5
        if len(rows) >= 9 and stable:
            status = "high"
        elif len(rows) >= 2 and stable:
            status = "experimental"
        else:
            status = "withheld"
        radar_horizontal = iwr_horizontal.get(shot_number)
        aggregate.append(
            {
                "shot": shot_number,
                "support": len(rows),
                "support_pct": round(support_pct, 1),
                "camera_horizontal_deg": round(median_horizontal, 3),
                "parameter_mad_deg": round(parameter_mad, 3),
                "window_mad_deg": round(window_mad, 3),
                "speed_error_mph": round(speed_error, 3),
                "iwr_horizontal_deg": round(radar_horizontal, 3),
                "camera_iwr_delta_deg": round(median_horizontal - radar_horizontal, 3),
                "status": status,
            }
        )
    _write_csv(output / "shot_summary.csv", aggregate)

    accepted = [row for row in aggregate if row["status"] != "withheld"]
    high = [row for row in aggregate if row["status"] == "high"]
    experimental = [row for row in aggregate if row["status"] == "experimental"]
    deltas = [abs(float(row["camera_iwr_delta_deg"])) for row in accepted]
    summary = {
        "shots_with_all_sensors": len(common),
        "accepted": len(accepted),
        "high_confidence": len(high),
        "experimental": len(experimental),
        "coverage_pct": round(100.0 * len(accepted) / len(common), 1) if common else 0.0,
        "median_parameter_mad_deg": round(
            statistics.median(float(row["parameter_mad_deg"]) for row in accepted), 3
        )
        if accepted
        else None,
        "median_window_mad_deg": round(
            statistics.median(float(row["window_mad_deg"]) for row in accepted), 3
        )
        if accepted
        else None,
        "median_abs_camera_iwr_delta_deg": round(statistics.median(deltas), 3) if deltas else None,
        "anchor": asdict(anchor),
        "camera_height_m": args.camera_height_m,
        "radar_height_m": radar_height_m,
        "camera_radar_vertical_separation_m": args.camera_height_m - radar_height_m,
        "focal_px_from_ball": round(model[0], 3),
        "camera_pitch_deg": round(math.degrees(model[1]), 3),
        "target_line_yaw_deg": round(math.degrees(model[2]), 3),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    print("\nshot  camera_H  IWR_H  delta  support  param_MAD  window_MAD  status")
    for row in aggregate:
        print(
            f"{row['shot']:>4} {str(row['camera_horizontal_deg']):>9} "
            f"{str(row['iwr_horizontal_deg']):>6} {str(row['camera_iwr_delta_deg']):>6} "
            f"{row['support_pct']:>7}% {str(row['parameter_mad_deg']):>9} "
            f"{str(row['window_mad_deg']):>10}  {row['status']}"
        )
    print(f"\nWrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
