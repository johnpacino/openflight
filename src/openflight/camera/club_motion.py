"""Offline OV9281 club-motion measurements around impact.

The tracker intentionally reports image-plane motion only. Converting these
measurements to club path or attack angle requires camera-pose calibration and
an independent downrange velocity measurement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy import ndimage

BALL_DIAMETER_MM = 42.67


@dataclass(frozen=True)
class ReferenceBall:
    """Stationary ball location and apparent size before the swing."""

    x: float
    y: float
    diameter_px: float
    area_px: int


@dataclass(frozen=True)
class ImagePoint:
    """Tracked image coordinate for one camera frame."""

    frame_index: int
    x: float
    y: float


@dataclass(frozen=True)
class ImagePlaneMotion:
    """Terminal transverse motion measured in the camera image plane."""

    horizontal_px_s: float
    vertical_px_s: float
    horizontal_m_s: float
    vertical_m_s: float
    mm_per_px: float
    interval_ms: float


@dataclass(frozen=True)
class ShaftTrack:
    """Bright-shaft endpoint track leading into impact."""

    points: tuple[ImagePoint, ...]
    confidence: float
    reason: str


def detect_reference_ball(
    frames: np.ndarray,
    *,
    roi: tuple[int, int, int, int] | None = None,
    brightness_threshold: int = 210,
) -> ReferenceBall:
    """Find the central round bright object in the pre-impact background."""
    if frames.ndim != 3 or frames.shape[0] < 3:
        raise ValueError("frames must have shape (n, height, width) with n >= 3")

    background = np.median(frames[: min(20, frames.shape[0])], axis=0)
    height, width = background.shape
    x0, y0, x1, y1 = roi or (0, 0, width, height)
    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
        raise ValueError("ball ROI is outside the image")

    mask = np.zeros_like(background, dtype=bool)
    mask[y0:y1, x0:x1] = background[y0:y1, x0:x1] >= brightness_threshold
    labels, count = ndimage.label(mask)
    candidates: list[tuple[float, ReferenceBall]] = []
    image_center = np.asarray((width / 2, height / 2))

    for label in range(1, count + 1):
        ys, xs = np.where(labels == label)
        area = len(xs)
        if not 30 <= area <= 600:
            continue
        component_width = int(np.ptp(xs)) + 1
        component_height = int(np.ptp(ys)) + 1
        aspect = component_width / component_height
        fill = area / (component_width * component_height)
        if not (0.55 <= aspect <= 1.8 and 0.45 <= fill <= 1.0):
            continue
        center = np.asarray((float(xs.mean()), float(ys.mean())))
        diameter = math.sqrt(4 * area / math.pi)
        score = float(np.linalg.norm(center - image_center)) + abs(aspect - 1.0) * 20
        candidates.append(
            (
                score,
                ReferenceBall(
                    x=float(center[0]),
                    y=float(center[1]),
                    diameter_px=diameter,
                    area_px=area,
                ),
            )
        )

    if not candidates:
        raise ValueError("no round bright reference ball found")
    return min(candidates, key=lambda item: item[0])[1]


def _shaft_candidates(
    frame: np.ndarray,
    background: np.ndarray,
    ball: ReferenceBall,
    *,
    bright_threshold: int,
    difference_threshold: int,
) -> list[tuple[float, float, float]]:
    """Return Hough shaft endpoints as (score, x, y)."""
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - depends on optional analysis extra
        raise RuntimeError(
            "camera club tracking needs the analysis extra: uv run --extra analysis ..."
        ) from exc

    background_u8 = background.astype(np.uint8)
    difference = cv2.absdiff(frame, background_u8)
    mask = ((frame > bright_threshold) & (difference > difference_threshold)).astype(np.uint8)
    mask *= 255
    height, width = frame.shape
    mask[: max(10, height // 20)] = 0
    mask[min(height, int(height * 0.7)) :] = 0
    mask[:, : max(10, int(width * 0.15))] = 0
    mask[:, min(width, int(width * 0.85)) :] = 0

    lines = cv2.HoughLinesP(
        mask,
        1,
        np.pi / 360,
        threshold=25,
        minLineLength=25,
        maxLineGap=12,
    )
    if lines is None:
        return []

    candidates: list[tuple[float, float, float]] = []
    for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
        dx = float(x2 - x1)
        dy = float(y2 - y1)
        length = math.hypot(dx, dy)
        verticality = abs(dy) / (abs(dx) + 1.0)
        if verticality < 0.7:
            continue
        distance1 = math.hypot(x1 - ball.x, y1 - ball.y)
        distance2 = math.hypot(x2 - ball.x, y2 - ball.y)
        endpoint_x, endpoint_y, distance = (
            (float(x1), float(y1), distance1)
            if distance1 < distance2
            else (float(x2), float(y2), distance2)
        )
        if distance > max(frame.shape) * 0.35:
            continue
        candidates.append((distance - length * 0.08, endpoint_x, endpoint_y))
    return candidates


def track_bright_shaft_endpoint(
    frames: np.ndarray,
    *,
    trigger_frame_index: int,
    ball: ReferenceBall,
    frames_before_trigger: int = 4,
    bright_threshold: int = 145,
    difference_threshold: int = 25,
) -> ShaftTrack:
    """Track the lower endpoint of the bright shaft before the trigger frame."""
    if frames.ndim != 3:
        raise ValueError("frames must have shape (n, height, width)")
    first = trigger_frame_index - frames_before_trigger
    if first < 0 or trigger_frame_index > len(frames):
        raise ValueError("trigger frame does not leave the requested pre-impact window")

    background = np.median(frames[: min(20, len(frames))], axis=0)
    points: list[ImagePoint] = []
    for frame_index in range(first, trigger_frame_index):
        candidates = _shaft_candidates(
            frames[frame_index],
            background,
            ball,
            bright_threshold=bright_threshold,
            difference_threshold=difference_threshold,
        )
        if not candidates:
            continue
        _, x, y = min(candidates, key=lambda candidate: candidate[0])
        points.append(ImagePoint(frame_index=frame_index, x=x, y=y))

    if len(points) < 2:
        return ShaftTrack(tuple(points), 0.0, "insufficient_shaft_endpoints")

    distances = np.asarray([math.hypot(point.x - ball.x, point.y - ball.y) for point in points])
    decreasing_fraction = float(np.mean(np.diff(distances) < 0)) if len(points) > 1 else 0.0
    coverage = len(points) / frames_before_trigger
    confidence = min(1.0, 0.65 * coverage + 0.35 * decreasing_fraction)
    reason = "ok" if len(points) == frames_before_trigger else "partial_preimpact_track"
    return ShaftTrack(tuple(points), confidence, reason)


def image_plane_motion(
    points: Sequence[ImagePoint],
    timestamps_ns: np.ndarray,
    *,
    ball_diameter_px: float,
) -> ImagePlaneMotion:
    """Calculate terminal image-plane motion from the final adjacent points."""
    if len(points) < 2:
        raise ValueError("at least two tracked points are required")
    first, second = points[-2:]
    if second.frame_index != first.frame_index + 1:
        raise ValueError("terminal tracked points must be consecutive")
    if ball_diameter_px <= 0:
        raise ValueError("ball diameter must be positive")

    delta_s = (int(timestamps_ns[second.frame_index]) - int(timestamps_ns[first.frame_index])) / 1e9
    if delta_s <= 0:
        raise ValueError("frame timestamps must increase")
    horizontal_px_s = (second.x - first.x) / delta_s
    # Image y grows downward; physical vertical velocity uses the opposite sign.
    vertical_px_s = -(second.y - first.y) / delta_s
    mm_per_px = BALL_DIAMETER_MM / ball_diameter_px
    return ImagePlaneMotion(
        horizontal_px_s=horizontal_px_s,
        vertical_px_s=vertical_px_s,
        horizontal_m_s=horizontal_px_s * mm_per_px / 1000.0,
        vertical_m_s=vertical_px_s * mm_per_px / 1000.0,
        mm_per_px=mm_per_px,
        interval_ms=delta_s * 1000.0,
    )
