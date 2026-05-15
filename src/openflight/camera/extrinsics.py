"""
Camera -> K-LD7 extrinsic transform: dataclasses, loader, and the conversion
from camera-frame ball position to radar-frame (L, d, h).

Calibration JSON is produced by scripts/setup/calibrate_camera_extrinsics.py
and lives at ~/openflight_calibration.json by default. Loading is lenient —
missing file or malformed JSON returns None so the runtime can fall back
to the pre-camera pipeline.

Sign conventions (all inches, all signed):
    Camera frame: X+ right, Y+ up, Z+ forward.
    Radar frame:  L+ right of antenna centerline, d+ forward of antenna face,
                  h+ above antenna height.
The radar's origin in the camera frame is what's saved in the JSON. To convert
a ball measured in the camera frame to the radar frame, subtract that origin.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class BallPosition:
    """Ball position in the K-LD7 radar's reference frame.

    Used by the aim-correction pipeline to subtract per-frame geometric
    bearing. `d_initial_in` is the forward distance at the moment of
    impact (or the most recent camera observation); the radar pipeline
    advances this by ball_speed * t for each subsequent frame.
    """

    L_in: float                 # lateral offset, + right of centerline
    d_initial_in: float         # forward distance from antenna face
    h_in: float                 # vertical offset, + above antenna (informational)
    confidence: float           # detector confidence at this measurement
    timestamp: float            # epoch seconds when the camera captured the ball

    def is_in_range(
        self,
        x_range: tuple[float, float] | None = None,
        d_range: tuple[float, float] | None = None,
        y_range: tuple[float, float] | None = None,
    ) -> bool:
        """True if all provided ranges contain this position.

        Used by the ball watcher to gate "IN_RANGE" vs "OUT_OF_RANGE"
        status. Bounds are inclusive (min <= value <= max).
        """
        if x_range is not None and not (x_range[0] <= self.L_in <= x_range[1]):
            return False
        if d_range is not None and not (d_range[0] <= self.d_initial_in <= d_range[1]):
            return False
        if y_range is not None and not (y_range[0] <= self.h_in <= y_range[1]):
            return False
        return True


@dataclass(frozen=True)
class RadarOrigin:
    """Location of a K-LD7 radar's antenna face in the camera's frame."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class CameraExtrinsics:
    """Calibrated camera -> K-LD7 transform, loaded from JSON."""

    focal_px: float
    resolution: tuple[int, int]
    ball_diameter_in: float
    radars: dict[str, RadarOrigin]


def load_extrinsics(path: Path | str | None = None) -> Optional[CameraExtrinsics]:
    """Load calibration JSON. Returns None if the file is missing or invalid.

    The runtime treats None as "no camera correction available" and falls
    back to the existing static-offset pipeline.
    """
    if path is None:
        path = Path.home() / "openflight_calibration.json"
    path = Path(path)

    if not path.exists():
        logger.info("camera extrinsics: %s not found, aim correction disabled", path)
        return None

    try:
        with path.open("r") as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError) as e:
        logger.warning("camera extrinsics: failed to read %s (%s); disabling", path, e)
        return None

    try:
        focal_px = float(data["focal_px"])
        res = data.get("resolution", [640, 480])
        resolution = (int(res[0]), int(res[1]))
        ball_diameter_in = float(data.get("ball_diameter_in", 1.68))
        radars: dict[str, RadarOrigin] = {}
        for name, entry in (data.get("radars") or {}).items():
            origin = entry.get("radar_origin_in_camera_frame_in") or {}
            radars[name] = RadarOrigin(
                x=float(origin.get("x", 0.0)),
                y=float(origin.get("y", 0.0)),
                z=float(origin.get("z", 0.0)),
            )
    except (KeyError, TypeError, ValueError) as e:
        logger.warning("camera extrinsics: malformed JSON at %s (%s); disabling", path, e)
        return None

    if not radars:
        logger.warning("camera extrinsics: %s has no radars; disabling", path)
        return None

    return CameraExtrinsics(
        focal_px=focal_px,
        resolution=resolution,
        ball_diameter_in=ball_diameter_in,
        radars=radars,
    )


def camera_to_radar_position(
    x_cam_in: float,
    y_cam_in: float,
    depth_cam_in: float,
    radar_name: str,
    extrinsics: CameraExtrinsics,
    confidence: float,
    timestamp: float,
) -> Optional[BallPosition]:
    """Convert camera-frame measurement to radar-frame BallPosition.

    Returns None if the named radar isn't in the loaded extrinsics
    (e.g., user only calibrated horizontal but vertical was requested).
    """
    origin = extrinsics.radars.get(radar_name)
    if origin is None:
        return None
    L_in = x_cam_in - origin.x
    h_in = y_cam_in - origin.y
    d_in = depth_cam_in - origin.z
    return BallPosition(
        L_in=L_in,
        d_initial_in=d_in,
        h_in=h_in,
        confidence=confidence,
        timestamp=timestamp,
    )
