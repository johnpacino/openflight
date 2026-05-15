"""
Ball watcher: polls the camera between shots, caches the most recent
in-range ball position, and emits status transitions for the UI.

State machine:
    NOT_DETECTED  -- no ball found within stale_after_s
    IN_RANGE      -- ball detected and within the configured x/y/d bounds
    OUT_OF_RANGE  -- ball detected but outside one or more bounds

At shot time, the server reads `latest_ball_position` (None when status
is NOT_DETECTED) and passes it to the horizontal K-LD7 tracker.

The watcher does not own the camera lifecycle — it polls a `frame_provider`
callable. This keeps it testable (the callable is the test seam) and
lets the server decide whether to share a single picamera2 instance
or open one specifically for the watcher.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from .extrinsics import (
    BallPosition,
    CameraExtrinsics,
    camera_to_radar_position,
)


logger = logging.getLogger(__name__)


STATE_NOT_DETECTED = "NOT_DETECTED"
STATE_IN_RANGE = "IN_RANGE"
STATE_OUT_OF_RANGE = "OUT_OF_RANGE"

BALL_DIAMETER_IN = 1.68


@dataclass(frozen=True)
class AimStatus:
    """Snapshot of ball-watcher state. Emitted on state change."""

    state: str  # one of STATE_NOT_DETECTED / STATE_IN_RANGE / STATE_OUT_OF_RANGE
    ball_position: Optional[BallPosition]
    reason: Optional[str]  # human-readable, e.g. "L=+12in exceeds +8 limit"
    timestamp: float


def _detection_to_camera_xyz(
    detection: Any,
    frame_w: int,
    frame_h: int,
    focal_px: float,
) -> Optional[tuple[float, float, float]]:
    """Pixel ball detection -> (x_cam_in, y_cam_in, depth_cam_in)."""
    if detection is None:
        return None
    pixel_diameter = max(1.0, 2.0 * float(detection.radius))
    depth_in = (BALL_DIAMETER_IN * focal_px) / pixel_diameter
    dx = float(detection.x) - frame_w / 2.0
    dy = float(detection.y) - frame_h / 2.0
    x_in = (dx / focal_px) * depth_in
    y_in = -(dy / focal_px) * depth_in  # image y down -> world Y+ up
    return x_in, y_in, depth_in


def _out_of_range_reason(
    bp: BallPosition,
    x_range: Optional[tuple[float, float]],
    y_range: Optional[tuple[float, float]],
    d_range: Optional[tuple[float, float]],
) -> Optional[str]:
    """Human-readable explanation for why a position is out of range, or
    None if it is in range."""
    reasons = []
    if x_range is not None and not (x_range[0] <= bp.L_in <= x_range[1]):
        delta = bp.L_in - (x_range[1] if bp.L_in > x_range[1] else x_range[0])
        side = "right" if delta > 0 else "left"
        reasons.append(f"L={bp.L_in:+.1f}in is {abs(delta):.1f}in too far {side}")
    if y_range is not None and not (y_range[0] <= bp.h_in <= y_range[1]):
        reasons.append(f"h={bp.h_in:+.1f}in outside [{y_range[0]:+.1f}, {y_range[1]:+.1f}]")
    if d_range is not None and not (d_range[0] <= bp.d_initial_in <= d_range[1]):
        reasons.append(
            f"d={bp.d_initial_in:.1f}in outside [{d_range[0]:.0f}, {d_range[1]:.0f}]"
        )
    return "; ".join(reasons) if reasons else None


class BallWatcher:
    """Background polling of the camera with state-machine status output.

    Production use (in the server):
        watcher = BallWatcher(
            frame_provider=lambda: camera_capture.capture_single(),
            extrinsics=ext,
            radar_name="horizontal_kld7",
            detector=BallDetector(det_cfg),
            rotate_180=True,
            ball_x_range=(-8.0, 4.0),
            ball_d_range=(48.0, 72.0),
            status_callback=lambda s: socketio.emit("aim_status", asdict(s)),
        )
        watcher.start()
        ...
        bp = watcher.latest_ball_position  # at shot time
        ...
        watcher.stop()
    """

    def __init__(
        self,
        frame_provider: Callable[[], Optional[Any]],
        extrinsics: CameraExtrinsics,
        radar_name: str = "horizontal_kld7",
        detector: Any = None,
        rotate_180: bool = False,
        ball_x_range: Optional[tuple[float, float]] = None,
        ball_y_range: Optional[tuple[float, float]] = None,
        ball_d_range: Optional[tuple[float, float]] = None,
        poll_interval_s: float = 0.5,
        stale_after_s: float = 3.0,
        status_callback: Optional[Callable[["AimStatus"], None]] = None,
        clock: Callable[[], float] = time.time,
    ):
        self._frame_provider = frame_provider
        self._extrinsics = extrinsics
        self._radar_name = radar_name
        self._detector = detector  # BallDetector or compatible duck type
        self._rotate_180 = rotate_180
        self._ball_x_range = ball_x_range
        self._ball_y_range = ball_y_range
        self._ball_d_range = ball_d_range
        self._poll_interval_s = poll_interval_s
        self._stale_after_s = stale_after_s
        self._status_callback = status_callback
        self._clock = clock

        self._lock = threading.Lock()
        self._status = AimStatus(
            state=STATE_NOT_DETECTED,
            ball_position=None,
            reason="watcher just started; no frames processed yet",
            timestamp=clock(),
        )
        self._last_in_or_out_t: Optional[float] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    # ----- Public API -----

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, name="BallWatcher", daemon=True)
        self._thread.start()
        logger.info("BallWatcher started (poll=%ss, radar=%s)",
                    self._poll_interval_s, self._radar_name)

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=self._poll_interval_s * 4)
            self._thread = None
        logger.info("BallWatcher stopped")

    @property
    def current_status(self) -> AimStatus:
        with self._lock:
            return self._status

    @property
    def latest_ball_position(self) -> Optional[BallPosition]:
        """Returns the cached BallPosition only when the ball is currently
        in range. Out-of-range and not-detected return None — the caller
        then falls back to the existing pre-camera pipeline."""
        with self._lock:
            if self._status.state == STATE_IN_RANGE:
                return self._status.ball_position
            return None

    def latest_ball_position_with_age_ms(
        self, at_time: Optional[float] = None
    ) -> tuple[Optional[BallPosition], Optional[float]]:
        """Returns (position, age_ms_relative_to_at_time). The age is how
        old the most recent IN_RANGE detection is relative to `at_time`
        (the shot's impact timestamp, typically). Returns (None, None)
        when nothing in-range is cached."""
        with self._lock:
            if self._status.state != STATE_IN_RANGE or self._status.ball_position is None:
                return None, None
            bp = self._status.ball_position
        now = at_time if at_time is not None else self._clock()
        age_ms = (now - bp.timestamp) * 1000.0
        return bp, age_ms

    # ----- Single-frame processing (also the test seam) -----

    def process_one_frame(self) -> AimStatus:
        """Synchronously poll one frame and update the state machine.
        Used by the run loop, and useful in tests."""
        frame = self._frame_provider()
        now = self._clock()
        detection = self._run_detector(frame) if frame is not None else None
        if detection is None:
            return self._handle_missed(now)
        return self._handle_detection(frame, detection, now)

    # ----- Internal -----

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.process_one_frame()
            except Exception as exc:
                logger.warning("BallWatcher poll failed: %s", exc, exc_info=True)
            self._stop_event.wait(self._poll_interval_s)

    def _run_detector(self, frame: Any) -> Any:
        if self._detector is None:
            return None
        # Apply rotation if needed. We import cv2 lazily to keep import
        # of this module cheap during tests that mock the detector path.
        if self._rotate_180:
            try:
                import cv2  # type: ignore
                rotated_data = cv2.rotate(frame.data, cv2.ROTATE_180)
                from .capture import CapturedFrame
                frame = CapturedFrame(data=rotated_data, timestamp=frame.timestamp,
                                      frame_number=frame.frame_number)
            except ImportError:
                pass
        return self._detector.detect(frame)

    def _handle_missed(self, now: float) -> AimStatus:
        if self._last_in_or_out_t is None:
            # Never seen the ball; stay in NOT_DETECTED quietly.
            return self.current_status
        elapsed = now - self._last_in_or_out_t
        if elapsed < self._stale_after_s:
            # Recent detection — keep the cached position warm. Don't
            # transition until truly stale.
            return self.current_status
        return self._set_status(AimStatus(
            state=STATE_NOT_DETECTED,
            ball_position=None,
            reason=f"no detection for {elapsed:.1f}s",
            timestamp=now,
        ))

    def _handle_detection(self, frame: Any, detection: Any, now: float) -> AimStatus:
        cam_xyz = _detection_to_camera_xyz(
            detection,
            frame_w=frame.data.shape[1],
            frame_h=frame.data.shape[0],
            focal_px=self._extrinsics.focal_px,
        )
        if cam_xyz is None:
            return self._handle_missed(now)
        x_cam, y_cam, depth_cam = cam_xyz
        bp = camera_to_radar_position(
            x_cam_in=x_cam, y_cam_in=y_cam, depth_cam_in=depth_cam,
            radar_name=self._radar_name, extrinsics=self._extrinsics,
            confidence=float(getattr(detection, "confidence", 0.0)),
            timestamp=now,
        )
        if bp is None:
            # Radar not in extrinsics — caller asked us for something we
            # haven't been calibrated against. Treat as no detection.
            return self._handle_missed(now)

        self._last_in_or_out_t = now
        oor_reason = _out_of_range_reason(
            bp, self._ball_x_range, self._ball_y_range, self._ball_d_range
        )
        if oor_reason is None:
            return self._set_status(AimStatus(
                state=STATE_IN_RANGE,
                ball_position=bp,
                reason=None,
                timestamp=now,
            ))
        return self._set_status(AimStatus(
            state=STATE_OUT_OF_RANGE,
            ball_position=bp,
            reason=oor_reason,
            timestamp=now,
        ))

    def _set_status(self, new: AimStatus) -> AimStatus:
        with self._lock:
            old = self._status
            self._status = new
        if new.state != old.state:
            logger.info("BallWatcher state %s -> %s (%s)", old.state, new.state, new.reason)
            if self._status_callback is not None:
                try:
                    self._status_callback(new)
                except Exception as exc:
                    logger.warning("BallWatcher status_callback raised: %s", exc, exc_info=True)
        return new
