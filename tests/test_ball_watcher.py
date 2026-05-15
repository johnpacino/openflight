"""
Tests for BallWatcher state machine and status transitions.

Uses synthetic frames + mock detections — no real camera. Each test
drives the state machine via `process_one_frame()` (the test seam),
not the background thread.
"""

import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Optional

import pytest


_SRC = Path(__file__).parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))


from openflight.camera.ball_watcher import (
    BallWatcher,
    AimStatus,
    STATE_NOT_DETECTED,
    STATE_IN_RANGE,
    STATE_OUT_OF_RANGE,
    _out_of_range_reason,
    _detection_to_camera_xyz,
)
from openflight.camera.extrinsics import (
    BallPosition,
    CameraExtrinsics,
    RadarOrigin,
)


# ----------------------------------------------------------------------------
# Test fixtures
# ----------------------------------------------------------------------------


@dataclass
class _FakeFrame:
    data: object  # has a .shape attribute (we use namespace below)
    timestamp: float = 0.0
    frame_number: int = 0


def _frame_640x480():
    """Returns a fake CapturedFrame-like object with a numpy-compatible shape."""
    return _FakeFrame(
        data=SimpleNamespace(shape=(480, 640, 3)),
    )


def _detection(x=320.0, y=240.0, radius=11.0, confidence=0.2):
    return SimpleNamespace(x=x, y=y, radius=radius, confidence=confidence)


def _ext(radar_x=0.0, radar_z=0.0, focal_px=811.0):
    return CameraExtrinsics(
        focal_px=focal_px,
        resolution=(640, 480),
        ball_diameter_in=1.68,
        radars={"horizontal_kld7": RadarOrigin(x=radar_x, y=0.0, z=radar_z)},
    )


class _MockClock:
    def __init__(self, t0=1000.0):
        self.t = t0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class _MockDetector:
    """Always returns the same detection (or None) regardless of frame."""

    def __init__(self, detection=None):
        self.detection = detection

    def detect(self, _frame):
        return self.detection


def _make_watcher(
    detection=None,
    frame=None,
    extrinsics=None,
    ball_x_range=None,
    ball_d_range=None,
    stale_after_s=3.0,
    clock=None,
    status_callback=None,
):
    if frame is None:
        frame = _frame_640x480()
    if extrinsics is None:
        extrinsics = _ext()
    if clock is None:
        clock = _MockClock()
    return BallWatcher(
        frame_provider=lambda: frame,
        extrinsics=extrinsics,
        radar_name="horizontal_kld7",
        detector=_MockDetector(detection),
        rotate_180=False,
        ball_x_range=ball_x_range,
        ball_d_range=ball_d_range,
        stale_after_s=stale_after_s,
        status_callback=status_callback,
        clock=clock,
    )


# ----------------------------------------------------------------------------
# Pure helpers
# ----------------------------------------------------------------------------


class TestDetectionToCameraXyz:
    def test_returns_none_for_no_detection(self):
        assert _detection_to_camera_xyz(None, 640, 480, 811.0) is None

    def test_center_ball_at_known_radius(self):
        # 1.68in ball at radius 11.2px with focal 811: depth = 1.68*811/22.4 = 60.8in
        det = _detection(x=320.0, y=240.0, radius=11.2)
        x, y, depth = _detection_to_camera_xyz(det, 640, 480, 811.0)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)
        assert depth == pytest.approx(60.8, rel=1e-2)


class TestOutOfRangeReason:
    def _bp(self, L=0.0, h=0.0, d=60.0):
        return BallPosition(L_in=L, d_initial_in=d, h_in=h, confidence=0.2, timestamp=1.0)

    def test_in_range_returns_none(self):
        assert _out_of_range_reason(
            self._bp(L=2.0, d=60.0),
            x_range=(-8.0, 4.0), y_range=None, d_range=(48.0, 72.0),
        ) is None

    def test_too_far_right(self):
        reason = _out_of_range_reason(
            self._bp(L=8.0),
            x_range=(-8.0, 4.0), y_range=None, d_range=None,
        )
        assert reason is not None
        assert "right" in reason
        assert "4.0in" in reason  # 8 - 4 = 4 too far

    def test_too_far_left(self):
        reason = _out_of_range_reason(
            self._bp(L=-10.0),
            x_range=(-8.0, 4.0), y_range=None, d_range=None,
        )
        assert reason is not None
        assert "left" in reason

    def test_depth_out_of_range(self):
        reason = _out_of_range_reason(
            self._bp(d=80.0),
            x_range=None, y_range=None, d_range=(48.0, 72.0),
        )
        assert reason is not None
        assert "80" in reason


# ----------------------------------------------------------------------------
# State machine
# ----------------------------------------------------------------------------


class TestInitialState:
    def test_starts_in_not_detected(self):
        w = _make_watcher(detection=None)
        assert w.current_status.state == STATE_NOT_DETECTED
        assert w.latest_ball_position is None


class TestDetectionTransitionsToInRange:
    def test_centered_ball_no_bounds_is_in_range(self):
        # Ball centered, no bounds set → IN_RANGE
        det = _detection(x=320.0, y=240.0, radius=11.0)
        w = _make_watcher(detection=det)
        status = w.process_one_frame()
        assert status.state == STATE_IN_RANGE
        assert status.ball_position is not None
        assert w.latest_ball_position is not None

    def test_off_axis_ball_outside_bounds_is_out_of_range(self):
        # Ball detected far right; ROI says max +4in lateral
        # With focal 811 and depth ~60, cx_offset of 100px → L ≈ 7.4in
        det = _detection(x=420.0, y=240.0, radius=11.2)
        w = _make_watcher(
            detection=det,
            ball_x_range=(-8.0, 4.0),
        )
        status = w.process_one_frame()
        assert status.state == STATE_OUT_OF_RANGE
        assert "right" in (status.reason or "")
        # latest_ball_position returns None when OUT_OF_RANGE (we don't
        # want the server applying correction with an out-of-range ball)
        assert w.latest_ball_position is None


class TestDetectionLossTransitions:
    def test_stays_warm_briefly_after_loss(self):
        clock = _MockClock(1000.0)
        det = _detection(x=320, y=240, radius=11.0)
        w = _make_watcher(detection=det, clock=clock, stale_after_s=3.0)
        # First poll: detection → IN_RANGE
        assert w.process_one_frame().state == STATE_IN_RANGE
        # Lose the detection
        w._detector.detection = None
        clock.advance(1.0)  # only 1 sec elapsed
        status = w.process_one_frame()
        assert status.state == STATE_IN_RANGE  # still warm

    def test_transitions_to_not_detected_after_stale(self):
        clock = _MockClock(1000.0)
        det = _detection(x=320, y=240, radius=11.0)
        w = _make_watcher(detection=det, clock=clock, stale_after_s=3.0)
        w.process_one_frame()  # → IN_RANGE
        w._detector.detection = None
        clock.advance(5.0)  # past stale_after_s
        status = w.process_one_frame()
        assert status.state == STATE_NOT_DETECTED
        assert w.latest_ball_position is None


class TestStatusCallback:
    def test_callback_fires_only_on_state_change(self):
        events = []

        def cb(s: AimStatus):
            events.append(s.state)

        clock = _MockClock(1000.0)
        det = _detection(x=320, y=240, radius=11.0)
        w = _make_watcher(detection=det, clock=clock, status_callback=cb)
        # Multiple polls with the same detection → only ONE callback (the
        # NOT_DETECTED → IN_RANGE transition).
        w.process_one_frame()
        w.process_one_frame()
        w.process_one_frame()
        assert events == [STATE_IN_RANGE]

    def test_callback_fires_again_on_real_transition(self):
        events = []

        def cb(s: AimStatus):
            events.append(s.state)

        clock = _MockClock(1000.0)
        det = _detection(x=320, y=240, radius=11.0)
        w = _make_watcher(detection=det, clock=clock, status_callback=cb,
                          ball_x_range=(-8.0, 4.0))
        w.process_one_frame()  # → IN_RANGE
        # Now move ball out of range
        w._detector.detection = _detection(x=480, y=240, radius=11.0)
        w.process_one_frame()
        assert events == [STATE_IN_RANGE, STATE_OUT_OF_RANGE]


class TestLatestBallPositionWithAge:
    def test_returns_none_when_not_detected(self):
        w = _make_watcher(detection=None)
        bp, age = w.latest_ball_position_with_age_ms()
        assert bp is None
        assert age is None

    def test_returns_age_relative_to_at_time(self):
        clock = _MockClock(1000.0)
        det = _detection(x=320, y=240, radius=11.0)
        w = _make_watcher(detection=det, clock=clock)
        w.process_one_frame()  # cached at t=1000
        bp, age = w.latest_ball_position_with_age_ms(at_time=1000.5)
        assert bp is not None
        assert age == pytest.approx(500.0)  # 500ms after detection
