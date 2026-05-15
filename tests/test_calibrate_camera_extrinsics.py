"""Tests for the camera->K-LD7 extrinsic solver in scripts/setup/calibrate_camera_extrinsics.py."""

import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).parent.parent / "scripts" / "setup" / "calibrate_camera_extrinsics.py"


@pytest.fixture(scope="module")
def calib_mod():
    spec = importlib.util.spec_from_file_location("calibrate_camera_extrinsics", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _sample(target_L, target_d, x_cam, y_cam, depth_cam):
    return {
        "target_L_in": target_L,
        "target_d_in": target_d,
        "x_cam_in": x_cam,
        "y_cam_in": y_cam,
        "depth_cam_in": depth_cam,
    }


class TestSolveExtrinsic:
    def test_two_centerline_samples_recovers_X_and_Z(self, calib_mod):
        # Camera mounted 3 in left of radar (X = -3) and 1.5 in forward (Z = +1.5).
        # Ball on centerline (L=0) at d=48 in -> appears in camera at x=-3, depth=49.5.
        # Ball on centerline (L=0) at d=96 in -> appears in camera at x=-3, depth=97.5.
        samples = [
            _sample(target_L=0.0, target_d=48.0, x_cam=-3.0, y_cam=4.0, depth_cam=49.5),
            _sample(target_L=0.0, target_d=96.0, x_cam=-3.0, y_cam=4.0, depth_cam=97.5),
        ]
        (X, Y, Z), (x_rms, z_rms) = calib_mod.solve_extrinsic(samples)
        assert X == pytest.approx(-3.0)
        assert Z == pytest.approx(1.5)
        assert Y == pytest.approx(4.0)
        assert x_rms == pytest.approx(0.0)
        assert z_rms == pytest.approx(0.0)

    def test_off_center_sample_recovers_X_with_correct_sign(self, calib_mod):
        # Camera 2 in right of radar (X = +2). Ball at L=+6 (right of radar) should
        # appear at x_cam = 6 - 2 = +4 (because ball is 4 in right of camera centerline).
        # Actually: ball_in_radar = ball_in_cam - radar_origin_in_cam => 6 = x_cam - 2 => x_cam = 8.
        # Let me redo: with X=+2 (radar to the right of camera), a ball at L=+6 in radar
        # frame is at x_cam = L + X = 6 + 2 = 8. So sample (L=+6, x_cam=8) -> X = 8 - 6 = 2.
        samples = [
            _sample(target_L=0.0, target_d=48.0, x_cam=2.0, y_cam=0.0, depth_cam=49.0),
            _sample(target_L=6.0, target_d=48.0, x_cam=8.0, y_cam=0.0, depth_cam=49.0),
        ]
        (X, _Y, Z), _ = calib_mod.solve_extrinsic(samples)
        assert X == pytest.approx(2.0)
        assert Z == pytest.approx(1.0)

    def test_noisy_samples_residual_reflects_noise(self, calib_mod):
        # True X = 0, true Z = 0. Add +/- 0.1 in x noise on each sample.
        samples = [
            _sample(target_L=0.0, target_d=48.0, x_cam=+0.1, y_cam=0.0, depth_cam=48.0),
            _sample(target_L=0.0, target_d=72.0, x_cam=-0.1, y_cam=0.0, depth_cam=72.0),
            _sample(target_L=0.0, target_d=96.0, x_cam=+0.1, y_cam=0.0, depth_cam=96.0),
        ]
        (X, _Y, Z), (x_rms, z_rms) = calib_mod.solve_extrinsic(samples)
        # Mean of (+0.1, -0.1, +0.1) = 0.0333; residuals should be small but non-zero
        assert abs(X) < 0.2
        assert Z == pytest.approx(0.0)
        assert x_rms > 0.0
        assert z_rms == pytest.approx(0.0)

    def test_single_sample_residuals_zero(self, calib_mod):
        samples = [_sample(target_L=0.0, target_d=60.0, x_cam=1.0, y_cam=2.0, depth_cam=61.0)]
        (X, Y, Z), (x_rms, z_rms) = calib_mod.solve_extrinsic(samples)
        assert X == pytest.approx(1.0)
        assert Y == pytest.approx(2.0)
        assert Z == pytest.approx(1.0)
        assert x_rms == 0.0
        assert z_rms == 0.0


class TestDetectionToCamXyz:
    def test_returns_none_when_no_detection(self, calib_mod):
        assert calib_mod.detection_to_cam_xyz(None, 640, 480, 800.0) is None

    def test_center_ball_at_depth_yields_zero_lateral(self, calib_mod):
        # Ball detected exactly at frame center, radius matches a depth of 60 in.
        # depth = ball_d * f / pixel_d => pixel_d = 1.68 * 800 / 60 = 22.4 px -> radius 11.2
        ball = _Ball(x=320, y=240, radius=11.2, confidence=1.0)
        x, y, depth = calib_mod.detection_to_cam_xyz(ball, 640, 480, 800.0)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)
        assert depth == pytest.approx(60.0, rel=1e-3)

    def test_right_offset_ball_has_positive_x(self, calib_mod):
        # Ball 100 px right of center, same radius as the centered case.
        ball = _Ball(x=420, y=240, radius=11.2, confidence=1.0)
        x, y, depth = calib_mod.detection_to_cam_xyz(ball, 640, 480, 800.0)
        # x_in = (100 / 800) * 60 = 7.5
        assert x == pytest.approx(7.5, rel=1e-3)
        assert y == pytest.approx(0.0)

    def test_below_center_ball_has_negative_y(self, calib_mod):
        # Ball 50 px below center should be y < 0 (Y+ is up).
        ball = _Ball(x=320, y=290, radius=11.2, confidence=1.0)
        x, y, depth = calib_mod.detection_to_cam_xyz(ball, 640, 480, 800.0)
        assert y == pytest.approx(-3.75, rel=1e-3)


class _Ball:
    """Minimal stand-in for DetectedBall — solver only reads .x .y .radius."""

    def __init__(self, x, y, radius, confidence=1.0):
        self.x = x
        self.y = y
        self.radius = radius
        self.confidence = confidence


class TestRoundTrip:
    def test_solved_extrinsic_inverts_measurements(self, calib_mod):
        """Apply solver, then use result to predict (L, d) for each sample; should match input."""
        samples = [
            _sample(target_L=0.0, target_d=48.0, x_cam=-3.0, y_cam=0.0, depth_cam=49.5),
            _sample(target_L=0.0, target_d=96.0, x_cam=-3.0, y_cam=0.0, depth_cam=97.5),
            _sample(target_L=+6.0, target_d=72.0, x_cam=3.0, y_cam=0.0, depth_cam=73.5),
        ]
        (X, _Y, Z), _ = calib_mod.solve_extrinsic(samples)
        for s in samples:
            L_pred = s["x_cam_in"] - X
            d_pred = s["depth_cam_in"] - Z
            assert L_pred == pytest.approx(s["target_L_in"], abs=0.01)
            assert d_pred == pytest.approx(s["target_d_in"], abs=0.01)
