"""Tests for the live camera club-delivery estimator and fusion."""

import math

import numpy as np
import pytest

from openflight.camera.club_delivery import (
    SCENE_P995_MIN,
    FusedDelivery,
    TraceResult,
    aoa_offset_for_club,
    estimate_delivery_trace,
    fuse_club_delivery,
)
from openflight.launch_monitor import ClubType

# ---------------------------------------------------------------------------
# Per-club AoA offsets
# ---------------------------------------------------------------------------


class TestAoaOffsets:
    def test_measured_clubs_return_measured_values(self):
        assert aoa_offset_for_club(ClubType.IRON_9) == (16.0, "measured")
        assert aoa_offset_for_club(ClubType.IRON_7) == (10.5, "measured")
        assert aoa_offset_for_club(ClubType.IRON_5) == (9.1, "measured")
        assert aoa_offset_for_club(ClubType.DRIVER) == (21.6, "measured")

    def test_mid_iron_interpolates_between_anchors(self):
        offset, source = aoa_offset_for_club(ClubType.IRON_8)  # loft 38, between 34 and 42
        assert source == "loft_interpolated"
        assert 10.5 < offset < 16.0

    def test_interpolation_is_monotone_with_loft(self):
        six, _ = aoa_offset_for_club(ClubType.IRON_6)
        eight, _ = aoa_offset_for_club(ClubType.IRON_8)
        assert 9.1 <= six <= 10.5
        assert 10.5 <= eight <= 16.0

    def test_wedges_clamp_to_last_anchor_not_extrapolate(self):
        # No measurements above the 9-iron: clamping is the conservative call.
        for club in (ClubType.PW, ClubType.SW, ClubType.LW):
            offset, source = aoa_offset_for_club(club)
            assert offset == 16.0
            assert source == "loft_interpolated"

    def test_long_irons_clamp_to_first_anchor(self):
        offset, source = aoa_offset_for_club(ClubType.IRON_3)
        assert offset == 9.1
        assert source == "loft_interpolated"

    def test_woods_and_hybrids_borrow_driver_offset(self):
        for club in (ClubType.WOOD_3, ClubType.HYBRID_5):
            offset, source = aoa_offset_for_club(club)
            assert offset == 21.6
            assert source == "class_extrapolated"

    def test_unknown_club_uses_iron_line(self):
        offset, source = aoa_offset_for_club(ClubType.UNKNOWN)
        assert source == "loft_interpolated"
        assert offset == 10.5  # mid-iron default loft = the 7-iron anchor


# ---------------------------------------------------------------------------
# Fusion math
# ---------------------------------------------------------------------------


class TestFusion:
    def _trace(self, deg: float) -> TraceResult:
        return TraceResult(status="ok", trace_deg=deg, n_pairs=3)

    def test_fused_reproduces_offline_9i_chain(self):
        # Radar candidate -20.4 + 16.0 = -4.4; trace -48.3 -> path ~ +3.9
        fused = fuse_club_delivery(-20.4, self._trace(-48.3), ClubType.IRON_9)
        assert fused.status == "fused"
        assert fused.attack_angle_deg == pytest.approx(-4.4, abs=0.05)
        expected_path = math.degrees(
            math.atan(math.tan(math.radians(-4.4)) / math.tan(math.radians(-48.3)))
        )
        assert fused.club_path_deg == pytest.approx(expected_path, abs=0.05)
        assert fused.club_path_deg > 0  # descending blow on a -48 trace = in-to-out

    def test_positive_aoa_negative_trace_is_inconsistent(self):
        # +3.6 corrected AoA (up hit) with a negative trace cannot both be
        # true: the trace's sign IS the AoA's sign. Blocked, AoA kept.
        fused = fuse_club_delivery(-18.0, self._trace(-48.0), ClubType.DRIVER)
        assert fused.status == "trace_aoa_sign_mismatch"
        assert fused.attack_angle_deg == pytest.approx(3.6, abs=0.05)
        assert fused.club_path_deg is None

    def test_no_radar_aoa(self):
        fused = fuse_club_delivery(None, self._trace(-48.0), ClubType.IRON_9)
        assert fused.status == "no_radar_aoa"
        assert fused.attack_angle_deg is None
        assert fused.club_path_deg is None

    def test_trace_failure_still_reports_corrected_aoa(self):
        fused = fuse_club_delivery(-20.4, TraceResult(status="low_light"), ClubType.IRON_9)
        assert fused.status == "trace_low_light"
        assert fused.attack_angle_deg == pytest.approx(-4.4, abs=0.05)
        assert fused.club_path_deg is None

    @pytest.mark.parametrize("trace_deg", [5.0, -10.0, 88.0, -89.0])
    def test_trace_out_of_range_blocks_path(self, trace_deg):
        fused = fuse_club_delivery(-20.4, self._trace(trace_deg), ClubType.IRON_9)
        assert fused.status == "trace_out_of_range"
        assert fused.club_path_deg is None
        assert fused.attack_angle_deg is not None

    def test_sign_mismatch_blocks_path(self):
        # Negative corrected AoA with a positive trace = camera locked onto
        # the wrong mover (observed live: flying-ball lock gave trace +19).
        fused = fuse_club_delivery(-20.4, self._trace(19.5), ClubType.IRON_9)
        assert fused.status == "trace_aoa_sign_mismatch"
        assert fused.club_path_deg is None
        assert fused.attack_angle_deg == pytest.approx(-4.4, abs=0.05)

    def test_driver_positive_aoa_positive_trace_is_legal(self):
        fused = fuse_club_delivery(-18.0, self._trace(78.0), ClubType.DRIVER)
        assert fused.status == "fused"  # +3.6 AoA with +78 trace is consistent
        assert fused.club_path_deg is not None and fused.club_path_deg > 0

    def test_offset_source_propagates(self):
        fused = fuse_club_delivery(-15.0, self._trace(-50.0), ClubType.IRON_8)
        assert fused.offset_source == "loft_interpolated"

    def test_returns_dataclass(self):
        assert isinstance(
            fuse_club_delivery(None, TraceResult(status="no_capture"), ClubType.IRON_9),
            FusedDelivery,
        )


# ---------------------------------------------------------------------------
# Trace estimation on synthetic captures
# ---------------------------------------------------------------------------

HEIGHT, WIDTH = 400, 640  # production capture geometry
BALL_XY = (400, 200)
FPS = 288.0


def _synthetic_capture(
    *,
    scene_level: int = 60,
    ball_bright: int = 245,
    club_bright: int = 240,
    club_velocity_px=(30.0, -18.0),
    n_frames: int = 40,
    impact_frame: int = 30,
) -> tuple[np.ndarray, np.ndarray]:
    """DTL-like capture: static bright ball on tee, bright club sweeping in.

    The club is a diagonal line segment with a blob (head) at its lower end,
    moving at ``club_velocity_px`` per frame toward the ball; the ball
    disappears after ``impact_frame``.
    """
    rng = np.random.default_rng(7)
    frames = np.full((n_frames, HEIGHT, WIDTH), scene_level, dtype=np.uint8)
    frames += rng.integers(0, 8, size=frames.shape, dtype=np.uint8)
    # a static bright distractor (ball pile) far from the tee
    frames[:, 360:380, 40:80] = 230
    # bright static "sky/background" band so scene brightness matches a real
    # daylight capture (the light gate keys on the background's p99.5)
    frames[:, 0:28, :] = min(255, int(scene_level * 2.4))

    yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
    ball_mask = (xx - BALL_XY[0]) ** 2 + (yy - BALL_XY[1]) ** 2 <= 5 * 5

    vx, vy = club_velocity_px
    for idx in range(n_frames):
        if idx <= impact_frame:
            frames[idx][ball_mask] = ball_bright
        steps_to_impact = impact_frame + 0.5 - idx
        head_x = BALL_XY[0] - vx * steps_to_impact
        head_y = BALL_XY[1] - vy * steps_to_impact
        if not (0 <= head_x < WIDTH and 0 <= head_y < HEIGHT):
            continue
        if abs(head_x - BALL_XY[0]) < 8 and abs(head_y - BALL_XY[1]) < 8 and idx <= impact_frame:
            continue  # don't overdraw the ball pre-impact
        # shaft: connected bright line from the head up-left; head: small blob
        import cv2

        cv2.line(
            frames[idx],
            (int(round(head_x)), int(round(head_y))),
            (int(round(head_x - 90)), int(round(head_y - 140))),
            int(club_bright),
            3,
        )
        hx, hy = int(round(head_x)), int(round(head_y))
        head_blob = (xx - hx) ** 2 + (yy - hy) ** 2 <= 6 * 6
        frames[idx][head_blob] = club_bright - 60  # head darker than shaft

    ts = (np.arange(n_frames) * (1e9 / FPS)).astype(np.int64)
    return frames, ts


class TestTraceEstimation:
    def test_bright_scene_produces_trace(self):
        frames, ts = _synthetic_capture()
        result = estimate_delivery_trace(frames, ts)
        assert result.status == "ok", result
        assert result.n_pairs >= 2
        # velocity (30, -18) px/frame, image y down -> trace = atan2(18, 30)
        expected = math.degrees(math.atan2(18.0, 30.0))
        assert result.trace_deg == pytest.approx(expected, abs=12.0)

    def test_impact_frame_detected_near_truth(self):
        frames, ts = _synthetic_capture(impact_frame=30)
        result = estimate_delivery_trace(frames, ts)
        assert result.impact_frame is not None
        assert abs(result.impact_frame - 30) <= 1

    def test_dim_scene_reports_low_light(self):
        frames, ts = _synthetic_capture(scene_level=20, ball_bright=90, club_bright=85)
        result = estimate_delivery_trace(frames, ts)
        assert result.status == "low_light"
        assert result.trace_deg is None
        assert result.scene_p995 is not None
        assert result.scene_p995 < SCENE_P995_MIN

    def test_saturation_near_ball_reports_overexposed(self):
        frames, ts = _synthetic_capture()
        # hot patch on the mat right next to the ball (inside the ball zone)
        frames[:, BALL_XY[1] + 30 : BALL_XY[1] + 90, BALL_XY[0] - 60 : BALL_XY[0] + 60] = 255
        result = estimate_delivery_trace(frames, ts)
        assert result.status == "overexposed"
        assert result.trace_deg is None

    def test_dappled_background_saturation_is_tolerated(self):
        frames, ts = _synthetic_capture()
        frames[:, 0:80, 0:200] = 255  # sunlit trees far from the hitting zone
        result = estimate_delivery_trace(frames, ts)
        assert result.status == "ok"

    def test_impact_far_from_trigger_is_implausible(self):
        # A never-departing ball patch (wrong-blob lock) pins "impact" to the
        # buffer end; the trigger-window check must catch it.
        frames, ts = _synthetic_capture(impact_frame=39)  # ball never leaves
        frames[39:, :, :] = frames[38]  # freeze the scene: ball stays put
        result = estimate_delivery_trace(frames, ts, trigger_index=20)
        assert result.status in ("impact_implausible", "no_impact")
        assert result.trace_deg is None

    def test_no_ball_status(self):
        frames, ts = _synthetic_capture()
        # erase the ball everywhere (keep the scene bright via the distractor)
        yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
        ball_mask = (xx - BALL_XY[0]) ** 2 + (yy - BALL_XY[1]) ** 2 <= 6 * 6
        frames[:, ball_mask] = 60
        frames[:, 10:60, 10:60] = 235  # keep p99.5 above the light gate
        result = estimate_delivery_trace(frames, ts)
        assert result.status in ("no_ball", "no_impact", "insufficient_pairs")
        assert result.trace_deg is None

    def test_rejects_malformed_input(self):
        result = estimate_delivery_trace(np.zeros((5, 4, 4), dtype=np.uint8), np.arange(5))
        assert result.status == "error"
