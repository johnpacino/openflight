"""End-to-end tests for the openflight.iwr6843 shot pipeline.

A synthetic moving ball is packed into the exact wire format (pack_dump is
the executable spec the firmware mirrors), then the full chain — parse, MTI,
track, TDM correction, MUSIC, trajectory fits — must recover the truth.
Conventions locked here: ImRe byte order, TDM phase sign, physical-orientation
flip, header-driven geometry and time order.
"""
from __future__ import annotations

import numpy as np
import pytest

from openflight.iwr6843 import Calibration, process_dump
from openflight.iwr6843.dump import pack_dump, parse_header
from openflight.iwr6843.music import LAM, steer
from openflight.iwr6843.shot import geometry_from_header
from openflight.iwr6843.tracking import LOOP_PRI_S, RANGE_SPAN_M

TAU_S = 45e-6
RADAR_HEIGHT_M = 0.152


def synth_shot(*, speed_ms=45.0, launch_deg=18.0, tee_m=1.5, tilt_deg=10.4,
               n_frames=12, n_loops=16, n_samples=128, frame_period_us=6000,
               trigger_frame=3, amp=400.0, noise=6.0, image_gain=0.0,
               accel_ms2=0.0, seed=0):
    """Raw dump bytes for one synthetic ball flight (+ optional floor image).

    The cube is built in CHIP conventions (the reverse of physical element
    order; TX1 block carries the +Doppler TDM phase) so the pipeline's
    decode path is what's under test. ``accel_ms2`` makes the radial speed
    time-varying (Doppler phase follows the true displacement; the TDM
    phase follows the INSTANTANEOUS velocity).
    """
    rng = np.random.default_rng(seed)
    res = RANGE_SPAN_M / n_samples
    cpf = 2 * n_loops
    cube = np.zeros((n_frames, cpf, 4, n_samples), dtype=complex)
    samples = np.arange(n_samples)
    tan_la = np.tan(np.radians(launch_deg))
    for slot in range(n_frames):
        t_slot = ((slot - trigger_frame) % n_frames) * frame_period_us / 1e6
        for loop in range(n_loops):
            t_s = t_slot + loop * LOOP_PRI_S
            disp = speed_ms * t_s + 0.5 * accel_ms2 * t_s * t_s
            v_inst = speed_ms + accel_ms2 * t_s
            x_m = tee_m + disp
            h_m = tan_la * (x_m - tee_m)          # height above radar plane
            theta = np.arctan2(h_m, x_m) - np.radians(tilt_deg)
            rbin = x_m / res
            if rbin >= n_samples - 2:
                continue
            tone = np.exp(2j * np.pi * rbin * samples / n_samples)
            phys = steer(theta, 8)
            if image_gain:
                theta_img = (np.arctan2(-(h_m + 2 * RADAR_HEIGHT_M), x_m)
                             - np.radians(tilt_deg))
                phys = phys + image_gain * steer(theta_img, 8)
            chip = phys[::-1].copy()              # physical -> chip order
            doppler = np.exp(1j * 4 * np.pi * disp / LAM)
            chip[4:] *= np.exp(1j * 4 * np.pi * v_inst * TAU_S / LAM)
            for rx in range(4):
                cube[slot, 2 * loop, rx] = amp * doppler * chip[rx] * tone
                cube[slot, 2 * loop + 1, rx] = (amp * doppler * chip[4 + rx]
                                                * tone)
    cube += rng.standard_normal(cube.shape) * noise \
        + 1j * rng.standard_normal(cube.shape) * noise
    return pack_dump(cube, n_tx=2, trigger_frame=trigger_frame, version=3,
                     frame_period_us=frame_period_us)


@pytest.fixture(name="cal")
def _cal():
    cal = Calibration.identity()
    cal.tilt_rad = np.radians(10.4)
    cal.tee_range_m = 1.5
    # synthetic flights launch AT the radar plane -> ball height above the
    # floor equals the radar height (anchor h = 0 in radar-plane coords)
    cal.tee_ball_height_m = RADAR_HEIGHT_M
    cal.meta["radar_height_m"] = RADAR_HEIGHT_M
    return cal


def test_header_carries_period_and_trigger():
    raw = synth_shot(frame_period_us=6000, trigger_frame=5)
    meta = parse_header(raw)
    assert meta["frame_period_us"] == 6000
    assert meta["trigger_frame"] == 5
    geo = geometry_from_header(meta)
    assert geo.frame_period_s == pytest.approx(0.006)
    assert geo.n_loops == 16


def test_recovers_speed_and_launch_angle(cal):
    truth_v, truth_la = 45.0, 18.0
    raw = synth_shot(speed_ms=truth_v, launch_deg=truth_la)
    shot = process_dump(raw, cal, coherent_loops=1, two_ray=False)
    assert shot.ball_found
    assert shot.track.speed_ms == pytest.approx(truth_v, rel=0.02)
    assert shot.fits["free"].launch_angle_deg == pytest.approx(truth_la,
                                                               abs=1.5)
    assert shot.fits["tee"].launch_angle_deg == pytest.approx(truth_la,
                                                              abs=1.5)


def test_time_order_uses_header_rotation(cal):
    for trig in (0, 4, 9):
        raw = synth_shot(trigger_frame=trig)
        shot = process_dump(raw, cal, coherent_loops=1, two_ray=False)
        assert shot.ball_found, f"trigger_frame={trig}"
        assert shot.track.speed_ms == pytest.approx(45.0, rel=0.03)


def test_coherent_integration_raises_snr(cal):
    raw = synth_shot(noise=25.0)
    shot_k1 = process_dump(raw, cal, coherent_loops=1, two_ray=False)
    shot_k4 = process_dump(raw, cal, coherent_loops=4, two_ray=False)
    assert shot_k4.ball_found
    # K=4 sums 4 loops coherently: fewer points, each ~4x the energy
    assert 0 < shot_k4.n_angle_points <= shot_k1.n_angle_points
    assert shot_k4.fits["free"].launch_angle_deg == pytest.approx(18.0,
                                                                  abs=2.0)


def test_two_ray_recovers_angle_under_multipath(cal):
    """With a strong floor image the plain fits bend; two-ray should not."""
    raw = synth_shot(image_gain=0.6, noise=4.0)
    shot = process_dump(raw, cal, coherent_loops=2, two_ray=True)
    assert shot.ball_found
    assert "two_ray" in shot.fits
    assert shot.fits["two_ray"].launch_angle_deg == pytest.approx(18.0,
                                                                  abs=2.0)


def test_no_ball_in_empty_scene(cal):
    rng = np.random.default_rng(1)
    cube = (rng.standard_normal((12, 32, 4, 128))
            + 1j * rng.standard_normal((12, 32, 4, 128))) * 8.0
    raw = pack_dump(cube, n_tx=2, version=3, frame_period_us=6000)
    shot = process_dump(raw, Calibration.identity())
    assert not shot.ball_found
    assert shot.summary() == "no ball detected"


def _two_streak_cube(*, slow_ms=24.0, fast_ms=60.0, n_frames=12, n_loops=16,
                     n_samples=128, seed=3):
    """A lingering slow object (whole window) plus a faster, shorter ball."""
    rng = np.random.default_rng(seed)
    res = RANGE_SPAN_M / n_samples
    cube = np.zeros((n_frames, 2 * n_loops, 4, n_samples), dtype=complex)
    samples = np.arange(n_samples)
    for slot in range(n_frames):
        for loop in range(n_loops):
            t_s = slot * 0.006 + loop * LOOP_PRI_S
            movers = [(1.00 + fast_ms * t_s, 400.0)]
            if t_s < 0.045:                # the flying tee lands mid-window
                movers.append((2.30 + slow_ms * t_s, 500.0))
            for x_m, amp in movers:
                rbin = x_m / res
                if rbin >= n_samples - 2:
                    continue
                tone = amp * np.exp(2j * np.pi * rbin * samples / n_samples)
                dopp = np.exp(1j * 4 * np.pi * (x_m - 1.0) / LAM)
                cube[slot, 2 * loop] += dopp * tone
                cube[slot, 2 * loop + 1] += dopp * tone
    cube += (rng.standard_normal(cube.shape)
             + 1j * rng.standard_normal(cube.shape)) * 6.0
    return pack_dump(cube, n_tx=2, version=3, frame_period_us=6000)


def test_fastest_credible_track_beats_slow_theft():
    """A slow lingering object must not steal the track from the ball.

    2026-07-14 live bug: the flying tee (~25 m/s) outlasted the ball in the
    gates and won most-inliers RANSAC on 5 driver + several SW shots.
    """
    from openflight.iwr6843.shot import geometry_from_header as gfh
    from openflight.iwr6843.dump import parse_header
    from openflight.iwr6843.tracking import find_ball, mti_filter
    from openflight.iwr6843.dump import parse_dump
    raw = _two_streak_cube()
    meta, cube = parse_dump(raw)
    geo = gfh(parse_header(raw))
    track = find_ball(mti_filter(cube), geo)
    assert track is not None
    assert track.speed_ms == pytest.approx(60.0, abs=3.0)


def test_local_velocity_on_accelerating_ball(cal):
    """Quadratic refit exposes the instantaneous radial speed."""
    raw = synth_shot(speed_ms=40.0, accel_ms2=60.0, noise=4.0)
    shot = process_dump(raw, cal, two_ray=False)
    assert shot.ball_found and shot.track is not None
    trk = shot.track
    assert trk.quad_bins is not None
    res = RANGE_SPAN_M / 128
    v_first = trk.speed_ms_at(trk.t_first, res)
    v_last = trk.speed_ms_at(trk.t_last, res)
    assert v_first == pytest.approx(40.0 + 60.0 * trk.t_first, abs=2.0)
    assert v_last == pytest.approx(40.0 + 60.0 * trk.t_last, abs=2.0)
    assert v_last - v_first > 2.0


def test_tee_anchor_is_floor_referenced():
    """fit_tee anchors at ball-above-floor minus radar height.

    The old radar-plane anchor sat ~0.15 m high; on a short-lever wedge
    that was a ~-11 deg launch-angle bias.
    """
    from openflight.iwr6843.doa import AnglePoint
    from openflight.iwr6843.trajectory import fit_tee
    cal = Calibration.identity()
    cal.tee_range_m = 1.5
    cal.tee_ball_height_m = 0.04
    cal.meta["radar_height_m"] = 0.152
    truth = np.radians(15.0)
    h_anchor = 0.04 - 0.152
    points = []
    for x in np.linspace(2.4, 4.4, 12):
        h = h_anchor + np.tan(truth) * (x - 1.5)
        points.append(AnglePoint(t_s=0.0, range_m=float(np.hypot(x, h)),
                                 theta_rad=float(np.arctan2(h, x)),
                                 snr=30.0, n_summed=1))
    fit = fit_tee(points, cal)
    assert fit is not None
    assert fit.launch_angle_deg == pytest.approx(15.0, abs=0.1)
    # anchoring at the radar plane instead must visibly rotate the fit
    cal.tee_ball_height_m = 0.04 + 0.152
    fit_old = fit_tee(points, cal)
    assert fit_old.launch_angle_deg < fit.launch_angle_deg - 2.0


def test_two_ray_policy_anchored_gated_multipath(cal):
    """Mid-iron policy (tee anchor + angle gate) under a strong floor image."""
    raw = synth_shot(image_gain=0.6, noise=4.0)
    shot = process_dump(raw, cal, two_ray=True, club="9i")
    assert shot.ball_found
    assert "two_ray" in shot.fits
    assert shot.fits["two_ray"].launch_angle_deg == pytest.approx(18.0,
                                                                  abs=2.0)


def test_cosine_speed_factor_matches_kinematics():
    """Factor must equal (r2-r1)/path from direct kinematics."""
    from openflight.iwr6843.trajectory import cosine_speed_factor
    cal = Calibration.identity()
    cal.tee_range_m = 1.5
    cal.tee_ball_height_m = 0.152          # dz=0: x_tee == slant
    cal.meta["radar_height_m"] = 0.152
    la = np.radians(20.0)
    xs = np.linspace(1.5, 5.0, 400)
    zs = 0.152 + np.tan(la) * (xs - 1.5)
    rs = np.hypot(xs, zs - 0.152)
    r1, r2 = 2.0, 4.0
    i1, i2 = np.searchsorted(rs, r1), np.searchsorted(rs, r2)
    path = (xs[i2] - xs[i1]) / np.cos(la)
    expect = (rs[i2] - rs[i1]) / path
    got = cosine_speed_factor(20.0, r1, r2, cal)
    assert got == pytest.approx(expect, abs=0.005)
    assert got < 0.985                     # meaningfully below 1 at LA 20


def test_thin_capture_rejected(cal):
    """Junk captures (few frames of ball) must be a no-read, not a number."""
    raw = synth_shot(n_frames=3, trigger_frame=0, tee_m=2.3)
    shot = process_dump(raw, cal)
    assert shot.ball_found
    assert shot.quality == "reject"
    assert shot.launch_angle_deg is None
    assert "rejected" in shot.summary()


def test_notch_speed_ball_recovered(cal):
    """A ball at 2 x 26.93 m/s is invisible to burst-MTI (loop phase ~ 0
    mod 2pi); the window-scope retry must recover track AND angle."""
    from openflight.iwr6843.tracking import find_ball, mti_filter
    from openflight.iwr6843.dump import parse_dump
    notch_v = 2 * 26.93
    raw = synth_shot(speed_ms=notch_v, noise=4.0)
    meta, cube = parse_dump(raw)
    geo = geometry_from_header(meta)
    burst_track = find_ball(mti_filter(cube), geo)
    shot = process_dump(raw, cal)
    assert shot.ball_found and shot.quality != "reject"
    assert shot.notch_recovered
    if burst_track is not None:           # burst track is visibly degraded
        assert shot.track.rms_bins < burst_track.rms_bins
    assert shot.track.speed_ms == pytest.approx(notch_v, rel=0.03)
    assert shot.policy == "far"           # notch-near -> no-anchor config
    assert shot.launch_angle_deg == pytest.approx(18.0, abs=2.5)


def test_policy_is_observable_keyed(cal):
    """Same club label, different speeds -> different policy."""
    slow = process_dump(synth_shot(speed_ms=45.0), cal, club="7Iron")
    fast = process_dump(synth_shot(speed_ms=60.0), cal, club="7Iron")
    assert slow.policy.startswith("anchored")
    assert fast.policy == "far"


def test_club_class_mapping():
    from openflight.iwr6843.shot import club_class
    assert club_class("SandWedge") == "wedge"
    assert club_class("PW") == "wedge"
    assert club_class("9i") == "mid_iron"
    assert club_class("7Iron") == "mid_iron"
    assert club_class("5Iron") == "long_iron"
    assert club_class("3 hybrid") == "long_iron"
    assert club_class("Driver") == "driver"
    assert club_class(None) == "default"
    assert club_class("putter") == "default"
