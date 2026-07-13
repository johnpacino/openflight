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
               seed=0):
    """Raw dump bytes for one synthetic ball flight (+ optional floor image).

    The cube is built in CHIP conventions (the reverse of physical element
    order; TX1 block carries the +Doppler TDM phase) so the pipeline's
    decode path is what's under test.
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
            x_m = tee_m + speed_ms * t_s
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
            doppler = np.exp(1j * 4 * np.pi * speed_ms * t_s / LAM)
            chip[4:] *= np.exp(1j * 4 * np.pi * speed_ms * TAU_S / LAM)
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
    cal.tee_height_m = 0.0
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
