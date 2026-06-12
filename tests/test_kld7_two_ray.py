"""Tests for the two-ray multipath demodulation vertical estimator.

The synthetic payloads are physically exact: each sample carries the
two-way path phase of the ball AND its floor image computed from the
trajectory geometry, so Doppler aliasing, the multipath fringe, the
intra-frame phase drift, and the F1B range phase all emerge from the
physics rather than being injected as separate tones.
"""

import math

import numpy as np
import pytest

from openflight.kld7.radc import WAVELENGTH_M, extract_launch_angle
from openflight.kld7.two_ray import (
    ACQ_MS,
    MPH_TO_FTS,
    SAMPLE_DT_MS,
    SAMPLES,
    estimate_two_ray,
    two_ray_fit,
)

FT_TO_M = 0.3048
G_FT_S2 = 32.17

MOUNT_DEG = 10.0
OFFSET_DEG = 2.5
DISTANCE_FT = 5.0
BALL_ABOVE_RADAR_FT = -4.0 / 12.0
RANGE_UNAMB_FT = 5.0 * 3.28084  # 5 m FSK setting


def _trajectory(t_s: np.ndarray, la_deg: float, speed_mph: float):
    v = speed_mph * MPH_TO_FTS
    la = math.radians(la_deg)
    x = DISTANCE_FT + v * math.cos(la) * t_s
    y = BALL_ABOVE_RADAR_FT + v * math.sin(la) * t_s - 0.5 * G_FT_S2 * t_s**2
    return x, y


def make_two_ray_payload(
    frame_t_ms: float,
    la_deg: float,
    speed_mph: float,
    rho: float = 0.45,
    amp: float = 3000.0,
    noise: float = 25.0,
    seed: int = 0,
) -> bytes:
    """RADC payload for a frame ENDING frame_t_ms after impact."""
    rng = np.random.default_rng(seed)
    n = np.arange(SAMPLES)
    t_ms = frame_t_ms - (SAMPLES - n) * SAMPLE_DT_MS
    t_s = t_ms / 1000.0

    x, y = _trajectory(t_s, la_deg, speed_mph)
    y_img = 2.0 * BALL_ABOVE_RADAR_FT - y
    r_b = np.hypot(x, y) * FT_TO_M
    r_i = np.hypot(x, y_img) * FT_TO_M

    boresight = math.radians(MOUNT_DEG + OFFSET_DEG)
    spacing = 8.0e-3  # effective Rx spacing used by the angle code
    th_b = np.arctan2(y, x) - boresight
    th_i = np.arctan2(y_img, x) - boresight
    delta_b = 2.0 * math.pi * spacing / WAVELENGTH_M * np.sin(th_b)
    delta_i = 2.0 * math.pi * spacing / WAVELENGTH_M * np.sin(th_i)

    # Two-way carrier phase; K-LD7 convention: receding = positive Doppler
    phi_b = 4.0 * math.pi * r_b / WAVELENGTH_M
    phi_i = 4.0 * math.pi * r_i / WAVELENGTH_M
    # F1B leads F1A by the FSK range phase 2*pi*r/R_unambiguous
    psi_b = 2.0 * math.pi * (r_b / FT_TO_M) / RANGE_UNAMB_FT
    psi_i = 2.0 * math.pi * (r_i / FT_TO_M) / RANGE_UNAMB_FT

    a, b = amp, amp * rho
    s1 = a * np.exp(1j * phi_b) + b * np.exp(1j * phi_i)
    s2 = a * np.exp(1j * (phi_b - delta_b)) + b * np.exp(1j * (phi_i - delta_i))
    s1b = a * np.exp(1j * (phi_b + psi_b)) + b * np.exp(1j * (phi_i + psi_i))

    payload = np.empty(0, dtype=np.uint16)
    for sig in (s1, s2, s1b):
        i_ch = 2048 + sig.real + rng.normal(0, noise, SAMPLES)
        q_ch = 2048 + sig.imag + rng.normal(0, noise, SAMPLES)
        payload = np.concatenate(
            [
                payload,
                np.clip(i_ch, 0, 4095).astype(np.uint16),
                np.clip(q_ch, 0, 4095).astype(np.uint16),
            ]
        )
    return payload.tobytes()


def make_quiet_payload(seed: int = 99) -> bytes:
    rng = np.random.default_rng(seed)
    samples = np.clip(2048 + rng.normal(0, 25.0, SAMPLES * 6), 0, 4095)
    return samples.astype(np.uint16).tobytes()


def _frames_for_shot(la_deg: float, speed_mph: float, impact_ts: float) -> list[dict]:
    frames = [
        {"timestamp": impact_ts - 0.40 + 0.0287 * i, "radc": make_quiet_payload(seed=i)}
        for i in range(4)
    ]
    # rho=0.25 keeps the synthetic inside the validated fringe regime: the
    # exact mirror geometry synthesized here produces a faster fringe than
    # observed indoors (chi_dot ~0.7 vs 0.1-0.5 rad/ms measured), and at
    # high rho that stresses the decomposition beyond what real sessions
    # showed (see docs/kld7-subframe-stft-findings.md).
    for k, t_ms in enumerate((30.0, 55.0, 75.0)):
        frames.append(
            {
                "timestamp": impact_ts + t_ms / 1000.0,
                "radc": make_two_ray_payload(t_ms, la_deg, speed_mph, rho=0.25, seed=k),
            }
        )
    return frames


class TestTwoRayFit:
    def test_recovers_ball_phase_and_rho(self):
        delta_b, delta_i, rho, chi_dot = 0.9, -0.5, 0.6, 0.7
        t = np.linspace(-12.0, 12.0, 13)
        g = rho * np.exp(1j * 0.8)
        u, v = np.exp(-1j * delta_b), np.exp(-1j * delta_i)
        z = (u + g * np.exp(1j * chi_dot * t) * v) / (1.0 + g * np.exp(1j * chi_dot * t))
        fit = two_ray_fit(z, t, np.ones_like(t))
        assert fit is not None
        assert fit["resid"] < 1e-6
        # The model is symmetric under swapping the two components (with
        # chi -> -chi, rho -> 1/rho); disambiguation happens downstream.
        direct = abs(fit["ball_phase"] - delta_b) < 0.02 and abs(fit["rho"] - rho) < 0.02
        swapped = abs(fit["ball_phase"] - delta_i) < 0.02 and abs(fit["rho"] - 1.0 / rho) < 0.05
        assert direct or swapped, fit
        assert abs(abs(fit["chi_dot"]) - chi_dot) < 0.05

    def test_too_few_points_returns_none(self):
        t = np.linspace(-5, 5, 5)
        assert two_ray_fit(np.ones(5, dtype=complex), t, np.ones(5)) is None


class TestEstimateTwoRay:
    def test_recovers_launch_angle(self):
        la_true, speed, impact_ts = 18.0, 100.0, 1000.0
        est = estimate_two_ray(
            _frames_for_shot(la_true, speed, impact_ts),
            impact_ts,
            speed,
            MOUNT_DEG,
            OFFSET_DEG,
            DISTANCE_FT,
            BALL_ABOVE_RADAR_FT,
        )
        assert est.refusal_reason is None, est.diagnostics
        assert est.launch_angle_deg == pytest.approx(la_true, abs=2.5)
        assert est.confidence >= 0.68
        assert abs(est.diagnostics["tau_range_ms"]) < 15.0

    def test_dc_blind_zone_refused(self):
        # 128 mph keeps every frame's RADIAL alias inside the +/-4 km/h
        # clutter core (the gate is per-frame radial now, not per-shot
        # impact speed: 124 or even 130 mph shots have escape frames
        # and measure; only the narrow true core refuses)
        est = estimate_two_ray(
            _frames_for_shot(18.0, 128.0, 1000.0),
            1000.0,
            128.0,
            MOUNT_DEG,
            OFFSET_DEG,
            DISTANCE_FT,
            BALL_ABOVE_RADAR_FT,
        )
        assert est.launch_angle_deg is None
        assert est.refusal_reason == "dc_blind_zone"
        assert est.diagnostics["refusal_reason"] == "dc_blind_zone"
        assert est.diagnostics["n_frames_dc_core_skipped"] >= 3

    def test_no_impact_timestamp_refused(self):
        est = estimate_two_ray(
            [], None, 100.0, MOUNT_DEG, OFFSET_DEG, DISTANCE_FT, BALL_ABOVE_RADAR_FT
        )
        assert est.refusal_reason == "no_impact_timestamp"

    def test_no_frames_refused(self):
        est = estimate_two_ray(
            [], 1000.0, 100.0, MOUNT_DEG, OFFSET_DEG, DISTANCE_FT, BALL_ABOVE_RADAR_FT
        )
        assert est.launch_angle_deg is None
        assert est.refusal_reason == "no_range_track"


class TestExtractLaunchAngleTwoRay:
    def test_end_to_end_two_ray_estimator(self):
        la_true, speed, impact_ts = 18.0, 100.0, 1000.0
        results = extract_launch_angle(
            _frames_for_shot(la_true, speed, impact_ts),
            ops243_ball_speed_mph=speed,
            angle_offset_deg=OFFSET_DEG,
            orientation="vertical",
            vertical_estimator="two_ray",
            shot_timestamp=impact_ts,
            impact_timestamp=impact_ts,
            mount_deg=MOUNT_DEG,
            distance_ft=DISTANCE_FT,
            ball_above_radar_ft=BALL_ABOVE_RADAR_FT,
        )
        assert results, "expected at least one shot result"
        best = results[0]
        assert best["estimator"] == "two_ray"
        assert best["selection_path"] == "two_ray"
        assert best["launch_angle_deg"] == pytest.approx(la_true, abs=2.5)
        assert best["confidence"] >= 0.68
        assert best["two_ray"]["la_curve_deg"] == pytest.approx(la_true, abs=2.5)

    def test_refusal_falls_back_to_geometry_path(self):
        # Blind-zone ball speed: two_ray must refuse and the pipeline must
        # still return via the geometry/naive fallback without crashing.
        la_true, speed, impact_ts = 18.0, 128.0, 1000.0
        results = extract_launch_angle(
            _frames_for_shot(la_true, speed, impact_ts),
            ops243_ball_speed_mph=speed,
            angle_offset_deg=OFFSET_DEG,
            orientation="vertical",
            vertical_estimator="two_ray",
            shot_timestamp=impact_ts,
            impact_timestamp=impact_ts,
            mount_deg=MOUNT_DEG,
            distance_ft=DISTANCE_FT,
            ball_above_radar_ft=BALL_ABOVE_RADAR_FT,
        )
        for res in results:
            assert res["estimator"] != "two_ray"
            assert res["two_ray"]["refusal_reason"] == "dc_blind_zone"


class TestFrameTiming:
    def test_acquisition_constants_consistent(self):
        # 256 samples must span one ~28.7 ms frame
        assert ACQ_MS == pytest.approx(SAMPLES * SAMPLE_DT_MS)
        assert 28.0 < ACQ_MS < 29.5
