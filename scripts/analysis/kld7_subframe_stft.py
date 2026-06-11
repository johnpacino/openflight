#!/usr/bin/env python3
"""Sub-frame STFT analysis of K-LD7 RADC frames (multipath fringe probe).

Each K-LD7 RADC frame is a ~28.6 ms acquisition (256 I/Q samples) during
which the ball climbs far enough to sweep several ground-multipath fringes
and several degrees of elevation. The production pipeline collapses the
whole acquisition into a single FFT, averaging across that structure.

This prototype splits each frame into overlapping sub-frame windows and
extracts a bearing per sub-frame, giving:

- a bearing-vs-time micro-track *within* each frame
- a detrended phase-ripple metric (the multipath fringe signature)
- an inter-channel coherence metric (blended-return detector)

Results are scored against TrackMan truth: the expected elevation-vs-time
curve is computed from TM ball speed + launch angle and the known
radar/tee geometry.

Usage:
    uv run python scripts/analysis/kld7_subframe_stft.py \
        --session ~/openflight_sessions/trackman-6-8/session_20260608_103725_trackman_8i.jsonl \
        --frames-csv ~/openflight_sessions/trackman-6-8/session_20260608_103725_trackman_8i_kld7_geometry_report/frames_live.csv \
        --compare-csv ~/openflight_sessions/trackman-6-8/compare_8i.csv \
        --out ~/openflight_sessions/trackman-6-8/subframe_stft_report
"""

from __future__ import annotations

import argparse
import base64
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))

from openflight.kld7.radc import (  # noqa: E402
    ANTENNA_SPACING_M,
    WAVELENGTH_M,
    parse_radc_payload,
    to_complex_iq,
)

MAX_SPEED_KMH = 100.0
FULL_FFT_SIZE = 2048
SAMPLES = 256
G_FT_S2 = 32.17
MPH_TO_FPS = 1.4666667
M_TO_FT = 3.28084
RANGE_SETTING_M = 5.0  # K-LD7 RRAI=0 -> 5 m unambiguous FSK range

# Acquisition timing: complex sample rate must cover +/-100 km/h Doppler.
_FD_MAX_HZ = 2.0 * (MAX_SPEED_KMH / 3.6) / WAVELENGTH_M
SAMPLE_DT_MS = 1000.0 / (2.0 * _FD_MAX_HZ)
ACQ_MS = SAMPLES * SAMPLE_DT_MS


@dataclass
class Geometry:
    ball_distance_ft: float = 5.0
    ball_above_radar_ft: float = -4.0 / 12.0
    mount_deg: float = 10.0
    angle_offset_deg: float = 2.5
    screen_from_ball_ft: float = 12.0


@dataclass
class SubFrame:
    start: int
    t_ms: float  # absolute time vs impact (nominal, before tau shift)
    peak_bin: int
    snr_db: float
    balance: float  # |F2A| / |F1A| at peak
    raw_angle_deg: float
    elevation_deg: float
    phase_rad: float
    magnitude: float
    good: bool
    range_ft: float = float("nan")
    f1b_snr: float = 0.0


@dataclass
class FrameResult:
    frame_index: int
    t_ms: float  # frame timestamp vs impact (end of acquisition)
    t_center_ms: float  # mid-acquisition time
    expected_bin: int
    full_peak_bin: int = 0
    full_snr_db: float = 0.0
    full_elevation_deg: float = 0.0
    live_elevation_deg: float | None = None
    live_status: str = ""
    subframes: list[SubFrame] = field(default_factory=list)
    ripple_el_rms_deg: float = 0.0  # detrended phase ripple, elevation degrees
    coherence: float = 0.0  # detrended inter-channel coherence [0..1]
    sweep_el_deg: float = 0.0  # fitted elevation sweep across acquisition
    n_good: int = 0
    # Two-ray demodulation outputs (per frame, fixed bin)
    el_2ray_deg: float = float("nan")  # demodulated ball elevation
    el_2ray_image_deg: float = float("nan")  # demodulated image elevation
    rho_2ray: float = float("nan")  # image/ball amplitude ratio
    chidot_2ray: float = float("nan")  # fringe rate (rad/ms)
    resid_2ray: float = float("nan")  # normalized fit residual
    umod_2ray: float = float("nan")  # |ball phasor| (1.0 if model exact)
    range_med_ft: float = float("nan")  # median F1B range over gated subs
    live_f1b_range_ft: float | None = None  # full-frame F1B range (live report)
    live_f1b_snr: float | None = None
    range_fixedbin_ft: float = float("nan")  # full-frame F1B range at demod bin
    f1b_fixedbin_snr: float = float("nan")


def hann_fft(iq: np.ndarray, fft_size: int) -> np.ndarray:
    windowed = (iq - np.mean(iq)) * np.hanning(len(iq))
    padded = np.zeros(fft_size, dtype=np.complex128)
    padded[: len(windowed)] = windowed
    return np.fft.fft(padded)


def circular_window(center: int, half: int, size: int) -> np.ndarray:
    return (np.arange(center - half, center + half + 1)) % size


def find_peak_near(mag: np.ndarray, expected: int, half_window: int) -> int:
    idx = circular_window(expected, half_window, len(mag))
    return int(idx[np.argmax(mag[idx])])


def noise_floor(mag: np.ndarray, peak: int, peak_guard: int, dc_guard: int) -> float:
    mask = np.ones(len(mag), dtype=bool)
    mask[circular_window(peak, peak_guard, len(mag))] = False
    mask[:dc_guard] = False
    mask[-dc_guard:] = False
    vals = mag[mask]
    return float(np.median(vals)) if len(vals) else 1.0


def raw_angle_from_phase(phase_rad: float) -> float:
    sin_theta = phase_rad * WAVELENGTH_M / (2.0 * math.pi * ANTENNA_SPACING_M)
    return math.degrees(math.asin(max(-1.0, min(1.0, sin_theta))))


def two_ray_fit(
    z: np.ndarray,
    t_rel_ms: np.ndarray,
    w: np.ndarray,
    alpha: float = 0.0,
    beta: float = 0.0,
) -> dict | None:
    """Demodulate the two-ray interference from the sub-frame phasor ratios.

    Model: z(t) = (u(t) + g e^{j chi_dot t} v(t)) / (1 + g e^{j chi_dot t})
    where u = e^{-j delta_ball}, v = e^{-j delta_image}, g = rho e^{j chi0}.

    alpha/beta are FIXED intra-frame phase-drift rates (rad/ms) for the
    ball and image components — computed from known geometry (ball speed,
    range, nominal trajectory), NOT fitted. A free 3-rate fit was tested
    and overfits badly (9 params vs 13 noisy points); with the rates fixed
    the model keeps its original degrees of freedom:

        z = u0 e^{j alpha t} + (g v0) e^{j (chi_dot+beta) t}
              - g e^{j chi_dot t} z

    — LINEAR in (u0, p = g v0, q = g) for fixed chi_dot: 1D grid +
    closed-form batched weighted least squares.

    Returns dict with ball/image phases, rho, chi_dot, residual — or None.
    u0 is the ball phasor AT THE FRAME CENTER (t_rel is center-relative).
    """
    if len(z) < 9:
        return None
    sw = np.sqrt(w)
    zw = z * sw
    z_norm = max(float(np.sum(np.abs(zw) ** 2)), 1e-12)

    def solve_grid(
        chi_arr: np.ndarray, alpha_arr: np.ndarray, beta_arr: np.ndarray
    ) -> tuple | None:
        mesh = np.meshgrid(chi_arr, alpha_arr, beta_arr, indexing="ij")
        cd = mesh[0].ravel()
        al = mesh[1].ravel()
        be = mesh[2].ravel()
        c1 = np.exp(1j * np.outer(al, t_rel_ms))
        c2 = np.exp(1j * np.outer(cd + be, t_rel_ms))
        c3 = -np.exp(1j * np.outer(cd, t_rel_ms)) * z[None, :]
        a_mat = np.stack([c1, c2, c3], axis=-1) * sw[None, :, None]  # K,N,3
        ah = a_mat.conj().transpose(0, 2, 1)
        gram = ah @ a_mat
        ridge = 1e-8 * np.trace(gram, axis1=1, axis2=2).real[:, None, None] * np.eye(3)[None]
        rhs = np.einsum("kcn,n->kc", ah, zw)
        try:
            x = np.linalg.solve(gram + ridge, rhs[..., None])[..., 0]
        except np.linalg.LinAlgError:
            return None
        resid = (
            np.sum(np.abs(np.einsum("knc,kc->kn", a_mat, x) - zw[None, :]) ** 2, axis=1) / z_norm
        )
        k = int(np.argmin(resid))
        return float(resid[k]), float(cd[k]), float(al[k]), float(be[k]), x[k]

    chi = np.concatenate([np.arange(-2.5, -0.12, 0.04), np.arange(0.12, 2.5, 0.04)])
    best = solve_grid(chi, np.array([alpha]), np.array([beta]))
    if best is None:
        return None

    resid, chi_dot, alpha, beta, (u, p, q) = best
    rho = abs(q)
    v = p / q if rho > 1e-6 else complex(0.0)
    return {
        "ball_phase": -float(np.angle(u)),
        "image_phase": -float(np.angle(v)) if rho > 1e-6 else float("nan"),
        "rho": rho,
        "chi_dot": chi_dot,
        "resid": resid,
        "u_mod": abs(u),
        "v_mod": abs(v),
        "alpha": alpha,
        "beta": beta,
    }


def predicted_drift_rates(
    r_ft: float, speed_mph: float, geo: Geometry, nominal_la_deg: float
) -> tuple[float, float]:
    """Predicted intra-frame inter-channel phase drift (rad/ms), ball and image.

    Deterministic from geometry: invert measured range to a nominal
    trajectory point, compute the elevation rates of the ball and its
    floor mirror image there, and convert via d(delta)/dt =
    (2*pi*d/lambda) * cos(theta_boresight) * d(theta)/dt. The model's
    convention u = e^{-j delta} gives alpha = -d(delta)/dt.
    """
    v = speed_mph * MPH_TO_FPS / 1000.0  # ft/ms
    la = math.radians(nominal_la_deg)
    t_hat = max((r_ft - geo.ball_distance_ft) / v, 0.0)
    x = geo.ball_distance_ft + v * math.cos(la) * t_hat
    y = geo.ball_above_radar_ft + v * math.sin(la) * t_hat
    vx, vy = v * math.cos(la), v * math.sin(la)
    el_rate_ball = (vy * x - vx * y) / (x * x + y * y)  # rad/ms
    # floor mirror: ball starts at floor level => mirror plane at tee height
    y_img = 2.0 * geo.ball_above_radar_ft - y
    el_rate_img = (-vy * x - vx * y_img) / (x * x + y_img * y_img)
    k = 2.0 * math.pi * ANTENNA_SPACING_M / WAVELENGTH_M
    boresight = math.radians(geo.mount_deg + geo.angle_offset_deg)
    th_b = math.atan2(y, x) - boresight
    th_i = math.atan2(y_img, x) - boresight
    alpha = -k * math.cos(th_b) * el_rate_ball
    beta = -k * math.cos(th_i) * el_rate_img
    return alpha, beta


def elevation_truth_deg(t_ms: float, la_deg: float, speed_mph: float, geo: Geometry) -> float:
    """Elevation of the ball as seen from the radar, t_ms after impact."""
    t = t_ms / 1000.0
    v = speed_mph * MPH_TO_FPS
    la = math.radians(la_deg)
    x = geo.ball_distance_ft + v * math.cos(la) * t
    y = geo.ball_above_radar_ft + v * math.sin(la) * t - 0.5 * G_FT_S2 * t * t
    return math.degrees(math.atan2(y, x))


def screen_time_ms(la_deg: float, speed_mph: float, geo: Geometry) -> float:
    v = speed_mph * MPH_TO_FPS
    return 1000.0 * geo.screen_from_ball_ft / (v * math.cos(math.radians(la_deg)))


def analyze_frame(
    payload: bytes,
    frame_t_ms: float,
    expected_bin: int,
    geo: Geometry,
    window: int,
    step: int,
    sub_fft: int,
    snr_gate_db: float,
    balance_gate: float,
    drift: bool = False,
    ball_speed_mph: float | None = None,
    nominal_la_deg: float = 19.0,
) -> FrameResult:
    parsed = parse_radc_payload(payload)
    f1a = to_complex_iq(parsed["f1a_i"], parsed["f1a_q"])
    f2a = to_complex_iq(parsed["f2a_i"], parsed["f2a_q"])
    f1b = to_complex_iq(parsed["f1b_i"], parsed["f1b_q"])

    result = FrameResult(
        frame_index=-1,
        t_ms=frame_t_ms,
        t_center_ms=frame_t_ms - ACQ_MS / 2.0,
        expected_bin=expected_bin,
    )

    # Full-frame reference (matches production: peak on |F1A| near OPS bin)
    fft1_full = hann_fft(f1a, FULL_FFT_SIZE)
    fft2_full = hann_fft(f2a, FULL_FFT_SIZE)
    mag1_full = np.abs(fft1_full)
    peak_full = find_peak_near(mag1_full, expected_bin, half_window=25)
    floor_full = noise_floor(np.abs(fft1_full), peak_full, peak_guard=50, dc_guard=150)
    result.full_peak_bin = peak_full
    result.full_snr_db = 20.0 * math.log10(max(mag1_full[peak_full] / floor_full, 1e-9))
    phase_full = float(np.angle(fft1_full[peak_full] * np.conj(fft2_full[peak_full])))
    result.full_elevation_deg = (
        raw_angle_from_phase(phase_full) + geo.angle_offset_deg + geo.mount_deg
    )

    # Sub-frame STFT
    expected_sub = int(round(expected_bin * sub_fft / FULL_FFT_SIZE)) % sub_fft
    bin_scale = FULL_FFT_SIZE / sub_fft
    search_half = max(4, int(round(25 / bin_scale)) + 4)

    sub_ffts: list[tuple[np.ndarray, np.ndarray]] = []
    for start in range(0, SAMPLES - window + 1, step):
        s1 = hann_fft(f1a[start : start + window], sub_fft)
        s2 = hann_fft(f2a[start : start + window], sub_fft)
        s1b = hann_fft(f1b[start : start + window], sub_fft)
        sub_ffts.append((s1, s2))
        mag = np.sqrt(np.abs(s1) * np.abs(s2))
        peak = find_peak_near(mag, expected_sub, search_half)
        floor = noise_floor(mag, peak, peak_guard=int(16 * 64 / window), dc_guard=20)
        snr_db = 20.0 * math.log10(max(mag[peak] / floor, 1e-9))
        m1, m2 = abs(s1[peak]), abs(s2[peak])
        balance = m2 / m1 if m1 > 0 else 0.0
        phase = float(np.angle(s1[peak] * np.conj(s2[peak])))
        raw_angle = raw_angle_from_phase(phase)
        center_sample = start + window / 2.0
        t_sub = frame_t_ms - (SAMPLES - center_sample) * SAMPLE_DT_MS
        good = snr_db >= snr_gate_db and (1.0 / balance_gate <= balance <= balance_gate)
        # FSK range: F1B-vs-F1A phase at the peak (repo convention, radc.py)
        nb = circular_window(peak, 1, sub_fft)
        cross_b = np.sum(mag[nb] * s1b[nb] * np.conj(s1[nb]))
        phase_b = float(np.angle(cross_b))
        range_ft = (phase_b % (2.0 * math.pi)) / (2.0 * math.pi) * RANGE_SETTING_M * M_TO_FT
        f1b_mag = np.abs(s1b)
        f1b_floor = float(np.median(f1b_mag[f1b_mag > 0])) or 1.0
        f1b_snr = float(f1b_mag[peak] / f1b_floor)
        result.subframes.append(
            SubFrame(
                start=start,
                t_ms=t_sub,
                peak_bin=peak,
                snr_db=snr_db,
                balance=balance,
                raw_angle_deg=raw_angle,
                elevation_deg=raw_angle + geo.angle_offset_deg + geo.mount_deg,
                phase_rad=phase,
                magnitude=float(mag[peak]),
                good=good,
                range_ft=range_ft,
                f1b_snr=f1b_snr,
            )
        )

    good_subs = [s for s in result.subframes if s.good]
    result.n_good = len(good_subs)

    # Detrended phase ripple + coherence over good sub-frames
    if len(good_subs) >= 4:
        t = np.array([s.t_ms for s in good_subs])
        phases = np.unwrap(np.array([s.phase_rad for s in good_subs]))
        weights = np.array([s.magnitude for s in good_subs])
        coeffs = np.polyfit(t, phases, 1, w=weights)
        resid = phases - np.polyval(coeffs, t)
        mean_raw = math.radians(float(np.mean([s.raw_angle_deg for s in good_subs])))
        dtheta_dphi = WAVELENGTH_M / (
            2.0 * math.pi * ANTENNA_SPACING_M * max(math.cos(mean_raw), 0.2)
        )
        result.ripple_el_rms_deg = math.degrees(
            float(np.sqrt(np.average(resid**2, weights=weights))) * dtheta_dphi
        )
        result.coherence = float(np.abs(np.average(np.exp(1j * resid), weights=weights)))
        # Elevation sweep over the full acquisition implied by the phase slope
        sweep_phase = coeffs[0] * ACQ_MS
        result.sweep_el_deg = math.degrees(sweep_phase * dtheta_dphi)

    # Median F1B range over quality-gated subs (multipath-immune observable)
    gated_ranges = [s.range_ft for s in result.subframes if s.good and s.f1b_snr >= 1.5]
    if len(gated_ranges) >= 3:
        result.range_med_ft = float(np.median(gated_ranges))

    # Two-ray demodulation at a fixed bin (per-sub peak hopping breaks the
    # model, so extract every sub-frame at the same Doppler bin)
    good_peaks = [s.peak_bin for s in good_subs]
    fixed_bin = int(np.median(good_peaks)) if len(good_peaks) >= 3 else expected_sub
    s1_vals = np.array([fts[0][fixed_bin] for fts in sub_ffts])
    s2_vals = np.array([fts[1][fixed_bin] for fts in sub_ffts])
    ok = np.abs(s1_vals) > 1e-9
    if int(np.sum(ok)) >= 9:
        z = s2_vals[ok] / s1_vals[ok]
        w = np.abs(s1_vals[ok] * s2_vals[ok])
        w = w / max(float(np.max(w)), 1e-12)
        t_rel = np.array([s.t_ms for s in result.subframes])[ok] - result.t_center_ms
        # Full-frame F1B range AT THE DEMOD'S BIN (computed first: the
        # drift compensation needs it to locate the ball on its trajectory)
        fft1b_full = hann_fft(f1b, FULL_FFT_SIZE)
        fb = fixed_bin * (FULL_FFT_SIZE // sub_fft)
        nb_full = circular_window(fb, 4, FULL_FFT_SIZE)
        wts = np.abs(fft1_full[nb_full])
        cross_full = np.sum(wts * fft1b_full[nb_full] * np.conj(fft1_full[nb_full]))
        phase_full_b = float(np.angle(cross_full))
        result.range_fixedbin_ft = (
            (phase_full_b % (2.0 * math.pi)) / (2.0 * math.pi) * RANGE_SETTING_M * M_TO_FT
        )
        f1b_full_mag = np.abs(fft1b_full)
        positive = f1b_full_mag[f1b_full_mag > 0]
        floor_b = float(np.median(positive)) if positive.size else 1.0
        result.f1b_fixedbin_snr = float(f1b_full_mag[fb] / floor_b)

        fit = two_ray_fit(z, t_rel, w)
        if drift and fit is not None and ball_speed_mph:
            # Physics-fixed intra-frame drift compensation (second pass).
            # Needs a plausible range to locate the ball on its trajectory.
            r_drift = float("nan")
            if 4.5 <= result.range_fixedbin_ft <= 16.0 and result.f1b_fixedbin_snr >= 2.0:
                r_drift = result.range_fixedbin_ft
            elif not math.isnan(result.range_med_ft) and 4.5 <= result.range_med_ft <= 16.0:
                r_drift = result.range_med_ft
            if not math.isnan(r_drift):
                alpha, beta = predicted_drift_rates(r_drift, ball_speed_mph, geo, nominal_la_deg)
                fit2 = two_ray_fit(z, t_rel, w, alpha=alpha, beta=beta)
                if fit2 is not None:
                    fit = fit2
        if fit is not None:
            el_u = raw_angle_from_phase(fit["ball_phase"]) + geo.angle_offset_deg + geo.mount_deg
            el_v = (
                raw_angle_from_phase(fit["image_phase"]) + geo.angle_offset_deg + geo.mount_deg
                if not math.isnan(fit["image_phase"])
                else float("nan")
            )
            # The model is symmetric under swapping the two components; the
            # physical image is always BELOW the ball, so the higher of the
            # two well-formed components is the ball.
            ball_el, image_el, rho = el_u, el_v, fit["rho"]
            if (
                not math.isnan(el_v)
                and 0.25 <= fit["rho"] <= 4.0
                and 0.5 <= fit["v_mod"] <= 1.5
                and el_v > el_u
            ):
                ball_el, image_el, rho = el_v, el_u, 1.0 / fit["rho"]
            result.el_2ray_deg = ball_el
            result.el_2ray_image_deg = image_el
            result.rho_2ray = rho
            result.chidot_2ray = fit["chi_dot"]
            result.resid_2ray = fit["resid"]
            result.umod_2ray = fit["u_mod"]

    return result


def fit_tau_ms(
    frames: list[FrameResult],
    la_deg: float,
    speed_mph: float,
    geo: Geometry,
    tau_range: float = 80.0,
    tau_step: float = 0.5,
) -> tuple[float, float]:
    """Per-shot acquisition-time offset: shift sub-frame times to best match truth.

    Returns (tau_ms, median_abs_err_deg at best tau). Robust median objective.
    """
    t_screen = screen_time_ms(la_deg, speed_mph, geo)
    obs_t, obs_el = [], []
    for fr in frames:
        for s in fr.subframes:
            if s.good:
                obs_t.append(s.t_ms)
                obs_el.append(s.elevation_deg)
    if len(obs_t) < 5:
        return 0.0, float("nan")
    obs_t_arr = np.array(obs_t)
    obs_el_arr = np.array(obs_el)
    best_tau, best_err = 0.0, float("inf")
    for tau in np.arange(-tau_range, tau_range + tau_step, tau_step):
        shifted = obs_t_arr + tau
        in_flight = (shifted >= 3.0) & (shifted <= t_screen)
        if int(np.sum(in_flight)) < 5:
            continue
        truth = np.array(
            [elevation_truth_deg(t, la_deg, speed_mph, geo) for t in shifted[in_flight]]
        )
        err = float(np.median(np.abs(obs_el_arr[in_flight] - truth)))
        if err < best_err:
            best_err, best_tau = err, float(tau)
    return best_tau, best_err


def range_anchored_tau(
    times_ms: np.ndarray,
    ranges_ft: np.ndarray,
    speed_mph: float,
    geo: Geometry,
) -> tuple[float, float]:
    """Timing from the multipath-immune F1B range progression.

    The A-vs-B FSK phase sees near-identical multipath on both frequencies,
    so range survives fringes that corrupt bearing. Range = ball_distance at
    impact anchors absolute time with no clock involved.

    Slope is FIXED to the known ball speed; only the intercept is fitted
    (median, robust). Returns (tau_ms, mad_ft residual).
    """
    v_ft_ms = speed_mph * MPH_TO_FPS / 1000.0
    if len(times_ms) < 4:
        return float("nan"), float("nan")
    intercepts = ranges_ft - v_ft_ms * times_ms
    c = float(np.median(intercepts))
    resid = np.abs(intercepts - c)
    mad = float(np.median(resid))
    # refit on inliers
    keep = resid <= max(3.0 * mad, 0.5)
    if int(np.sum(keep)) >= 4:
        c = float(np.median(intercepts[keep]))
        mad = float(np.median(np.abs(intercepts[keep] - c)))
    tau = (c - geo.ball_distance_ft) / v_ft_ms
    return tau, mad


def position_fit_la(
    ranges_ft: np.ndarray,
    elevations_deg: np.ndarray,
    weights: np.ndarray,
    geo: Geometry,
    trim_iters: int = 2,
    trim_frac: float = 0.3,
) -> tuple[float, int]:
    """Timing-free launch angle: tee-anchored line through (range, elevation) positions.

    The line is constrained through the tee, so the slope IS the launch
    direction and no clock is involved. Iteratively trims the worst
    residuals to reject the multipath ghost population.

    Returns (launch_angle_deg, n_points_used).
    """
    el = np.radians(elevations_deg)
    xs = ranges_ft * np.cos(el)
    ys = ranges_ft * np.sin(el)
    dx = xs - geo.ball_distance_ft
    dy = ys - geo.ball_above_radar_ft
    keep = dx > 0.3  # in front of the tee
    if int(np.sum(keep)) < 3:
        return float("nan"), 0
    dx, dy, w = dx[keep], dy[keep], weights[keep]
    for _ in range(trim_iters + 1):
        slope = float(np.sum(w * dx * dy) / np.sum(w * dx * dx))
        resid = np.abs(dy - slope * dx)
        if len(dx) <= 6:
            break
        order = np.argsort(resid)
        n_keep = max(6, int(len(order) * (1.0 - trim_frac)))
        dx, dy, w = dx[order[:n_keep]], dy[order[:n_keep]], w[order[:n_keep]]
    return math.degrees(math.atan(slope)), len(dx)


def joint_fit_la_tau(
    times_ms: np.ndarray,
    elevations: np.ndarray,
    weights: np.ndarray,
    speed_mph: float,
    geo: Geometry,
    trim_frac: float = 0.3,
) -> tuple[float, float]:
    """Fit (launch_angle, tau) jointly from radar data alone — no truth leakage.

    Trimmed weighted objective: ghost/multipath sub-frames form a second
    population, so the worst `trim_frac` residuals are dropped before scoring.
    Coarse-to-fine grid search.
    """

    def objective(la: float, tau: float) -> float:
        t = times_ms + tau
        t_screen = screen_time_ms(la, speed_mph, geo)
        mask = (t >= 3.0) & (t <= t_screen)
        if int(np.sum(mask)) < 6:
            return float("inf")
        truth = np.array([elevation_truth_deg(ti, la, speed_mph, geo) for ti in t[mask]])
        err = np.abs(elevations[mask] - truth)
        w = weights[mask]
        order = np.argsort(err)
        keep = order[: max(6, int(len(order) * (1.0 - trim_frac)))]
        return float(np.average(err[keep], weights=w[keep]))

    best = (float("nan"), 0.0, float("inf"))
    for la in np.arange(5.0, 35.0, 0.5):
        for tau in np.arange(-80.0, 80.5, 4.0):
            score = objective(la, tau)
            if score < best[2]:
                best = (float(la), float(tau), score)
    la0, tau0, _ = best
    if math.isnan(la0):
        return float("nan"), 0.0
    for la in np.arange(la0 - 1.0, la0 + 1.05, 0.1):
        for tau in np.arange(tau0 - 5.0, tau0 + 5.25, 0.5):
            score = objective(la, tau)
            if score < best[2]:
                best = (float(la), float(tau), score)
    return best[0], best[1]


def session_self_cal(session_obs: list[dict], geo: Geometry) -> tuple[float, dict[int, float]]:
    """Tee-anchor self-calibration: one shared elevation offset per session.

    Every trajectory leaves the tee, so el(t->0) = atan2(tee_y, tee_x) for
    ALL launch angles — early-flight observations identify a session-wide
    additive elevation offset (mount-tilt error + boresight drift), while
    late-flight curvature identifies each shot's LA. Joint fit: grid over
    the shared offset, closed-form per-shot LA scan inside. Truth-free.

    Returns (offset_deg, {shot_row_idx: la_selfcal_deg}).
    """
    if len(session_obs) < 3:
        return float("nan"), {}
    la_grid = np.arange(5.0, 36.0, 0.1)
    curves = []
    for ob in session_obs:
        curves.append(
            np.array(
                [[elevation_truth_deg(t, la, ob["speed"], geo) for t in ob["t"]] for la in la_grid]
            )
        )
    best = (float("inf"), 0.0)
    for delta in np.arange(-4.0, 4.01, 0.1):
        total = 0.0
        for ob, m in zip(session_obs, curves):
            cost = np.average(np.abs(ob["el"][None, :] + delta - m), axis=1, weights=ob["w"])
            total += float(np.min(cost))
        if total < best[0]:
            best = (total, float(delta))
    delta = best[1]
    las: dict[int, float] = {}
    for ob, m in zip(session_obs, curves):
        cost = np.average(np.abs(ob["el"][None, :] + delta - m), axis=1, weights=ob["w"])
        las[ob["row_idx"]] = float(la_grid[int(np.argmin(cost))])
    return delta, las


def fit_launch_angle(
    times_ms: np.ndarray,
    elevations: np.ndarray,
    weights: np.ndarray,
    speed_mph: float,
    geo: Geometry,
) -> float:
    """Grid-fit launch angle: best LA whose truth curve matches observed el(t)."""
    best_la, best_err = float("nan"), float("inf")
    for la in np.arange(0.0, 40.0, 0.05):
        truth = np.array([elevation_truth_deg(t, la, speed_mph, geo) for t in times_ms])
        err = float(np.average(np.abs(elevations - truth), weights=weights))
        if err < best_err:
            best_err, best_la = err, float(la)
    return best_la


def load_compare_csv(path: Path) -> dict[int, dict]:
    out: dict[int, dict] = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                shot = int(row["shot_number_of"])
                out[shot] = {
                    "tm_la": float(row["launch_v_tm"]),
                    "tm_speed": float(row["ball_speed_tm"]),
                    "of_speed": float(row["ball_speed_of"]),
                    "of_la": float(row["launch_v_of"]) if row["launch_v_of"] else None,
                }
            except (ValueError, KeyError):
                continue
    return out


def load_frames_csv(path: Path) -> dict[tuple[int, int], dict]:
    out: dict[tuple[int, int], dict] = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                key = (int(row["shot_number"]), int(row["frame_index"]))
                out[key] = {
                    "t_ms": float(row["t_ms"]),
                    "expected_bin": int(row["expected_bin"]),
                    "elevation_deg": float(row["elevation_deg"])
                    if row.get("elevation_deg")
                    else None,
                    "status": row.get("status", ""),
                    # Full-frame F1B range from the live report: 4x the
                    # coherent integration of the sub-frame F1B estimates
                    "f1b_range_ft": float(row["f1b_range_unwrapped_ft"])
                    if row.get("f1b_range_unwrapped_ft")
                    else None,
                    "f1b_snr": float(row["f1b_same_bin_snr"])
                    if row.get("f1b_same_bin_snr")
                    else None,
                }
            except (ValueError, KeyError):
                continue
    return out


def load_buffers(path: Path) -> dict[int, list[dict]]:
    out: dict[int, list[dict]] = {}
    with open(path) as f:
        for line in f:
            if '"kld7_buffer"' not in line:
                continue
            entry = json.loads(line)
            if entry.get("type") != "kld7_buffer":
                continue
            if entry.get("orientation") != "vertical":
                continue
            out[int(entry["shot_number"])] = entry["frames"]
    return out


def load_shot_meta(path: Path) -> dict[int, dict]:
    """Ball speed + live launch angle per shot from shot_detected entries.

    Fallback source when there is no TrackMan compare CSV (range sessions).
    """
    out: dict[int, dict] = {}
    with open(path) as f:
        for line in f:
            if '"shot_detected"' not in line:
                continue
            entry = json.loads(line)
            if entry.get("type") != "shot_detected":
                continue
            out[int(entry["shot_number"])] = {
                "speed": float(entry["ball_speed_mph"]),
                "live_la": entry.get("launch_angle_vertical"),
                "live_la_source": entry.get("launch_angle_vertical_source"),
            }
    return out


def spearman(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) < 3:
        return float("nan")
    ra = np.argsort(np.argsort(a)).astype(float)
    rb = np.argsort(np.argsort(b)).astype(float)
    ra -= ra.mean()
    rb -= rb.mean()
    denom = math.sqrt(float(np.sum(ra**2)) * float(np.sum(rb**2)))
    return float(np.sum(ra * rb) / denom) if denom > 0 else float("nan")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", required=True, type=Path)
    ap.add_argument("--frames-csv", required=True, type=Path)
    ap.add_argument(
        "--compare-csv",
        type=Path,
        default=None,
        help="TrackMan pairing CSV; omit for range sessions (no truth scoring)",
    )
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--window", type=int, default=64)
    ap.add_argument("--step", type=int, default=16)
    ap.add_argument("--sub-fft", type=int, default=512)
    ap.add_argument("--snr-gate-db", type=float, default=6.0)
    ap.add_argument("--balance-gate", type=float, default=2.5)
    ap.add_argument("--ball-distance-ft", type=float, default=5.0)
    ap.add_argument("--ball-above-radar-ft", type=float, default=-4.0 / 12.0)
    ap.add_argument("--mount-deg", type=float, default=10.0)
    ap.add_argument("--angle-offset-deg", type=float, default=2.5)
    ap.add_argument("--screen-from-ball-ft", type=float, default=12.0)
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument(
        "--drift",
        action="store_true",
        help="compensate intra-frame ball/image phase drift (rates fixed from geometry)",
    )
    ap.add_argument("--drift-nominal-la", type=float, default=19.0)
    ap.add_argument(
        "--tau-bias-ms",
        type=float,
        default=0.0,
        help="constant added to the range-anchored clock; negative corrects "
        "a unit whose F1B range reads long (chain phase offset)",
    )
    ap.add_argument(
        "--frame-window-ms",
        type=float,
        default=70.0,
        help="candidate frame window around nominal timestamps; widen for "
        "sessions with clock drift (tau_range re-centers per shot)",
    )
    args = ap.parse_args()

    geo = Geometry(
        ball_distance_ft=args.ball_distance_ft,
        ball_above_radar_ft=args.ball_above_radar_ft,
        mount_deg=args.mount_deg,
        angle_offset_deg=args.angle_offset_deg,
        screen_from_ball_ft=args.screen_from_ball_ft,
    )
    args.out.mkdir(parents=True, exist_ok=True)

    compare = load_compare_csv(args.compare_csv) if args.compare_csv else {}
    frame_meta = load_frames_csv(args.frames_csv)
    buffers = load_buffers(args.session)
    shot_meta = load_shot_meta(args.session)

    print(
        f"acquisition: {ACQ_MS:.1f} ms / {SAMPLES} samples "
        f"({SAMPLE_DT_MS * 1000:.0f} us per sample)"
    )
    print(
        f"sub-frame: {args.window} samples = {args.window * SAMPLE_DT_MS:.1f} ms, "
        f"step {args.step} = {args.step * SAMPLE_DT_MS:.1f} ms"
    )
    print(f"shots with vertical buffers: {sorted(buffers)}")

    subframe_rows: list[dict] = []
    frame_rows: list[dict] = []
    shot_rows: list[dict] = []
    session_obs: list[dict] = []

    for shot in sorted(buffers):
        truth = compare.get(shot)
        if truth is None:
            meta = shot_meta.get(shot)
            if meta is None or compare:
                print(f"shot {shot}: no TrackMan pairing, skipping")
                continue
            # Range session: no truth, score-free analysis from radar data only
            truth = {
                "tm_la": None,
                "tm_speed": meta["speed"],
                "of_speed": meta["speed"],
                "of_la": meta["live_la"],
            }
        tm_la, tm_speed = truth["tm_la"], truth["tm_speed"]
        has_truth = tm_la is not None
        t_screen = screen_time_ms(tm_la if has_truth else 19.0, tm_speed, geo)

        # Frames considered: generous window around flight to absorb timing jitter
        win = args.frame_window_ms
        candidates = [
            (fi, meta)
            for (s, fi), meta in frame_meta.items()
            if s == shot and -win <= meta["t_ms"] <= t_screen + win
        ]
        candidates.sort()

        frames: list[FrameResult] = []
        buf = buffers[shot]
        for fi, meta in candidates:
            if fi >= len(buf) or not buf[fi].get("radc_b64"):
                continue
            payload = base64.b64decode(buf[fi]["radc_b64"])
            fr = analyze_frame(
                payload,
                meta["t_ms"],
                meta["expected_bin"],
                geo,
                args.window,
                args.step,
                args.sub_fft,
                args.snr_gate_db,
                args.balance_gate,
                drift=args.drift,
                ball_speed_mph=truth["of_speed"],
                nominal_la_deg=args.drift_nominal_la,
            )
            fr.frame_index = fi
            fr.live_elevation_deg = meta["elevation_deg"]
            fr.live_status = meta["status"]
            fr.live_f1b_range_ft = meta.get("f1b_range_ft")
            fr.live_f1b_snr = meta.get("f1b_snr")
            frames.append(fr)

        tau, tau_err = (
            fit_tau_ms(frames, tm_la, tm_speed, geo) if has_truth else (0.0, float("nan"))
        )

        # Per-frame stats and single-frame LA fits
        frame_la_fits: list[tuple[float, float, float]] = []  # (la, coherence, snr)
        all_t, all_el, all_w = [], [], []
        for fr in frames:
            goods = [s for s in fr.subframes if s.good]
            t_shift = np.array([s.t_ms + tau for s in goods])
            in_flight = (t_shift >= 3.0) & (t_shift <= t_screen) if len(goods) else np.array([])
            sub_el = np.array([s.elevation_deg for s in goods])
            sub_w = np.array([s.magnitude for s in goods])
            truth_el = (
                np.array([elevation_truth_deg(t, tm_la, tm_speed, geo) for t in t_shift])
                if len(goods) and has_truth
                else np.array([])
            )
            n_flight = int(np.sum(in_flight)) if len(goods) else 0
            sub_mae = (
                float(np.mean(np.abs(sub_el[in_flight] - truth_el[in_flight])))
                if n_flight and has_truth
                else float("nan")
            )
            frame_center_shifted = fr.t_center_ms + tau
            full_err = (
                fr.full_elevation_deg
                - elevation_truth_deg(frame_center_shifted, tm_la, tm_speed, geo)
                if has_truth and 3.0 <= frame_center_shifted <= t_screen
                else float("nan")
            )
            la_fit = float("nan")
            if has_truth and n_flight >= 6:
                la_fit = fit_launch_angle(
                    t_shift[in_flight],
                    sub_el[in_flight],
                    sub_w[in_flight],
                    tm_speed,
                    geo,
                )
                frame_la_fits.append((la_fit, fr.coherence, fr.full_snr_db))
                all_t.extend(t_shift[in_flight])
                all_el.extend(sub_el[in_flight])
                all_w.extend(sub_w[in_flight] * max(fr.coherence, 0.05))

            frame_rows.append(
                {
                    "shot": shot,
                    "frame_index": fr.frame_index,
                    "t_ms": round(fr.t_ms, 2),
                    "t_center_shifted_ms": round(frame_center_shifted, 2),
                    "in_flight": bool(3.0 <= frame_center_shifted <= t_screen),
                    "full_peak_bin": fr.full_peak_bin,
                    "full_snr_db": round(fr.full_snr_db, 2),
                    "full_elevation_deg": round(fr.full_elevation_deg, 2),
                    "full_el_err_deg": round(full_err, 2) if not math.isnan(full_err) else "",
                    "live_elevation_deg": fr.live_elevation_deg,
                    "live_status": fr.live_status,
                    "n_sub_good": fr.n_good,
                    "n_sub_in_flight": n_flight,
                    "sub_el_mae_deg": round(sub_mae, 2) if not math.isnan(sub_mae) else "",
                    "ripple_el_rms_deg": round(fr.ripple_el_rms_deg, 3),
                    "coherence": round(fr.coherence, 4),
                    "sweep_el_deg": round(fr.sweep_el_deg, 2),
                    "la_fit_deg": round(la_fit, 2) if not math.isnan(la_fit) else "",
                    "la_fit_err_deg": round(la_fit - tm_la, 2) if not math.isnan(la_fit) else "",
                    "el_2ray_deg": round(fr.el_2ray_deg, 2)
                    if not math.isnan(fr.el_2ray_deg)
                    else "",
                    "el_2ray_image_deg": round(fr.el_2ray_image_deg, 2)
                    if not math.isnan(fr.el_2ray_image_deg)
                    else "",
                    "rho_2ray": round(fr.rho_2ray, 3) if not math.isnan(fr.rho_2ray) else "",
                    "chidot_2ray_rad_ms": round(fr.chidot_2ray, 3)
                    if not math.isnan(fr.chidot_2ray)
                    else "",
                    "resid_2ray": round(fr.resid_2ray, 4) if not math.isnan(fr.resid_2ray) else "",
                    "umod_2ray": round(fr.umod_2ray, 3) if not math.isnan(fr.umod_2ray) else "",
                    "range_med_ft": round(fr.range_med_ft, 2)
                    if not math.isnan(fr.range_med_ft)
                    else "",
                }
            )
            for s in fr.subframes:
                subframe_rows.append(
                    {
                        "shot": shot,
                        "frame_index": fr.frame_index,
                        "start": s.start,
                        "t_ms": round(s.t_ms, 2),
                        "t_shifted_ms": round(s.t_ms + tau, 2),
                        "peak_bin": s.peak_bin,
                        "snr_db": round(s.snr_db, 2),
                        "balance": round(s.balance, 3),
                        "elevation_deg": round(s.elevation_deg, 2),
                        "range_ft": round(s.range_ft, 2),
                        "f1b_snr": round(s.f1b_snr, 2),
                        "truth_el_deg": round(
                            elevation_truth_deg(s.t_ms + tau, tm_la, tm_speed, geo), 2
                        )
                        if has_truth
                        else "",
                        "good": s.good,
                    }
                )

        # Honest production-style fit: radar data + OPS speed only.
        # Truth (tm_la) is used solely for scoring afterward.
        raw_t, raw_el, raw_w = [], [], []
        for fr in frames:
            for s in fr.subframes:
                if s.good:
                    raw_t.append(s.t_ms)
                    raw_el.append(s.elevation_deg)
                    raw_w.append(s.magnitude / (1.0 + abs(math.log(max(s.balance, 1e-3)))))
        la_joint, tau_joint = float("nan"), float("nan")
        if len(raw_t) >= 6:
            la_joint, tau_joint = joint_fit_la_tau(
                np.array(raw_t),
                np.array(raw_el),
                np.array(raw_w),
                truth["of_speed"],
                geo,
            )

        # Timing-free position fit: range + elevation per sub-frame, tee anchor.
        # Production-honest: no truth, no clock. F1B-quality gated.
        pos_r, pos_el, pos_w = [], [], []
        for fr in frames:
            for s in fr.subframes:
                if (
                    s.good
                    and s.f1b_snr >= 2.0
                    and 4.5 <= s.range_ft <= 16.0
                    and -5.0 <= s.elevation_deg <= 45.0
                ):
                    pos_r.append(s.range_ft)
                    pos_el.append(s.elevation_deg)
                    pos_w.append(s.magnitude / (1.0 + abs(math.log(max(s.balance, 1e-3)))))
        la_pos, n_pos = float("nan"), 0
        if len(pos_r) >= 6:
            la_pos, n_pos = position_fit_la(np.array(pos_r), np.array(pos_el), np.array(pos_w), geo)

        # Range-anchored timing (multipath-immune) + honest curve fit
        rng_t, rng_r = [], []
        for fr in frames:
            for s in fr.subframes:
                if s.good and s.f1b_snr >= 2.0 and 4.5 <= s.range_ft <= 16.0:
                    rng_t.append(s.t_ms)
                    rng_r.append(s.range_ft)
        tau_range, range_mad = range_anchored_tau(
            np.array(rng_t), np.array(rng_r), truth["of_speed"], geo
        )
        if not math.isnan(tau_range):
            tau_range += args.tau_bias_ms
        la_rt = float("nan")
        if not math.isnan(tau_range) and len(raw_t) >= 6:
            t_shift_rt = np.array(raw_t) + tau_range
            t_screen_of = screen_time_ms(19.0, truth["of_speed"], geo)
            mask_rt = (t_shift_rt >= 3.0) & (t_shift_rt <= t_screen_of)
            if int(np.sum(mask_rt)) >= 6:
                la_rt = fit_launch_angle(
                    t_shift_rt[mask_rt],
                    np.array(raw_el)[mask_rt],
                    np.array(raw_w)[mask_rt],
                    truth["of_speed"],
                    geo,
                )

        # Two-ray demodulated estimators (honest: no truth anywhere).
        # Physicality gates: the demodulated image must be a floor bounce
        # (at/below horizon) unless the fringe is negligible (rho < 0.25,
        # effectively single-ray); ball elevation must be plausible.
        tr_t, tr_el, tr_r, tr_w = [], [], [], []
        for fr in frames:
            if (
                math.isnan(fr.el_2ray_deg)
                or fr.resid_2ray > 0.15
                or not (0.6 <= fr.umod_2ray <= 1.4)
                or not (-5.0 <= fr.el_2ray_deg <= 45.0)
            ):
                continue
            # Valid when: fringe negligible (single-ray), image is a floor
            # bounce (at/below horizon), or the two components are merged
            # (nearly equal angles — low-launch clubs where ball and image
            # haven't separated; either component IS the ball direction)
            image_physical = fr.rho_2ray < 0.25 or (
                not math.isnan(fr.el_2ray_image_deg)
                and (
                    fr.el_2ray_image_deg <= 3.0 or abs(fr.el_2ray_deg - fr.el_2ray_image_deg) <= 4.0
                )
            )
            if not image_physical:
                continue
            tr_t.append(fr.t_center_ms)
            tr_el.append(fr.el_2ray_deg)
            # Prefer the full-frame F1B range at the demod's bin (same
            # target, 4x integration); fall back to the sub-frame median
            if (
                not math.isnan(fr.range_fixedbin_ft)
                and not math.isnan(fr.f1b_fixedbin_snr)
                and fr.f1b_fixedbin_snr >= 2.0
            ):
                tr_r.append(fr.range_fixedbin_ft)
            else:
                tr_r.append(fr.range_med_ft)
            tr_w.append(1.0 / (fr.resid_2ray + 0.02))
        la_2ray_pos, n_2ray_pos = float("nan"), 0
        pos_ok = [i for i in range(len(tr_r)) if not math.isnan(tr_r[i]) and 4.5 <= tr_r[i] <= 16.0]
        if len(pos_ok) >= 2:
            la_2ray_pos, n_2ray_pos = position_fit_la(
                np.array([tr_r[i] for i in pos_ok]),
                np.array([tr_el[i] for i in pos_ok]),
                np.array([tr_w[i] for i in pos_ok]),
                geo,
                trim_iters=1,
                trim_frac=0.2,
            )
        if math.isnan(la_2ray_pos) and len(pos_ok) >= 1:
            # Single-frame solve: direction from tee to the one observed
            # ball position (quality-tier Tier B style)
            i = max(pos_ok, key=lambda j: tr_w[j])
            el_rad = math.radians(tr_el[i])
            bx = tr_r[i] * math.cos(el_rad)
            by = tr_r[i] * math.sin(el_rad)
            la_2ray_pos = math.degrees(
                math.atan2(by - geo.ball_above_radar_ft, bx - geo.ball_distance_ft)
            )
            n_2ray_pos = 1
        la_2ray_curve = float("nan")
        if not math.isnan(tau_range) and len(tr_t) >= 2:
            t_2r = np.array(tr_t) + tau_range
            t_screen_of = screen_time_ms(19.0, truth["of_speed"], geo)
            m2r = (t_2r >= 0.0) & (t_2r <= t_screen_of + 10.0)
            if int(np.sum(m2r)) >= 2:
                la_2ray_curve = fit_launch_angle(
                    t_2r[m2r],
                    np.array(tr_el)[m2r],
                    np.array(tr_w)[m2r],
                    truth["of_speed"],
                    geo,
                )
                # Observations for the session-level tee-anchor self-cal
                session_obs.append(
                    {
                        "row_idx": len(shot_rows),
                        "tm_la": tm_la,
                        "t": t_2r[m2r],
                        "el": np.array(tr_el)[m2r],
                        "w": np.array(tr_w)[m2r],
                        "speed": truth["of_speed"],
                    }
                )

        # Shot-level aggregates
        la_multi = float("nan")
        if len(all_t) >= 6:
            la_multi = fit_launch_angle(
                np.array(all_t), np.array(all_el), np.array(all_w), tm_speed, geo
            )
        best_frame_la = float("nan")
        if frame_la_fits:
            best_frame_la = max(frame_la_fits, key=lambda x: x[1])[0]

        def _err(value: float) -> object:
            if math.isnan(value) or not has_truth:
                return ""
            return round(value - tm_la, 2)

        shot_rows.append(
            {
                "shot": shot,
                "tm_la_deg": tm_la if has_truth else "",
                "tm_speed_mph": tm_speed,
                "live_la_deg": truth["of_la"],
                "live_la_err_deg": round(truth["of_la"] - tm_la, 2)
                if has_truth and truth["of_la"] is not None
                else "",
                "tau_ms": tau,
                "tau_med_err_deg": round(tau_err, 2) if not math.isnan(tau_err) else "",
                "t_screen_ms": round(t_screen, 1),
                "n_frames": len(frames),
                "n_frames_with_la_fit": len(frame_la_fits),
                "la_multi_deg": round(la_multi, 2) if not math.isnan(la_multi) else "",
                "la_multi_err_deg": _err(la_multi),
                "la_best_frame_deg": round(best_frame_la, 2)
                if not math.isnan(best_frame_la)
                else "",
                "la_best_frame_err_deg": _err(best_frame_la),
                "la_joint_deg": round(la_joint, 2) if not math.isnan(la_joint) else "",
                "la_joint_err_deg": _err(la_joint),
                "tau_joint_ms": round(tau_joint, 1) if not math.isnan(tau_joint) else "",
                "la_pos_deg": round(la_pos, 2) if not math.isnan(la_pos) else "",
                "la_pos_err_deg": _err(la_pos),
                "n_pos_points": n_pos,
                "tau_range_ms": round(tau_range, 1) if not math.isnan(tau_range) else "",
                "tau_el_minus_range_ms": round(tau - tau_range, 1)
                if has_truth and not math.isnan(tau_range)
                else "",
                "range_mad_ft": round(range_mad, 2) if not math.isnan(range_mad) else "",
                "la_rangetau_deg": round(la_rt, 2) if not math.isnan(la_rt) else "",
                "la_rangetau_err_deg": _err(la_rt),
                "la_2ray_pos_deg": round(la_2ray_pos, 2) if not math.isnan(la_2ray_pos) else "",
                "la_2ray_pos_err_deg": _err(la_2ray_pos),
                "n_2ray_pos": n_2ray_pos,
                "la_2ray_curve_deg": round(la_2ray_curve, 2)
                if not math.isnan(la_2ray_curve)
                else "",
                "la_2ray_curve_err_deg": _err(la_2ray_curve),
            }
        )
        tm_str = f"{tm_la:5.2f}" if has_truth else "   --"
        print(
            f"shot {shot:2d}: TM {tm_str} | tau_rng {tau_range:+6.1f} ms "
            f"| multi-LA {la_multi:6.2f} | 2ray-pos {la_2ray_pos:6.2f} "
            f"(n={n_2ray_pos}) | 2ray-curve {la_2ray_curve:6.2f}"
        )

        if not args.no_plots:
            # Without truth, reference the honest two-ray fit and use the
            # range-anchored clock for the time axis
            ref_la = tm_la if has_truth else la_2ray_curve
            ref_tau = tau if has_truth else (0.0 if math.isnan(tau_range) else tau_range)
            if ref_la is not None and not math.isnan(ref_la):
                plot_shot(
                    args.out,
                    shot,
                    frames,
                    ref_tau,
                    ref_la,
                    tm_speed,
                    t_screen,
                    geo,
                    truth_label=has_truth,
                )

    # Tee-anchor self-calibration: shared session offset + per-shot LA
    delta_cal, selfcal_las = session_self_cal(session_obs, geo)
    for row in shot_rows:
        row["session_selfcal_offset_deg"] = round(delta_cal, 2) if not math.isnan(delta_cal) else ""
        row["la_selfcal_deg"] = ""
        row["la_selfcal_err_deg"] = ""
    if not math.isnan(delta_cal):
        print(f"tee-anchor self-cal: session elevation offset {delta_cal:+.2f} deg")
        for ob in session_obs:
            la = selfcal_las.get(ob["row_idx"])
            if la is None:
                continue
            row = shot_rows[ob["row_idx"]]
            row["la_selfcal_deg"] = round(la, 2)
            if ob["tm_la"] is not None:
                row["la_selfcal_err_deg"] = round(la - ob["tm_la"], 2)

    write_csv(args.out / "subframes.csv", subframe_rows)
    write_csv(args.out / "frames_summary.csv", frame_rows)
    write_csv(args.out / "shots_summary.csv", shot_rows)

    summarize(frame_rows, shot_rows)


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {path} ({len(rows)} rows)")


def summarize(frame_rows: list[dict], shot_rows: list[dict]) -> None:
    flight = [
        r
        for r in frame_rows
        if r["in_flight"] and r["full_el_err_deg"] != "" and r["n_sub_good"] >= 4
    ]
    if flight:
        full_err = np.array([abs(float(r["full_el_err_deg"])) for r in flight])
        ripple = np.array([float(r["ripple_el_rms_deg"]) for r in flight])
        coh = np.array([float(r["coherence"]) for r in flight])
        snr = np.array([float(r["full_snr_db"]) for r in flight])
        sub_mae = np.array(
            [float(r["sub_el_mae_deg"]) for r in flight if r["sub_el_mae_deg"] != ""]
        )
        print(f"\n=== Per-frame quality-metric correlations (n={len(flight)} in-flight frames) ===")
        print(f"  |full-frame el err| vs ripple:     spearman {spearman(full_err, ripple):+.3f}")
        print(f"  |full-frame el err| vs coherence:  spearman {spearman(full_err, coh):+.3f}")
        print(f"  |full-frame el err| vs SNR:        spearman {spearman(full_err, snr):+.3f}")
        print(f"  full-frame el MAE:  {np.mean(full_err):.2f} deg")
        if len(sub_mae):
            print(f"  sub-frame el MAE:   {np.mean(sub_mae):.2f} deg")

    la_multi_errs = [
        abs(float(r["la_multi_err_deg"])) for r in shot_rows if r["la_multi_err_deg"] != ""
    ]
    la_best_errs = [
        abs(float(r["la_best_frame_err_deg"]))
        for r in shot_rows
        if r["la_best_frame_err_deg"] != ""
    ]
    live_errs = [abs(float(r["live_la_err_deg"])) for r in shot_rows if r["live_la_err_deg"] != ""]
    print("\n=== Launch angle vs TrackMan ===")
    if live_errs:
        print(f"  live pipeline:        MAE {np.mean(live_errs):.2f} deg (n={len(live_errs)})")
    if la_multi_errs:
        print(
            f"  sub-frame multi:      MAE {np.mean(la_multi_errs):.2f} deg (n={len(la_multi_errs)})"
        )
    if la_best_errs:
        print(
            f"  best coherent frame:  MAE {np.mean(la_best_errs):.2f} deg (n={len(la_best_errs)})"
        )
    la_joint_errs = [
        abs(float(r["la_joint_err_deg"])) for r in shot_rows if r["la_joint_err_deg"] != ""
    ]
    if la_joint_errs:
        print(
            f"  joint LA+tau (no truth, OPS speed): "
            f"MAE {np.mean(la_joint_errs):.2f} deg (n={len(la_joint_errs)})"
        )
    la_pos_errs = [abs(float(r["la_pos_err_deg"])) for r in shot_rows if r["la_pos_err_deg"] != ""]
    if la_pos_errs:
        print(
            f"  position fit (timing-free, no truth): "
            f"MAE {np.mean(la_pos_errs):.2f} deg (n={len(la_pos_errs)})"
        )
    la_rt_errs = [
        abs(float(r["la_rangetau_err_deg"])) for r in shot_rows if r["la_rangetau_err_deg"] != ""
    ]
    if la_rt_errs:
        print(
            f"  range-anchored tau fit (no truth): "
            f"MAE {np.mean(la_rt_errs):.2f} deg (n={len(la_rt_errs)})"
        )
    tau_deltas = [
        float(r["tau_el_minus_range_ms"]) for r in shot_rows if r["tau_el_minus_range_ms"] != ""
    ]
    if tau_deltas:
        print(
            f"  tau(el-fit) - tau(range): mean {np.mean(tau_deltas):+.1f} ms, "
            f"std {np.std(tau_deltas):.1f} ms "
            f"(consistent offset => fixed F1B range bias)"
        )
    for key, label in [
        ("la_2ray_pos_err_deg", "two-ray demod + position fit (no truth)"),
        ("la_2ray_curve_err_deg", "two-ray demod + range-tau curve (no truth)"),
        ("la_selfcal_err_deg", "+ tee-anchor session self-cal (no truth)"),
    ]:
        errs = [abs(float(r[key])) for r in shot_rows if r[key] != ""]
        if errs:
            print(f"  {label}: MAE {np.mean(errs):.2f} deg (n={len(errs)})")


def plot_shot(
    out_dir: Path,
    shot: int,
    frames: list[FrameResult],
    tau: float,
    tm_la: float,
    tm_speed: float,
    t_screen: float,
    geo: Geometry,
    truth_label: bool = True,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(12, 11), sharex=True, height_ratios=[2, 1, 1]
    )

    ref_name = "TM truth" if truth_label else "2ray fit"
    tt = np.linspace(1.0, t_screen, 200)
    ax1.plot(
        tt,
        [elevation_truth_deg(t, tm_la, tm_speed, geo) for t in tt],
        "k-",
        lw=2,
        label=f"{ref_name} (LA {tm_la:.1f} deg)",
        zorder=1,
    )
    for fr in frames:
        goods = [s for s in fr.subframes if s.good]
        bads = [s for s in fr.subframes if not s.good]
        if goods:
            sc = ax1.scatter(
                [s.t_ms + tau for s in goods],
                [s.elevation_deg for s in goods],
                c=[s.snr_db for s in goods],
                cmap="viridis",
                vmin=6,
                vmax=25,
                s=30,
                zorder=3,
            )
        if bads:
            ax1.scatter(
                [s.t_ms + tau for s in bads],
                [s.elevation_deg for s in bads],
                marker="x",
                c="lightgray",
                s=15,
                zorder=2,
            )
        ax1.scatter(
            [fr.t_center_ms + tau],
            [fr.full_elevation_deg],
            marker="D",
            facecolors="none",
            edgecolors="red",
            s=90,
            zorder=4,
        )
        if not math.isnan(fr.el_2ray_deg) and fr.resid_2ray <= 0.25:
            ax1.scatter(
                [fr.t_center_ms + tau],
                [fr.el_2ray_deg],
                marker="*",
                c="blue",
                s=160,
                zorder=5,
            )
    ax1.axvline(t_screen, color="brown", ls="--", alpha=0.6, label="screen arrival")
    ax1.axvline(0, color="gray", ls=":", alpha=0.6)
    ax1.set_ylabel("elevation (deg)")
    ax1.set_ylim(-10, 50)
    ax1.set_title(
        f"Shot {shot}: sub-frame STFT elevation track (tau {tau:+.1f} ms applied)\n"
        "dots=sub-frames (color=SNR dB), x=gated out, red diamond=full-frame FFT"
    )
    ax1.legend(loc="upper left")
    if any(s.good for fr in frames for s in fr.subframes):
        fig.colorbar(sc, ax=ax1, label="sub-frame SNR (dB)")

    for fr in frames:
        goods = [s for s in fr.subframes if s.good]
        if goods:
            ax2.plot(
                [s.t_ms + tau for s in goods],
                [s.balance for s in goods],
                ".-",
                alpha=0.7,
            )
    ax2.axhline(1.0, color="k", lw=0.5)
    ax2.set_yscale("log")
    ax2.set_ylabel("|F2A|/|F1A| balance")
    ax2.set_xlim(-40, t_screen + 40)

    # F1B range vs time: multipath-immune progression, truth overlay
    def truth_range(t_ms: float) -> float:
        t = t_ms / 1000.0
        v = tm_speed * MPH_TO_FPS
        la = math.radians(tm_la)
        x = geo.ball_distance_ft + v * math.cos(la) * t
        y = geo.ball_above_radar_ft + v * math.sin(la) * t - 0.5 * G_FT_S2 * t * t
        return math.hypot(x, y)

    ax3.plot(tt, [truth_range(t) for t in tt], "k-", lw=2, label=f"{ref_name} range")
    for fr in frames:
        subs = [s for s in fr.subframes if s.good and s.f1b_snr >= 1.0]
        if subs:
            ax3.scatter(
                [s.t_ms + tau for s in subs],
                [s.range_ft for s in subs],
                c=[min(s.f1b_snr, 10.0) for s in subs],
                cmap="plasma",
                vmin=1,
                vmax=10,
                s=25,
            )
    ax3.set_ylabel("F1B range (ft)")
    ax3.set_ylim(0, 17)
    ax3.set_xlabel("time after impact (ms, tau-shifted)")
    ax3.legend(loc="upper left")

    fig.tight_layout()
    fig.savefig(out_dir / f"shot_{shot:02d}.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
