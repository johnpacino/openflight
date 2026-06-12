"""Two-ray multipath demodulation estimator for vertical launch angle.

Live port of the validated offline pipeline
(scripts/analysis/kld7_subframe_stft.py; findings in
docs/kld7-subframe-stft-findings.md). Each ~28.6 ms RADC frame is split
into overlapping sub-frame windows; the per-sub-frame Rx2/Rx1 phasor
ratios are fit to a two-ray (ball + floor image) interference model,
recovering the ball's true elevation through ground multipath instead of
averaging across it. Impact time is anchored by the multipath-immune F1B
range progression (range = tee distance at impact), so the estimator is
robust to host/OPS clock offsets.

Validated offline: 2.64 deg MAE pooled PW-4i vs TrackMan (2026-06-08
sessions, angle offset 3.5), 3.16 deg blind on a drift-era holdout.
Known limitations: per-frame radial Doppler tracking (cosine compression
+ drag) recovers most of the DC alias band; only frames whose radial
alias sits inside the +/-4 km/h clutter core are skipped, so just a
narrow band near ~124-128 mph still refuses outright. Low-launch clubs
whose ball/image never separate (driver/3h) also refuse. Refusals fall
back to the geometry estimator and then the club-physics estimate.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

import numpy as np

from .radc import (
    ANTENNA_SPACING_M,
    WAVELENGTH_M,
    aliased_velocity_from_ball_speed_mph,
    expected_ball_bin_from_speed,
    parse_radc_payload,
    to_complex_iq,
)

logger = logging.getLogger(__name__)

MAX_SPEED_KMH = 100.0
FULL_FFT_SIZE = 2048
SAMPLES = 256
SUB_WINDOW = 64
SUB_STEP = 16
SUB_FFT = 512
M_TO_FT = 3.28084
MPH_TO_FTS = 1.4666667
G_FT_S2 = 32.17

# Acquisition timing: complex sample rate covers +/-100 km/h Doppler
_FD_MAX_HZ = 2.0 * (MAX_SPEED_KMH / 3.6) / WAVELENGTH_M
SAMPLE_DT_MS = 1000.0 / (2.0 * _FD_MAX_HZ)
ACQ_MS = SAMPLES * SAMPLE_DT_MS

# Frames whose RADIAL Doppler aliases into the irreducible DC clutter
# core are skipped per frame (the old shot-level gate refused everything
# within +/-15 km/h; per-frame radial tracking recovers most of that band)
DC_CORE_ALIASED_KMH = 4.0

# Per-frame demodulation validity gates (offline-validated)
MAX_FIT_RESID = 0.15
UMOD_RANGE = (0.6, 1.4)
IMAGE_MAX_EL_DEG = 3.0  # floor image must sit at/below horizon ...
MERGED_COMPONENT_DEG = 4.0  # ... unless merged with the ball (low launch)
SINGLE_RAY_RHO = 0.25
EL_RANGE_DEG = (-5.0, 45.0)

# Range-anchored clock plausibility (healthy sessions: +4..+30 ms)
TAU_RANGE_MS = (-15.0, 60.0)

# Sub-frame quality gates
SUB_SNR_GATE_DB = 6.0
SUB_BALANCE_GATE = 2.5
F1B_SUB_SNR_GATE = 1.5
F1B_FIXEDBIN_SNR_GATE = 2.0
RANGE_BAND_FT = (4.5, 16.0)

# Estimator cross-check: position fit vs curve fit agreement bonus
CROSS_CHECK_AGREE_DEG = 2.5

LA_GRID_DEG = np.arange(0.0, 45.0, 0.05)
DRIFT_NOMINAL_LA_DEG = 19.0


@dataclass
class FrameDemod:
    """Per-frame two-ray demodulation output."""

    t_ms: float  # frame timestamp vs impact (end of acquisition)
    t_center_ms: float
    el_ball_deg: float = float("nan")
    el_image_deg: float = float("nan")
    rho: float = float("nan")
    resid: float = float("nan")
    umod: float = float("nan")
    range_ft: float = float("nan")  # full-frame F1B at the demod bin
    sub_ranges: list = field(default_factory=list)  # (t_ms, range_ft) gated subs
    valid: bool = False


@dataclass
class TwoRayEstimate:
    launch_angle_deg: float | None
    confidence: float
    refusal_reason: str | None
    diagnostics: dict


def _refuse(reason: str, diag: dict) -> TwoRayEstimate:
    return TwoRayEstimate(None, 0.0, reason, {**diag, "refusal_reason": reason})


def _hann_fft(iq: np.ndarray, fft_size: int) -> np.ndarray:
    windowed = (iq - np.mean(iq)) * np.hanning(len(iq))
    padded = np.zeros(fft_size, dtype=np.complex128)
    padded[: len(windowed)] = windowed
    return np.fft.fft(padded)


def _circular_window(center: int, half: int, size: int) -> np.ndarray:
    return np.arange(center - half, center + half + 1) % size


def _find_peak_near(mag: np.ndarray, expected: int, half_window: int) -> int:
    idx = _circular_window(expected, half_window, len(mag))
    return int(idx[np.argmax(mag[idx])])


def _noise_floor(mag: np.ndarray, peak: int, peak_guard: int, dc_guard: int) -> float:
    mask = np.ones(len(mag), dtype=bool)
    mask[_circular_window(peak, peak_guard, len(mag))] = False
    mask[:dc_guard] = False
    mask[-dc_guard:] = False
    vals = mag[mask]
    return float(np.median(vals)) if len(vals) else 1.0


def _angle_from_phase(phase_rad: float) -> float:
    sin_theta = phase_rad * WAVELENGTH_M / (2.0 * math.pi * ANTENNA_SPACING_M)
    return math.degrees(math.asin(max(-1.0, min(1.0, sin_theta))))


def elevation_on_trajectory_deg(
    t_ms: float,
    launch_deg: float,
    ball_speed_mph: float,
    distance_ft: float,
    ball_above_radar_ft: float,
) -> float:
    """Apparent elevation of the ball from the radar, t_ms after impact."""
    t = t_ms / 1000.0
    v = ball_speed_mph * MPH_TO_FTS
    la = math.radians(launch_deg)
    x = distance_ft + v * math.cos(la) * t
    y = ball_above_radar_ft + v * math.sin(la) * t - 0.5 * G_FT_S2 * t * t
    return math.degrees(math.atan2(y, x))


def _predicted_drift_rates(
    r_ft: float,
    ball_speed_mph: float,
    distance_ft: float,
    ball_above_radar_ft: float,
    boresight_deg: float,
    nominal_la_deg: float = DRIFT_NOMINAL_LA_DEG,
) -> tuple[float, float]:
    """Intra-frame inter-channel phase drift (rad/ms) for ball and floor image.

    Deterministic from geometry — NOT fitted (a free fit overfits; see
    findings doc). Convention u = e^{-j delta} gives alpha = -d(delta)/dt.
    """
    v = ball_speed_mph * MPH_TO_FTS / 1000.0  # ft/ms
    la = math.radians(nominal_la_deg)
    t_hat = max((r_ft - distance_ft) / v, 0.0)
    x = distance_ft + v * math.cos(la) * t_hat
    y = ball_above_radar_ft + v * math.sin(la) * t_hat
    vx, vy = v * math.cos(la), v * math.sin(la)
    el_rate_ball = (vy * x - vx * y) / (x * x + y * y)  # rad/ms
    y_img = 2.0 * ball_above_radar_ft - y
    el_rate_img = (-vy * x - vx * y_img) / (x * x + y_img * y_img)
    k = 2.0 * math.pi * ANTENNA_SPACING_M / WAVELENGTH_M
    boresight = math.radians(boresight_deg)
    th_b = math.atan2(y, x) - boresight
    th_i = math.atan2(y_img, x) - boresight
    return -k * math.cos(th_b) * el_rate_ball, -k * math.cos(th_i) * el_rate_img


def radial_speed_mph(
    ball_speed_mph: float,
    t_ms: float,
    distance_ft: float,
    ball_above_radar_ft: float,
    nominal_la_deg: float = DRIFT_NOMINAL_LA_DEG,
) -> float:
    """Radar-apparent (radial) ball speed at t_ms after impact.

    Two effects move the apparent Doppler off the OPS impact speed: the
    LOS-to-velocity cosine compression (largest early) and drag
    deceleration (~0.027 mph/ms at iron speeds). Both shift the alias of
    near-DC ball speeds, so per-frame bins recover shots a static
    impact-speed bin would leave inside the clutter core.
    """
    v = ball_speed_mph - 0.027 * max(t_ms, 0.0)
    vf = v * MPH_TO_FTS
    la = math.radians(nominal_la_deg)
    t = max(t_ms, 0.0) / 1000.0
    x = distance_ft + vf * math.cos(la) * t
    y = ball_above_radar_ft + vf * math.sin(la) * t
    return v * math.cos(la - math.atan2(y, x))


def two_ray_fit(
    z: np.ndarray,
    t_rel_ms: np.ndarray,
    w: np.ndarray,
    alpha: float = 0.0,
    beta: float = 0.0,
) -> dict | None:
    """Fit the two-ray interference model to sub-frame phasor ratios.

    Model: z(t) = (u(t) + g e^{j chi t} v(t)) / (1 + g e^{j chi t}) with
    u = e^{-j delta_ball}, v = e^{-j delta_image}, g = rho e^{j chi0}.
    Multiplying out makes z LINEAR in (u0, g*v0, g) for a fixed fringe
    rate chi, so the fit is a 1D grid over chi with batched closed-form
    weighted least squares inside. alpha/beta are fixed intra-frame drift
    rates from geometry. u0 is the ball phasor at the frame center.
    """
    if len(z) < 9:
        return None
    sw = np.sqrt(w)
    zw = z * sw
    z_norm = max(float(np.sum(np.abs(zw) ** 2)), 1e-12)

    chi = np.concatenate([np.arange(-2.5, -0.12, 0.04), np.arange(0.12, 2.5, 0.04)])
    c1 = np.exp(1j * alpha * t_rel_ms)[None, :]
    c2 = np.exp(1j * np.outer(chi + beta, t_rel_ms))
    c3 = -np.exp(1j * np.outer(chi, t_rel_ms)) * z[None, :]
    a_mat = np.stack([np.broadcast_to(c1, c2.shape), c2, c3], axis=-1) * sw[None, :, None]
    ah = a_mat.conj().transpose(0, 2, 1)
    gram = ah @ a_mat
    ridge = 1e-8 * np.trace(gram, axis1=1, axis2=2).real[:, None, None] * np.eye(3)[None]
    rhs = np.einsum("kcn,n->kc", ah, zw)
    try:
        x = np.linalg.solve(gram + ridge, rhs[..., None])[..., 0]
    except np.linalg.LinAlgError:
        return None
    resid = np.sum(np.abs(np.einsum("knc,kc->kn", a_mat, x) - zw[None, :]) ** 2, axis=1) / z_norm
    k = int(np.argmin(resid))
    u, p, q = x[k]
    rho = abs(q)
    v = p / q if rho > 1e-6 else complex(0.0)
    return {
        "ball_phase": -float(np.angle(u)),
        "image_phase": -float(np.angle(v)) if rho > 1e-6 else float("nan"),
        "rho": rho,
        "chi_dot": float(chi[k]),
        "resid": float(resid[k]),
        "u_mod": abs(u),
        "v_mod": abs(v),
    }


def _demodulate_frame(
    payload: bytes,
    frame_t_ms: float,
    expected_bin: int,
    ball_speed_mph: float,
    distance_ft: float,
    ball_above_radar_ft: float,
    boresight_deg: float,
    range_m: float,
) -> FrameDemod:
    parsed = parse_radc_payload(payload)
    f1a = to_complex_iq(parsed["f1a_i"], parsed["f1a_q"])
    f2a = to_complex_iq(parsed["f2a_i"], parsed["f2a_q"])
    f1b = to_complex_iq(parsed["f1b_i"], parsed["f1b_q"])

    out = FrameDemod(t_ms=frame_t_ms, t_center_ms=frame_t_ms - ACQ_MS / 2.0)
    expected_sub = int(round(expected_bin * SUB_FFT / FULL_FFT_SIZE)) % SUB_FFT
    search_half = max(4, int(round(25 * SUB_FFT / FULL_FFT_SIZE)) + 4)
    unambiguous_ft = range_m * M_TO_FT

    peaks: list[int] = []
    sub_ffts: list[tuple[np.ndarray, np.ndarray]] = []
    times: list[float] = []
    for start in range(0, SAMPLES - SUB_WINDOW + 1, SUB_STEP):
        s1 = _hann_fft(f1a[start : start + SUB_WINDOW], SUB_FFT)
        s2 = _hann_fft(f2a[start : start + SUB_WINDOW], SUB_FFT)
        s1b = _hann_fft(f1b[start : start + SUB_WINDOW], SUB_FFT)
        mag = np.sqrt(np.abs(s1) * np.abs(s2))
        peak = _find_peak_near(mag, expected_sub, search_half)
        floor = _noise_floor(mag, peak, peak_guard=16, dc_guard=20)
        snr_db = 20.0 * math.log10(max(mag[peak] / floor, 1e-9))
        m1, m2 = abs(s1[peak]), abs(s2[peak])
        balance = m2 / m1 if m1 > 0 else 0.0
        center_sample = start + SUB_WINDOW / 2.0
        t_sub = frame_t_ms - (SAMPLES - center_sample) * SAMPLE_DT_MS
        good = snr_db >= SUB_SNR_GATE_DB and (1.0 / SUB_BALANCE_GATE <= balance <= SUB_BALANCE_GATE)
        sub_ffts.append((s1, s2))
        times.append(t_sub)
        if good:
            peaks.append(peak)
            # Per-sub-frame F1B range (multipath-immune clock observable)
            nb = _circular_window(peak, 1, SUB_FFT)
            phase_b = float(np.angle(np.sum(mag[nb] * s1b[nb] * np.conj(s1[nb]))))
            range_ft = (phase_b % (2.0 * math.pi)) / (2.0 * math.pi) * unambiguous_ft
            f1b_mag = np.abs(s1b)
            positive = f1b_mag[f1b_mag > 0]
            f1b_floor = float(np.median(positive)) if positive.size else 1.0
            if f1b_mag[peak] / f1b_floor >= F1B_SUB_SNR_GATE:
                out.sub_ranges.append((t_sub, range_ft))

    if len(peaks) < 3:
        return out
    fixed_bin = int(np.median(peaks))

    s1_vals = np.array([fts[0][fixed_bin] for fts in sub_ffts])
    s2_vals = np.array([fts[1][fixed_bin] for fts in sub_ffts])
    ok = np.abs(s1_vals) > 1e-9
    if int(np.sum(ok)) < 9:
        return out
    z = s2_vals[ok] / s1_vals[ok]
    w = np.abs(s1_vals[ok] * s2_vals[ok])
    w = w / max(float(np.max(w)), 1e-12)
    t_rel = np.array(times)[ok] - out.t_center_ms

    # Full-frame F1B range at the demod bin (4x sub-frame integration);
    # used to place the ball for drift compensation
    fft1_full = _hann_fft(f1a, FULL_FFT_SIZE)
    fft1b_full = _hann_fft(f1b, FULL_FFT_SIZE)
    fb = fixed_bin * (FULL_FFT_SIZE // SUB_FFT)
    nb_full = _circular_window(fb, 4, FULL_FFT_SIZE)
    wts = np.abs(fft1_full[nb_full])
    phase_full_b = float(np.angle(np.sum(wts * fft1b_full[nb_full] * np.conj(fft1_full[nb_full]))))
    range_fixedbin = (phase_full_b % (2.0 * math.pi)) / (2.0 * math.pi) * unambiguous_ft
    f1b_full_mag = np.abs(fft1b_full)
    positive = f1b_full_mag[f1b_full_mag > 0]
    floor_b = float(np.median(positive)) if positive.size else 1.0
    if f1b_full_mag[fb] / floor_b >= F1B_FIXEDBIN_SNR_GATE:
        out.range_ft = range_fixedbin

    fit = two_ray_fit(z, t_rel, w)
    r_drift = float("nan")
    if not math.isnan(out.range_ft) and RANGE_BAND_FT[0] <= out.range_ft <= RANGE_BAND_FT[1]:
        r_drift = out.range_ft
    elif out.sub_ranges:
        med = float(np.median([r for _, r in out.sub_ranges]))
        if RANGE_BAND_FT[0] <= med <= RANGE_BAND_FT[1]:
            r_drift = med
    if fit is not None and not math.isnan(r_drift):
        alpha, beta = _predicted_drift_rates(
            r_drift, ball_speed_mph, distance_ft, ball_above_radar_ft, boresight_deg
        )
        fit2 = two_ray_fit(z, t_rel, w, alpha=alpha, beta=beta)
        if fit2 is not None:
            fit = fit2
    if fit is None:
        return out

    el_u = _angle_from_phase(fit["ball_phase"]) + boresight_deg
    el_v = (
        _angle_from_phase(fit["image_phase"]) + boresight_deg
        if not math.isnan(fit["image_phase"])
        else float("nan")
    )
    # Model is symmetric under component swap; physical image is below ball
    ball_el, image_el, rho = el_u, el_v, fit["rho"]
    if (
        not math.isnan(el_v)
        and 0.25 <= fit["rho"] <= 4.0
        and 0.5 <= fit["v_mod"] <= 1.5
        and el_v > el_u
    ):
        ball_el, image_el, rho = el_v, el_u, 1.0 / fit["rho"]

    out.el_ball_deg = ball_el
    out.el_image_deg = image_el
    out.rho = rho
    out.resid = fit["resid"]
    out.umod = fit["u_mod"]

    image_physical = rho < SINGLE_RAY_RHO or (
        not math.isnan(image_el)
        and (image_el <= IMAGE_MAX_EL_DEG or abs(ball_el - image_el) <= MERGED_COMPONENT_DEG)
    )
    out.valid = (
        out.resid <= MAX_FIT_RESID
        and UMOD_RANGE[0] <= out.umod <= UMOD_RANGE[1]
        and EL_RANGE_DEG[0] <= ball_el <= EL_RANGE_DEG[1]
        and image_physical
    )
    return out


def _range_anchored_tau(
    frames: list[FrameDemod], ball_speed_mph: float, distance_ft: float
) -> float:
    """Impact-time offset from the F1B range progression (range = tee at impact)."""
    obs = [
        (t, r)
        for fr in frames
        for (t, r) in fr.sub_ranges
        if RANGE_BAND_FT[0] <= r <= RANGE_BAND_FT[1]
    ]
    if len(obs) < 4:
        return float("nan")
    v_ft_ms = ball_speed_mph * MPH_TO_FTS / 1000.0
    intercepts = np.array([r - v_ft_ms * t for t, r in obs])
    c = float(np.median(intercepts))
    resid = np.abs(intercepts - c)
    keep = resid <= max(3.0 * float(np.median(resid)), 0.5)
    if int(np.sum(keep)) >= 4:
        c = float(np.median(intercepts[keep]))
    return (c - distance_ft) / v_ft_ms


def _fit_curve_la(
    t_ms: np.ndarray,
    el_deg: np.ndarray,
    w: np.ndarray,
    ball_speed_mph: float,
    distance_ft: float,
    ball_above_radar_ft: float,
) -> float:
    """Launch angle whose trajectory elevation curve best matches el(t)."""
    best_la, best_err = float("nan"), float("inf")
    for la in LA_GRID_DEG:
        truth = np.array(
            [
                elevation_on_trajectory_deg(t, la, ball_speed_mph, distance_ft, ball_above_radar_ft)
                for t in t_ms
            ]
        )
        err = float(np.average(np.abs(el_deg - truth), weights=w))
        if err < best_err:
            best_err, best_la = err, float(la)
    return best_la


def _fit_position_la(
    ranges_ft: np.ndarray,
    el_deg: np.ndarray,
    w: np.ndarray,
    distance_ft: float,
    ball_above_radar_ft: float,
) -> float:
    """Timing-free launch angle: tee-anchored line through (range, elevation)."""
    el = np.radians(el_deg)
    dx = ranges_ft * np.cos(el) - distance_ft
    dy = ranges_ft * np.sin(el) - ball_above_radar_ft
    keep = dx > 0.3
    if int(np.sum(keep)) < 1:
        return float("nan")
    dx, dy, w = dx[keep], dy[keep], w[keep]
    if len(dx) == 1:
        return math.degrees(math.atan2(float(dy[0]), float(dx[0])))
    slope = float(np.sum(w * dx * dy) / np.sum(w * dx * dx))
    return math.degrees(math.atan(slope))


def estimate_two_ray(
    frames: list[dict],
    impact_timestamp: float | None,
    ball_speed_mph: float,
    mount_deg: float,
    angle_offset_deg: float,
    distance_ft: float,
    ball_above_radar_ft: float,
    net_distance_ft: float | None = 10.0,
    range_m: float = 5.0,
    frame_window_ms: float = 120.0,
) -> TwoRayEstimate:
    """Estimate vertical launch angle via two-ray demodulation.

    frames: list of dicts with 'timestamp' (host epoch s) and 'radc'
    (3072-byte payload). Refusals return launch_angle_deg=None with a
    reason; callers fall back to the geometry/naive estimators.
    """
    diag: dict = {"estimator": "two_ray"}
    boresight_deg = mount_deg + angle_offset_deg

    if impact_timestamp is None:
        return _refuse("no_impact_timestamp", diag)
    # Flight cap: net arrival or the FSK range wrap, whichever is sooner
    v_fts = ball_speed_mph * MPH_TO_FTS
    wrap_flight_ft = range_m * M_TO_FT - distance_ft
    cap_ft = min(net_distance_ft, wrap_flight_ft) if net_distance_ft else wrap_flight_ft
    t_cap_ms = 1000.0 * cap_ft / max(v_fts * math.cos(math.radians(DRIFT_NOMINAL_LA_DEG)), 1.0)

    demods: list[FrameDemod] = []
    n_core_skipped = 0
    for fr in frames:
        payload = fr.get("radc")
        ts = fr.get("timestamp")
        if payload is None or ts is None:
            continue
        t_ms = (float(ts) - float(impact_timestamp)) * 1000.0
        if not (-frame_window_ms <= t_ms <= t_cap_ms + frame_window_ms):
            continue
        # Per-frame radial expected bin; skip frames whose alias sits in
        # the irreducible clutter core instead of refusing the whole shot
        v_radial = radial_speed_mph(
            ball_speed_mph, t_ms - ACQ_MS / 2.0, distance_ft, ball_above_radar_ft
        )
        aliased = aliased_velocity_from_ball_speed_mph(v_radial)
        if abs(aliased) <= DC_CORE_ALIASED_KMH:
            n_core_skipped += 1
            continue
        demods.append(
            _demodulate_frame(
                bytes(payload),
                t_ms,
                expected_ball_bin_from_speed(v_radial),
                ball_speed_mph,
                distance_ft,
                ball_above_radar_ft,
                boresight_deg,
                range_m,
            )
        )

    diag["n_frames_dc_core_skipped"] = n_core_skipped
    if not demods and n_core_skipped > 0:
        return _refuse("dc_blind_zone", diag)
    tau_ms = _range_anchored_tau(demods, ball_speed_mph, distance_ft)
    diag["tau_range_ms"] = None if math.isnan(tau_ms) else round(tau_ms, 1)
    if math.isnan(tau_ms):
        return _refuse("no_range_track", diag)
    if not TAU_RANGE_MS[0] <= tau_ms <= TAU_RANGE_MS[1]:
        return _refuse("tau_implausible", diag)

    valid = [d for d in demods if d.valid and 0.0 <= d.t_center_ms + tau_ms <= t_cap_ms + 10.0]
    diag["n_frames_valid"] = len(valid)
    diag["frames"] = [
        {
            "t_ms": round(d.t_center_ms + tau_ms, 1),
            "el_deg": round(d.el_ball_deg, 2),
            "el_image_deg": None if math.isnan(d.el_image_deg) else round(d.el_image_deg, 2),
            "rho": round(d.rho, 2),
            "resid": round(d.resid, 4),
            "range_ft": None if math.isnan(d.range_ft) else round(d.range_ft, 2),
        }
        for d in valid
    ]
    if len(valid) == 1:
        # Single-frame position solve (validated offline as "Tier B"):
        # one clean (range, elevation) plus the known tee determines the
        # launch direction with no clock. Demands a tight demod fit and a
        # usable range; confidence capped at the soft-accept threshold.
        d = valid[0]
        if (
            d.resid <= 0.08
            and not math.isnan(d.range_ft)
            and RANGE_BAND_FT[0] <= d.range_ft <= RANGE_BAND_FT[1]
        ):
            el = math.radians(d.el_ball_deg)
            bx = d.range_ft * math.cos(el) - distance_ft
            by = d.range_ft * math.sin(el) - ball_above_radar_ft
            if bx > 0.3:
                la_single = math.degrees(math.atan2(by, bx))
                if 0.0 < la_single < 45.0:
                    diag["la_single_frame_deg"] = round(la_single, 2)
                    logger.info(
                        "[2RAY] single-frame LA %.2f deg (conf 0.68, tau %+.1f ms)",
                        la_single,
                        tau_ms,
                    )
                    return TwoRayEstimate(la_single, 0.68, None, diag)
        return _refuse("too_few_valid_frames", diag)
    if len(valid) < 2:
        return _refuse("too_few_valid_frames", diag)

    t_arr = np.array([d.t_center_ms + tau_ms for d in valid])
    el_arr = np.array([d.el_ball_deg for d in valid])
    w_arr = np.array([1.0 / (d.resid + 0.02) for d in valid])
    la_curve = _fit_curve_la(t_arr, el_arr, w_arr, ball_speed_mph, distance_ft, ball_above_radar_ft)
    diag["la_curve_deg"] = round(la_curve, 2)
    if math.isnan(la_curve) or la_curve <= 0.05 or la_curve >= LA_GRID_DEG[-1] - 0.05:
        return _refuse("curve_fit_at_grid_edge", diag)

    # Timing-free cross-check from frames that also carry a usable range
    pos_idx = [
        i
        for i, d in enumerate(valid)
        if not math.isnan(d.range_ft) and RANGE_BAND_FT[0] <= d.range_ft <= RANGE_BAND_FT[1]
    ]
    la_pos = float("nan")
    if pos_idx:
        la_pos = _fit_position_la(
            np.array([valid[i].range_ft for i in pos_idx]),
            el_arr[pos_idx],
            w_arr[pos_idx],
            distance_ft,
            ball_above_radar_ft,
        )
    diag["la_position_deg"] = None if math.isnan(la_pos) else round(la_pos, 2)

    # Confidence: base for a >=2-frame curve fit sits at the soft-accept
    # threshold; multi-frame depth and cross-check agreement raise it to
    # strict accept. Disagreement with a multi-point position fit refuses.
    confidence = 0.68
    if len(valid) >= 3:
        confidence += 0.06
    if not math.isnan(la_pos):
        if abs(la_pos - la_curve) <= CROSS_CHECK_AGREE_DEG:
            confidence += 0.12
        elif len(pos_idx) >= 2 and abs(la_pos - la_curve) > 2 * CROSS_CHECK_AGREE_DEG:
            return _refuse("estimator_disagreement", diag)
    confidence = round(min(confidence, 0.92), 2)

    logger.info(
        "[2RAY] LA %.2f deg (conf %.2f, %d frames, tau %+.1f ms, pos %s)",
        la_curve,
        confidence,
        len(valid),
        tau_ms,
        "n/a" if math.isnan(la_pos) else f"{la_pos:.2f}",
    )
    return TwoRayEstimate(la_curve, confidence, None, diag)
