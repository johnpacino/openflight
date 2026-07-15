"""Direction-of-arrival on the 8-element virtual elevation array.

Snapshot chain (hardware-validated 2026-07-12/13):
1. Extract the 2TXx4RX complex values at the ball's range bin from MTI data.
2. TDM Doppler correction — the TX2 chirp trails TX0 by 45 us, so a moving
   ball rotates the second block by 4*pi*v*tau/lambda. v comes from the
   (unambiguous) range walk; the SIGN convention is measured per shot from
   the loop-to-loop phase at the track bins.
3. Physical orientation flip (RX order is reversed on this board) and
   corner-reflector calibration (element phases/gains incl. TX block offset).
4. Angle per snapshot via FBSS-MUSIC with the higher-peak rule (the floor
   image always sits below the direct path), cross-checked against Bartlett.

Coherent integration: consecutive loops within a burst see the ball at
essentially one position, so after motion compensation (the same Doppler
phase used for TDM, applied per loop) they can be summed coherently for
~10*log10(K) dB of SNR before estimation.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.music import LAM, est_bartlett, est_music_fbss_high
from openflight.iwr6843.tracking import LOOP_PRI_S, BallTrack, Geometry

TDM_TAU_S = 45e-6             # TX0 -> TX2 chirp offset inside one loop


@dataclass
class AnglePoint:
    """One angle measurement along the ball's flight (radar frame)."""

    t_s: float
    range_m: float            # bias-corrected slant range
    theta_rad: float          # elevation relative to boresight
    snr: float
    n_summed: int             # loops coherently combined into this point


def measure_tdm_sign(mti: np.ndarray, track: BallTrack, geo: Geometry) -> int:
    """Resolve the Doppler phase sign by comparing measured loop-to-loop
    phase at the track bins against the range-walk prediction."""
    acc = 0j
    for frame in range(geo.n_frames):
        for loop in range(geo.n_loops - 1):
            t_s = geo.loop_time(frame, loop)
            b_0 = int(round(track.bin_at(t_s)))
            b_1 = int(round(track.bin_at(t_s + LOOP_PRI_S)))
            if 2 <= b_0 < geo.n_samples and 2 <= b_1 < geo.n_samples:
                acc += np.vdot(mti[frame, 0, loop, :, b_0],
                               mti[frame, 0, loop + 1, :, b_1])
    if abs(acc) == 0:
        return +1
    psi_pred = 4 * np.pi * track.speed_ms * LOOP_PRI_S / LAM

    def circ_dist(a: float, b: float) -> float:
        return abs(np.angle(np.exp(1j * (a - b))))

    measured = float(np.angle(acc))
    return +1 if circ_dist(measured, psi_pred) <= circ_dist(measured,
                                                            -psi_pred) else -1


def _snapshot(mti: np.ndarray, frame: int, loop: int, rbin: int,
              tdm_phase: float, cal: Calibration) -> np.ndarray:
    """One calibrated 8-element snapshot in physical orientation."""
    snap = np.concatenate([mti[frame, 0, loop, :, rbin],
                           mti[frame, 1, loop, :, rbin]])
    snap = snap.copy()
    snap[4:] *= np.exp(-1j * tdm_phase)
    return cal.apply(snap[::-1])


def snapshot_series(mti: np.ndarray, track: BallTrack, geo: Geometry,
                    cal: Calibration, *, coherent_loops: int = 1,
                    snr_min: float = 8.0
                    ) -> list[tuple[float, float, np.ndarray, float]]:
    """Calibrated snapshots along the fitted track.

    Returns [(t_mid_s, range_m_bias_corrected, 8-el snapshot, snr), ...].
    ``coherent_loops=K`` motion-compensates and sums K consecutive loops
    (~10*log10(K) dB gain); K=1 is the raw per-loop series.
    """
    sign = measure_tdm_sign(mti, track, geo)
    noise = float(np.median(np.abs(mti) ** 2))
    k = max(1, int(coherent_loops))
    out: list[tuple[float, float, np.ndarray, float]] = []
    for frame in range(geo.n_frames):
        for start in range(0, geo.n_loops - k + 1, k):
            t_mid = geo.loop_time(frame, start + (k - 1) / 2.0)
            if not track.t_first - 2e-3 <= t_mid <= track.t_last + 2e-3:
                continue
            # LOCAL radial speed (quadratic track): the ball's radial speed
            # changes along the flight; using the track-average leaves a
            # TX-block phase residual that grows toward the track ends
            # (TrackMan-truth finding, 2026-07-15)
            v_r = track.speed_ms_at(t_mid, geo.range_res_m)
            tdm_phase = sign * 4 * np.pi * v_r * TDM_TAU_S / LAM
            loop_phase = sign * 4 * np.pi * v_r * LOOP_PRI_S / LAM
            acc = np.zeros(8, dtype=complex)
            for off in range(k):
                loop = start + off
                t_s = geo.loop_time(frame, loop)
                rbin = int(round(track.bin_at(t_s)))
                if not 2 <= rbin < geo.n_samples - 1:
                    break
                snap = _snapshot(mti, frame, loop, rbin, tdm_phase, cal)
                acc += snap * np.exp(-1j * loop_phase * off)
            else:
                snr = float((np.abs(acc) ** 2).mean() / (noise * k))
                # STRICT full-gain gate, kept deliberately: on 2026-07-13
                # real shots the two-ray solver worked best on few
                # ultra-clean snapshots (7/8 fits at +/-2.1 deg); relaxing
                # to sqrt(k) admitted clutter that failed the two-source
                # model and starved the fits instead of feeding them
                if snr < snr_min * k:
                    continue
                rng_m = cal.true_range(
                    track.range_at(t_mid, geo.range_res_m))
                out.append((t_mid, rng_m, acc, snr))
    return out


def angle_points(mti: np.ndarray, track: BallTrack, geo: Geometry,
                 cal: Calibration, *, coherent_loops: int = 1,
                 snr_min: float = 8.0,
                 agreement_max_rad: float = np.radians(8.0)
                 ) -> list[AnglePoint]:
    """Per-point elevation estimates along the fitted ball track.

    MUSIC (higher-peak rule) per snapshot, gated on SNR and MUSIC/Bartlett
    agreement (a multipath-corruption tell).
    """
    series = snapshot_series(mti, track, geo, cal,
                             coherent_loops=coherent_loops, snr_min=snr_min)
    noise = float(np.median(np.abs(mti) ** 2))
    k = max(1, int(coherent_loops))
    points: list[AnglePoint] = []
    for t_mid, rng_m, snap, snr in series:
        theta = est_music_fbss_high(snap, noise * k)
        if abs(theta - est_bartlett(snap)) > agreement_max_rad:
            continue
        points.append(AnglePoint(t_s=t_mid, range_m=rng_m,
                                 theta_rad=float(theta), snr=snr,
                                 n_summed=k))
    return points
