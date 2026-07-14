"""Ball detection + range-walk tracking on raw L3 dumps.

The chain that found the ball on every swing this weekend:

1. TDM split + range FFT + MTI (subtract each bin's mean over the loops in a
   burst) — static clutter cancels; the static argmax NEVER sees the ball.
2. Per-loop peak detections inside meter-gates that exclude the golfer blob.
3. RANSAC line fit of range vs time = the unambiguous ball speed. Doppler is
   useless here: at 90 us loop PRI a ~50 m/s ball aliases to walking pace.

Ring slots stream in memory order; the header's ``trigger_frame`` (fw v2+)
gives the true time order and is treated as authoritative.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

LOOP_PRI_S = 90e-6            # per-TX chirp repeat; constant across variants
RANGE_SPAN_M = 6.0            # every cfg keeps a 6 m span: bin = 6.0/n_samples
BALL_GATES_M = ((2.25, 3.75), (3.75, 5.5))
SPEED_BOUNDS_MS = (20.0, 90.0)


@dataclass
class Geometry:
    """Per-dump capture geometry, derived from the dump header."""

    n_frames: int
    chirps_per_frame: int
    n_rx: int
    n_samples: int
    frame_period_s: float
    trigger_frame: int

    @property
    def n_loops(self) -> int:
        """TDM chirp pairs per frame."""
        return self.chirps_per_frame // 2

    @property
    def range_res_m(self) -> float:
        """Range bin size in meters."""
        return RANGE_SPAN_M / self.n_samples

    def loop_time(self, frame: int, loop: int) -> float:
        """Seconds from window start for (ring-slot frame, loop)."""
        slot_order = (frame - self.trigger_frame) % self.n_frames
        return slot_order * self.frame_period_s + loop * LOOP_PRI_S


@dataclass
class BallTrack:
    """Fitted ball range-walk: range(t) = slope_bins*t + intercept_bins."""

    speed_ms: float
    slope_bins: float
    intercept_bins: float
    rms_bins: float
    n_inliers: int
    t_first: float
    t_last: float
    low_confidence: bool

    @property
    def speed_mph(self) -> float:
        """Ball speed in mph."""
        return self.speed_ms * 2.237

    def bin_at(self, t_s: float) -> float:
        """Predicted (fractional) range bin at time t."""
        return self.slope_bins * t_s + self.intercept_bins

    def range_at(self, t_s: float, range_res_m: float) -> float:
        """Predicted range in meters at time t."""
        return self.bin_at(t_s) * range_res_m


def mti_filter(cube: np.ndarray) -> np.ndarray:
    """Raw cube [nf, cpf, nrx, ns] -> complex MTI [nf, 2(tx), loops, nrx, ns].

    Range FFT then per-burst mean removal over loops: statics cancel.
    """
    n_frames, cpf, n_rx, n_samples = cube.shape
    tdm = cube.reshape(n_frames, cpf // 2, 2, n_rx, n_samples)
    tdm = tdm.transpose(0, 2, 1, 3, 4)
    rfft = np.fft.fft(tdm, axis=-1)
    return rfft - rfft.mean(axis=2, keepdims=True)


def loop_power(mti: np.ndarray) -> np.ndarray:
    """MTI residual power per loop: [nf*loops, n_samples]."""
    n_frames = mti.shape[0]
    n_loops = mti.shape[2]
    power = (np.abs(mti) ** 2).sum(axis=(1, 3))
    return power.reshape(n_frames * n_loops, mti.shape[-1])


def _detections(power: np.ndarray, geo: Geometry,
                snr_min: float = 4.0,
                max_range_m: float | None = None
                ) -> tuple[np.ndarray, np.ndarray]:
    """Per-loop sub-bin peaks inside the ball gates, SNR-gated.

    ``max_range_m`` clamps the gates — set it just short of the NET so
    ball-riding-up-the-net motion never enters the track or angle fits.
    """
    n_samples = power.shape[1]
    res = geo.range_res_m
    loops_idx: list[int] = []
    bins: list[float] = []
    for lo_m, hi_m in BALL_GATES_M:
        if max_range_m is not None:
            hi_m = min(hi_m, max_range_m)
        if hi_m <= lo_m:
            continue
        g_lo, g_hi = int(lo_m / res), min(int(hi_m / res), n_samples - 2)
        if g_hi - g_lo < 3:
            continue
        gate = power[:, g_lo:g_hi]
        base = np.median(gate, axis=1) + 1e-12
        idx = np.argmax(gate, axis=1)
        snr = gate[np.arange(len(power)), idx] / base
        for i in np.nonzero(snr > snr_min)[0]:
            peak = float(g_lo + idx[i])
            j = int(peak)
            if g_lo < j < g_hi - 1:
                y_0, y_1, y_2 = power[i, j - 1], power[i, j], power[i, j + 1]
                den = y_0 - 2 * y_1 + y_2
                if den < 0 and abs((y_0 - y_2) / (2 * den)) < 1:
                    peak += (y_0 - y_2) / (2 * den)
            loops_idx.append(i)
            bins.append(peak)
    return np.asarray(loops_idx), np.asarray(bins)


def find_ball(mti: np.ndarray, geo: Geometry, *,
              iterations: int = 2500, seed: int = 1,
              max_range_m: float | None = None) -> BallTrack | None:
    """RANSAC the ball's range walk; None when no plausible streak exists."""
    power = loop_power(mti)
    loops_idx, bins = _detections(power, geo, max_range_m=max_range_m)
    if loops_idx.size < 8:
        return None
    res = geo.range_res_m
    n_loops = geo.n_loops
    times = np.array([geo.loop_time(i // n_loops, i % n_loops)
                      for i in loops_idx])
    tol = 1.2 if geo.n_samples >= 128 else 0.8
    rng = np.random.default_rng(seed)
    best = None   # (n_inliers, slope, intercept, rms, t_first, t_last)
    for _ in range(iterations):
        i, j = rng.choice(times.size, 2, replace=False)
        d_t = times[i] - times[j]
        if abs(d_t) < 3e-3:
            continue
        slope = (bins[i] - bins[j]) / d_t
        if not SPEED_BOUNDS_MS[0] <= slope * res <= SPEED_BOUNDS_MS[1]:
            continue
        icpt = bins[i] - slope * times[i]
        inliers = np.abs(bins - (slope * times + icpt)) < tol
        if inliers.sum() < 8 or (best is not None
                                 and inliers.sum() <= best[0]):
            continue
        design = np.vstack([times[inliers], np.ones(inliers.sum())]).T
        (sl2, ic2), *_ = np.linalg.lstsq(design, bins[inliers], rcond=None)
        if not SPEED_BOUNDS_MS[0] <= sl2 * res <= SPEED_BOUNDS_MS[1]:
            continue
        resid = bins[inliers] - (sl2 * times[inliers] + ic2)
        rms = float(np.sqrt((resid ** 2).mean()))
        best = (int(inliers.sum()), sl2, ic2, rms,
                float(times[inliers].min()), float(times[inliers].max()))
    if best is None:
        return None
    n_inl, slope, icpt, rms, t_first, t_last = best
    span_s = t_last - t_first
    return BallTrack(speed_ms=slope * res, slope_bins=slope,
                     intercept_bins=icpt, rms_bins=rms, n_inliers=n_inl,
                     t_first=t_first, t_last=t_last,
                     low_confidence=bool(rms >= 0.45 or span_s < 0.012))
