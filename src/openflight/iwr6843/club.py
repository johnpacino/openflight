"""Club path from the IWR6843's pre-impact frames.

Club path is the horizontal direction the club head travels through impact.
It is a DIRECTION, so it needs at least two spatial samples over time; a
single azimuth reading cannot produce it.

This estimator converts each snapshot to a Cartesian position (x along
boresight, y cross-range) and fits BOTH components linearly against time,
rather than fitting the azimuth angle itself. For a target moving in a
straight line at constant velocity -- the club, over a ~24 ms pre-impact
window -- x(t) and y(t) are each exactly linear in time, while azimuth(t)
and range(t) individually are not. An angle-rate fit's slope is therefore a
window-averaged rate (through the fit's own weighted-mean time, not the
impact instant), and no choice of range-evaluation point can turn that
average back into the instantaneous rate at impact -- confirmed on this
module's own synthetic fixture (2026-07-25): a fitted azimuth rate of
106.4 deg/s against a true instantaneous rate of 64.1 deg/s at impact and
102.5 deg/s at the fit's own mean time. Fitting x and y directly sidesteps
the problem rather than correcting for it: path_deg = atan2(v_y, v_x) is
then exact for straight-line motion, independent of where in the window the
samples happen to sit.

Absolute azimuth enters additively via ``aim_offset_deg``, not as an
estimator input: v_x and v_y come from per-sample DIFFERENCES against the
same track's range, so a constant per-element phase error from the shipped
array calibration (measured on a different board) cancels out of the path
itself and only shifts azimuth's absolute origin -- which is exactly what
``--iwr6843-azimuth-offset-deg`` (a later task) calibrates out.
"""

from __future__ import annotations

import logging
import math
from dataclasses import asdict, dataclass

import numpy as np

from openflight.iwr6843 import doa, tracking
from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.shot import (
    TX2_LOOP_PERIOD_S,
    geometry_from_header,
    is_range_snapshot,
    project_tx_pair,
)

logger = logging.getLogger(__name__)

MPH_PER_MS = 2.23694

# The search gate is ASYMMETRIC about the tee, because the clubhead cannot be
# beyond the ball before it strikes the ball. Admitting the post-tee region
# lets most-inliers RANSAC prefer a slow body/hands mover (~10 m/s, projection
# 0.28-0.32) over the real clubhead (~30 m/s): with a symmetric +/-0.6 m gate
# that happened on 5 of the 14 2026-07-25 captures, and the club was found on
# only 12. Capping at the tee finds it on 14 of 14.
CLUB_APPROACH_DEPTH_M = 0.6  # observed approach spans 0.90-1.33 m at a 1.372 m tee
CLUB_GATE_TEE_MARGIN_M = 0.05  # ~1 range bin, so samples at the tee are not clipped

# Radial speed bounds. Measured pre-impact: 24.7-37.9 m/s for 66-88 mph clubs.
# The 18.0 floor is a second, independent guard against the slow body mover
# above; it is well below the slowest observed club (24.7 m/s) but above that
# mover. NOTE: this floor assumes a full swing. A club under ~40 mph would be
# excluded, so putting and short chipping are out of scope for club path.
CLUB_SPEED_BOUNDS_MS = (18.0, 45.0)

CLUB_MIN_FRAMES = 4
CLUB_MIN_SNAPSHOTS = 24

# How many frames of approach history to fit, counting BACK from impact -- not
# a ring slot. The clubhead's useful range walk spans 0.90-1.33 m (measured,
# 2026-07-25), which it covers in 9-16 ms, so six frames at 4 ms brackets it
# with headroom. Reaching further back only adds samples from below the search
# gate's 0.772 m floor, where the club is still swinging down and around.
PRE_IMPACT_FRAMES = 6

# Provisional. Cannot be derived analytically; set from the observed residual
# distribution across local captures during bring-up.
CLUB_MAX_AZIMUTH_FIT_RESIDUAL_DEG = 0.5

# The track measures RADIAL speed = true club speed x an unknown projection
# factor, so the identity gate is a projection window, not a tolerance. A
# symmetric tolerance would either reject every valid shot or admit anything.
#
# Measured pre-impact across the 14 captures of 2026-07-25: 0.81-1.25, with 12
# of 14 inside (0.70, 1.00). The clubhead travels almost straight down the
# boresight at impact, hence the cluster near 1. Values above 1.0 are not
# physical -- radial speed cannot exceed total speed -- so the headroom to 1.20
# is an error budget for the OPS club-speed reading, not a real projection.
#
# The ceiling cannot go much higher: the BALL's radial speed is 40-49 m/s
# against a 30-39 m/s club, i.e. a ratio of ~1.2-1.3, so a looser ceiling would
# start admitting the ball. PROVISIONAL on one session; revalidate at the next.
CLUB_SPEED_PROJECTION_RANGE = (0.70, 1.20)

# Ceiling on the azimuth swing across the window, measured on PER-FRAME medians
# (see phase_span_rad). Tangential speed is at most ~20 m/s at a ~1.1 m range,
# so the azimuth rate stays under ~18 rad/s; over six 4 ms frames that is
# ~0.44 rad of azimuth, i.e. ~1.3 rad of lambda/2 baseline phase. pi/2 sits
# just above that.
CLUB_MAX_PHASE_SPAN_RAD = math.pi / 2

# How far one snapshot may sit from its own frame's circular median before it
# is discarded. The clubhead's azimuth barely moves within a single 1.08 ms
# burst, so real scatter here is noise, not signal.
CLUB_MAX_PHASE_DEVIATION_RAD = 0.6


@dataclass
class ClubPathResult:
    """One club-path estimate with the evidence behind it."""

    status: str
    path_deg: float | None = None
    confidence: float | None = None
    azimuth_rate_dps: float | None = None
    range_rate_ms: float | None = None
    club_range_m: float | None = None
    n_frames: int = 0
    n_snapshots: int = 0
    n_rejected_snapshots: int = 0
    phase_span_rad: float | None = None
    fit_residual_deg: float | None = None
    track_rms_bins: float | None = None
    track_inliers: int | None = None
    track_span_s: float | None = None

    @property
    def accepted(self) -> bool:
        """Whether a club path was produced."""
        return self.path_deg is not None and self.status.startswith("accepted")

    def to_dict(self) -> dict:
        """JSON-safe diagnostics for the session log."""
        return asdict(self)


def pre_impact_window_s(
    geo: tracking.Geometry,
    impact_t_s: float | None,
    *,
    n_frames: int = PRE_IMPACT_FRAMES,
) -> tuple[float, float] | None:
    """The ``n_frames`` of approach history ending at impact.

    Anchored to the measured impact instant, NOT to a ring slot. The freeze is
    requested by a UART command, so the trigger frame lands late by a variable
    2-4 frames; the previous slot-anchored window returned
    ``(0, n_frames * period)`` and therefore straddled impact on every shot
    measured, handing the estimator the follow-through instead of the approach.

    Returns None when there is no impact instant, or when impact sits at or
    before the start of the ring so no approach history was captured. A window
    shorter than ``n_frames`` is returned rather than rejected: judging whether
    it holds enough evidence is the job of the ``CLUB_MIN_FRAMES`` and
    ``CLUB_MIN_SNAPSHOTS`` gates, which record what they rejected.
    """
    if impact_t_s is None or impact_t_s <= 0.0:
        return None
    if impact_t_s > geo.n_frames * geo.frame_period_s:
        return None
    return (max(0.0, impact_t_s - n_frames * geo.frame_period_s), float(impact_t_s))


def _frame_medians(
    phases: np.ndarray, frames: np.ndarray
) -> list[tuple[int, float]]:
    """Each frame's circular median phase, in frame order."""
    return [
        (int(frame), doa.circular_median(list(phases[frames == frame])))
        for frame in np.unique(frames)
    ]


def phase_outlier_mask(phases: np.ndarray, frames: np.ndarray) -> np.ndarray:
    """Keep snapshots within CLUB_MAX_PHASE_DEVIATION_RAD of their frame median.

    Judged per frame, never globally: the clubhead's azimuth genuinely drifts
    from frame to frame -- that drift IS the signal -- while within one 1.08 ms
    burst it barely moves, so spread inside a frame is noise. Circular
    differences, so samples straddling +/-pi read as neighbours.
    """
    keep = np.zeros(phases.shape, dtype=bool)
    for frame, median in _frame_medians(phases, frames):
        selector = frames == frame
        deviation = np.abs(np.angle(np.exp(1j * (phases[selector] - median))))
        keep[selector] = deviation <= CLUB_MAX_PHASE_DEVIATION_RAD
    return keep


def phase_span_rad(phases: np.ndarray, frames: np.ndarray) -> float:
    """Total azimuth swing across the window, from per-frame medians.

    Deliberately NOT the range of the raw samples, and deliberately not an
    unwrapped range. A lambda/2 baseline carries no information beyond +/-pi,
    so unwrapping fabricates angles and turns per-snapshot noise into
    cumulative drift -- which is what rejected 4 of the 14 2026-07-25 shots
    with apparent swings of 1.0-17.3 rad. Successive frame medians are close
    together, so accumulating their circular differences is well posed.
    """
    medians = [median for _frame, median in _frame_medians(phases, frames)]
    if len(medians) < 2:
        return 0.0
    steps = [
        math.remainder(later - earlier, math.tau)
        for earlier, later in zip(medians, medians[1:])
    ]
    walk = np.cumsum([0.0] + steps)
    return float(np.ptp(walk))


def find_club(
    mti: np.ndarray,
    geo: tracking.Geometry,
    *,
    tee_range_m: float,
    window_s: tuple[float, float],
):
    """Track the clubhead's approach inside the pre-impact window.

    Two constraints do the separating, and both are load-bearing:

    - The time window, which must END at impact. The club's radial speed
      (24.7-37.9 m/s measured) overlaps the ball's (40-49 m/s), and the ball
      crosses this range gate on its way out, so without the window this fits
      the ball.
    - The gate's upper edge at the tee. Before impact the clubhead is always
      short of the ball, so anything beyond the tee is body, hands, or clutter
      -- and being slower, it wins most-inliers RANSAC when admitted.
    """
    lo = max(0.35, tee_range_m - CLUB_APPROACH_DEPTH_M)
    hi = tee_range_m + CLUB_GATE_TEE_MARGIN_M
    return tracking.find_ball(
        mti,
        geo,
        gates_m=((lo, hi),),
        speed_bounds_ms=CLUB_SPEED_BOUNDS_MS,
        min_ball_ms=CLUB_SPEED_BOUNDS_MS[0],
        time_window_s=window_s,
    )


def estimate_club_path(
    raw: bytes,
    cal: Calibration,
    *,
    ops_club_speed_mph: float,
    impact_t_s: float | None,
    aim_offset_deg: float = 0.0,
    tdm_sign: int = 1,
) -> ClubPathResult:
    """Estimate club path from the approach history ending at ``impact_t_s``.

    ``impact_t_s`` is seconds from the oldest retained frame, as located by
    ``shot.impact_time_s`` from the ball's own range walk. It is a required
    argument with no default on purpose: impact's ring slot varies shot to
    shot, so there is no safe value to assume.
    """
    meta0, _ = parse_dump(raw)
    if meta0.get("n_tx") != 3:
        return ClubPathResult(status="rejected_requires_three_tx")
    if impact_t_s is None:
        return ClubPathResult(status="rejected_no_impact_time")

    projected = project_tx_pair(raw, (0, 2))
    meta, cube = parse_dump(projected)
    geo = geometry_from_header(meta, loop_period_s=TX2_LOOP_PERIOD_S)
    res = geo.range_res_m

    window_s = pre_impact_window_s(geo, impact_t_s)
    if window_s is None:
        return ClubPathResult(status="rejected_no_pre_impact_frames")

    mti = tracking.mti_filter(cube, range_domain=is_range_snapshot(meta), geometry=geo)
    track = find_club(mti, geo, tee_range_m=cal.tee_range_m, window_s=window_s)
    if track is None:
        return ClubPathResult(status="rejected_no_club_track")

    result = ClubPathResult(
        status="pending",
        range_rate_ms=track.slope_bins * res,
        track_rms_bins=track.rms_bins,
        track_inliers=track.n_inliers,
        track_span_s=track.t_last - track.t_first,
    )

    projection = abs(result.range_rate_ms) / max(ops_club_speed_mph / MPH_PER_MS, 1e-6)
    low, high = CLUB_SPEED_PROJECTION_RANGE
    if not low <= projection <= high:
        result.status = "rejected_club_speed_mismatch"
        return result

    # TX2 phase needs all three transmitters, so re-split the ORIGINAL dump.
    _meta3, cube3 = parse_dump(raw)
    n_frames, chirps, n_rx, n_samples = cube3.shape
    n_tx = 3
    loops = chirps // n_tx
    tdm = cube3.reshape(n_frames, loops, n_tx, n_rx, n_samples)
    # Range-snapshot dumps (production) are already range-domain; raw-ADC
    # dumps (replay of older captures) still need the range FFT. Mirror
    # lcmf._tx2_horizontal_proxy: FFTing an already-FFT'd range snapshot a
    # second time would flatten a real peak into noise.
    rfft = tdm if is_range_snapshot(_meta3) else np.fft.fft(tdm, axis=-1)
    tdm = rfft - rfft.mean(axis=1, keepdims=True)

    times: list[float] = []
    phases: list[float] = []
    weights: list[float] = []
    frame_ids: list[int] = []
    frames_seen: set[int] = set()
    for frame in range(geo.n_frames):
        for loop in range(geo.n_loops):
            t_s = geo.loop_time(frame, loop)
            if not track.t_first <= t_s <= track.t_last:
                continue
            absolute_bin = int(round(track.bin_at(t_s)))
            if not geo.contains_bin(absolute_bin, margin=1, frame=frame):
                continue
            local_bin = geo.local_bin(absolute_bin, frame)
            if not 0 <= local_bin < n_samples:
                continue
            sample = doa.tx2_phase_at(
                tdm,
                frame,
                loop,
                local_bin,
                velocity_ms=track.speed_ms_at(t_s, res),
                tdm_sign=tdm_sign,
                n_rx=n_rx,
            )
            if sample is None:
                continue
            phase, weight = sample
            times.append(t_s)
            phases.append(phase)
            weights.append(weight)
            frame_ids.append(frame)
            frames_seen.add(frame)

    # Discard snapshots that disagree with their own frame before counting, so
    # the reported counts are the evidence the fit actually used.
    phase_array = np.asarray(phases)
    frame_array = np.asarray(frame_ids)
    if phase_array.size:
        keep = phase_outlier_mask(phase_array, frame_array)
        result.n_rejected_snapshots = int((~keep).sum())
        phase_array = phase_array[keep]
        frame_array = frame_array[keep]
        times = list(np.asarray(times)[keep])
        weights = list(np.asarray(weights)[keep])
        frames_seen = set(frame_array.tolist())

    result.n_snapshots = len(times)
    result.n_frames = len(frames_seen)
    if result.n_frames < CLUB_MIN_FRAMES or result.n_snapshots < CLUB_MIN_SNAPSHOTS:
        result.status = "rejected_insufficient_snapshots"
        return result

    result.phase_span_rad = phase_span_rad(phase_array, frame_array)
    if result.phase_span_rad > CLUB_MAX_PHASE_SPAN_RAD:
        result.status = "rejected_phase_span"
        return result

    # Phase -> azimuth. The wrapped phase is used as-is: a lambda/2 baseline is
    # unambiguous over exactly +/-pi, which arcsin maps onto the full +/-90
    # degree field of view. Unwrapping would invent angles beyond it.
    azimuth_rad = np.arcsin(np.clip(phase_array / math.pi, -1.0, 1.0))
    t_array = np.asarray(times)
    weight_array = np.asarray(weights)

    # Per-sample position in Cartesian coordinates: x along boresight, y
    # cross-range. r comes from the track's own range-vs-time fit (smooth;
    # de-noises the per-loop range), az from this sample's own phase.
    r_i = track.range_at(t_array, res)
    x_i = r_i * np.cos(azimuth_rad)
    y_i = r_i * np.sin(azimuth_rad)

    # x(t) and y(t) are each exactly linear in time for a target moving in
    # a straight line at constant velocity, so a weighted linear fit's
    # slope is v_x/v_y with no bias from where the samples sit in the
    # window -- unlike fitting the azimuth ANGLE, whose rate of change is
    # only constant for motion on a circular arc (see module docstring).
    design = np.vstack([t_array, np.ones(t_array.size)]).T
    weighted_design = design * np.sqrt(weight_array)[:, None]
    targets = np.stack([x_i, y_i], axis=1) * np.sqrt(weight_array)[:, None]
    (v_x, v_y), (_x0, y0) = np.linalg.lstsq(weighted_design, targets, rcond=None)[0]

    # Fit quality: cross-range residual is the direct, first-order-sensitive
    # signal for azimuth/phase noise (the x-residual is dominated by the
    # track's own range-fit smoothness and carries little independent
    # information). Expressed in degrees at the mean range so the existing,
    # separately-calibrated CLUB_MAX_AZIMUTH_FIT_RESIDUAL_DEG threshold still
    # applies to a quantity of the same scale as before.
    mean_r = float(np.mean(r_i))
    cross_range_resid_m = y_i - (v_y * t_array + y0)
    result.fit_residual_deg = float(
        np.degrees(np.sqrt((cross_range_resid_m**2).mean()) / max(mean_r, 1e-9))
    )
    if result.fit_residual_deg > CLUB_MAX_AZIMUTH_FIT_RESIDUAL_DEG:
        result.status = "rejected_azimuth_fit"
        return result

    result.club_range_m = mean_r
    # Diagnostic only (not a path input): the azimuth rate implied by the
    # cross-range velocity at the mean range.
    result.azimuth_rate_dps = float(np.degrees(v_y / max(mean_r, 1e-9)))

    path_deg = math.degrees(math.atan2(v_y, v_x))
    result.path_deg = path_deg + aim_offset_deg
    result.confidence = _confidence(result)
    result.status = "accepted"
    logger.info(
        "[CLUB] path=%.2f deg (rate %.1f deg/s, residual %.2f deg, %d frames)",
        result.path_deg,
        result.azimuth_rate_dps,
        result.fit_residual_deg,
        result.n_frames,
    )
    return result


def _confidence(result: ClubPathResult) -> float:
    """Confidence from fit residual and evidence count, never a constant."""
    residual = result.fit_residual_deg or CLUB_MAX_AZIMUTH_FIT_RESIDUAL_DEG
    residual_score = max(0.0, 1.0 - residual / CLUB_MAX_AZIMUTH_FIT_RESIDUAL_DEG)
    evidence_score = min(1.0, result.n_snapshots / (2.0 * CLUB_MIN_SNAPSHOTS))
    return round(0.2 + 0.7 * residual_score * evidence_score, 3)


__all__ = [
    "ClubPathResult",
    "estimate_club_path",
    "find_club",
    "pre_impact_window_s",
]
