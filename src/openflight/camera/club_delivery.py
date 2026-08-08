"""Live camera-assisted club delivery (attack angle + club path).

Fusion chain established offline on the 2026-08-07 55-shot session (see
docs/iwr6843-camera-experiment-branch.md, "Working fusion chain"):

1. The IWR6843's linear attack-angle candidate is tightly repeatable but
   biased steep by a stable, club-dependent amount (dominated by the radar
   scattering center migrating down the club through the hitting zone, plus
   a smaller ground-multipath term). A per-club calibration offset corrects
   it. Offsets were measured against July 2026 TrackMan baselines and are
   PENDING per-shot TrackMan validation — everything here stays at the
   experimental estimator level.
2. The down-the-line camera measures the delivery-plane trace — the
   direction of the clubhead's transverse (image-plane) motion, equal to
   atan2(tan AoA, tan path). Validated against TrackMan ratios on 9i
   (−48.4° ± 3.7 vs −48.3) and 7i.
3. Club path follows: tan(path) = tan(AoA_corrected) / tan(trace).

The camera trace is only trustworthy when the chrome shaft saturates the
sensor. In dim scenes the moving-bright mask locks onto the flying ball and
returns a confidently wrong trace (observed 2026-08-07 in evening light),
so a scene-brightness gate reports ``low_light`` instead of a number.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from openflight.camera.club_motion import detect_reference_ball
from openflight.launch_monitor import ClubType

# --- scene / mask constants -------------------------------------------------
# Scene brightness gate: background 99.5th percentile. The 2026-08-07 session
# measured p99 ~147 (9-iron block, clean traces), ~115 (5-iron, mask started
# locking onto the ball) and ~66 (driver block, unusable).
SCENE_P995_MIN = 105.0
# Over-exposure gate: fraction of saturated background pixels. Midday sun
# (2026-08-08 session) saturated large scene regions; the reference-ball
# detector then locks onto arbitrary bright blobs and impact detection
# never sees a departure.
SCENE_SATURATION_MAX = 0.02
# Moving-bright mask (adaptive): pixels saturated NOW but dark in the
# pre-swing background isolate the chrome shaft from static bright clutter.
BRIGHT_NOW_FLOOR = 110.0
BRIGHT_NOW_SCALE = 0.85  # fraction of the background's brightest content
DARK_BG_MARGIN = 25.0
BALL_THRESHOLD_FLOOR = 120.0
BALL_THRESHOLD_SCALE = 0.9

BALL_PATCH_RADIUS_FRAC = 0.4
BALL_PRESENT_DELTA = 30.0
MIN_CLUB_COMPONENT_PX = 40
MAX_COMPONENT_BALL_DIST_PX = 320.0
TIP_BAND_PX = 12.0
PATCH_HALF_PX = 45
SEARCH_HALF_PX = 110
MIN_MATCH_SCORE = 0.30
MIN_PAIR_STEP_PX = 2.0
MIN_PAIRS = 2
PAIR_OFFSETS = (-2, -1, 0, 1)

# --- fusion constants ---------------------------------------------------------
# A trace near 0° or ±90° makes tan(path) = tan(AoA)/tan(trace) explode or
# collapse; require it in a physically plausible band.
TRACE_ABS_RANGE_DEG = (15.0, 85.0)

# Measured AoA offsets (TrackMan July baseline minus radar candidate median,
# 2026-08-07 session, impact time corrected -2 ms). Keyed by nominal loft.
# The iron points are monotone with loft; irons/wedges interpolate along
# that line. The driver's offset is its own measurement (different head
# geometry entirely); woods and hybrids borrow it as the nearest club class.
_IRON_OFFSET_ANCHORS: tuple[tuple[float, float], ...] = (
    (27.0, 9.1),  # 5-iron
    (34.0, 10.5),  # 7-iron
    (42.0, 16.0),  # 9-iron
)
_DRIVER_OFFSET_DEG = 21.6

_NOMINAL_LOFT_DEG: dict[ClubType, float] = {
    ClubType.DRIVER: 10.5,
    ClubType.WOOD_3: 15.0,
    ClubType.WOOD_5: 18.0,
    ClubType.WOOD_7: 21.0,
    ClubType.HYBRID_3: 19.0,
    ClubType.HYBRID_5: 22.0,
    ClubType.HYBRID_7: 25.0,
    ClubType.HYBRID_9: 28.0,
    ClubType.IRON_2: 18.0,
    ClubType.IRON_3: 21.0,
    ClubType.IRON_4: 24.0,
    ClubType.IRON_5: 27.0,
    ClubType.IRON_6: 30.5,
    ClubType.IRON_7: 34.0,
    ClubType.IRON_8: 38.0,
    ClubType.IRON_9: 42.0,
    ClubType.PW: 46.0,
    ClubType.GW: 50.0,
    ClubType.SW: 54.0,
    ClubType.LW: 58.0,
}

_IRON_LIKE = {
    ClubType.IRON_2,
    ClubType.IRON_3,
    ClubType.IRON_4,
    ClubType.IRON_5,
    ClubType.IRON_6,
    ClubType.IRON_7,
    ClubType.IRON_8,
    ClubType.IRON_9,
    ClubType.PW,
    ClubType.GW,
    ClubType.SW,
    ClubType.LW,
}


@dataclass(frozen=True)
class TraceResult:
    """Camera delivery-plane trace for one capture."""

    status: str  # ok | low_light | overexposed | no_ball | no_impact | insufficient_pairs | error
    trace_deg: float | None = None
    n_pairs: int = 0
    match_scores: tuple[float, ...] = ()
    scene_p995: float | None = None
    impact_frame: int | None = None
    detail: str | None = None


@dataclass(frozen=True)
class FusedDelivery:
    """Offset-corrected AoA and trace-derived club path."""

    status: str  # fused | no_radar_aoa | trace_<status> | trace_out_of_range
    attack_angle_deg: float | None = None
    club_path_deg: float | None = None
    aoa_offset_deg: float | None = None
    offset_source: str | None = None  # measured | loft_interpolated | class_extrapolated
    trace_deg: float | None = None


_MEASURED_OFFSETS: dict[ClubType, float] = {
    ClubType.IRON_5: 9.1,
    ClubType.IRON_7: 10.5,
    ClubType.IRON_9: 16.0,
    ClubType.DRIVER: _DRIVER_OFFSET_DEG,
}


def aoa_offset_for_club(club: ClubType) -> tuple[float, str]:
    """Per-club AoA calibration offset and how it was obtained.

    Measured clubs return their measured offsets exactly. Other irons and
    wedges interpolate/extrapolate the measured iron anchors along nominal
    loft; woods and hybrids borrow the driver's offset as the nearest
    head-geometry class. UNKNOWN gets the iron line at a mid-iron loft.
    """
    measured = _MEASURED_OFFSETS.get(club)
    if measured is not None:
        return measured, "measured"
    if club in _IRON_LIKE or club is ClubType.UNKNOWN:
        loft = _NOMINAL_LOFT_DEG.get(club, 34.0)
        lofts = np.array([anchor[0] for anchor in _IRON_OFFSET_ANCHORS])
        offsets = np.array([anchor[1] for anchor in _IRON_OFFSET_ANCHORS])
        offset = float(np.interp(loft, lofts, offsets))
        return round(offset, 2), "loft_interpolated"
    # woods / hybrids: nearest measured class is the driver's hollow head
    return _DRIVER_OFFSET_DEG, "class_extrapolated"


def _adaptive_thresholds(background: np.ndarray) -> tuple[float, float, float, float]:
    """(scene_p995, ball_threshold, bright_now, dark_bg) for one capture."""
    scene_p995 = float(np.percentile(background, 99.5))
    top = float(np.percentile(background, 99.95))
    ball_threshold = max(BALL_THRESHOLD_FLOOR, BALL_THRESHOLD_SCALE * top)
    bright_now = max(BRIGHT_NOW_FLOOR, BRIGHT_NOW_SCALE * top)
    dark_bg = bright_now - DARK_BG_MARGIN
    return scene_p995, ball_threshold, bright_now, dark_bg


def _detect_impact_index(frames: np.ndarray, ball) -> int | None:
    """Last frame the teed ball's core pixels are undisturbed (halo-robust)."""
    radius = max(3, int(round(ball.diameter_px * BALL_PATCH_RADIUS_FRAC)))
    yy, xx = np.mgrid[0 : frames.shape[1], 0 : frames.shape[2]]
    disk = (xx - ball.x) ** 2 + (yy - ball.y) ** 2 <= radius * radius
    reference = float(np.median(frames[:15], axis=0)[disk].mean())
    means = np.array([float(frame[disk].mean()) for frame in frames])
    present = np.abs(means - reference) < BALL_PRESENT_DELTA
    indexes = np.nonzero(present)[0]
    if len(indexes) == 0:
        return None
    for idx in reversed(indexes):
        after = present[idx + 1 : idx + 3]
        if len(after) == 0 or not after.any():
            if idx + 1 < len(frames):
                return int(idx)
    return int(indexes[-1])


def _club_mask(
    frame: np.ndarray, background: np.ndarray, bright_now: float, dark_bg: float
) -> np.ndarray:
    return ((frame > bright_now) & (background < dark_bg)).astype(np.uint8)


def _head_end(mask: np.ndarray, ball) -> tuple[float, float] | None:
    """Ball-side extremal point of the club component nearest the ball."""
    import cv2  # pylint: disable=import-outside-toplevel

    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best = None
    for label in range(1, n_labels):
        if stats[label, cv2.CC_STAT_AREA] < MIN_CLUB_COMPONENT_PX:
            continue
        distance = math.hypot(centroids[label][0] - ball.x, centroids[label][1] - ball.y)
        if best is None or distance < best[0]:
            best = (distance, label)
    if best is None or best[0] > MAX_COMPONENT_BALL_DIST_PX:
        return None
    ys, xs = np.nonzero(labels == best[1])
    points = np.column_stack([xs, ys]).astype(np.float64)
    centered = points - points.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    projections = centered @ vt[0]
    low = points[projections <= projections.min() + TIP_BAND_PX].mean(axis=0)
    high = points[projections >= projections.max() - TIP_BAND_PX].mean(axis=0)
    head = min((low, high), key=lambda end: math.hypot(end[0] - ball.x, end[1] - ball.y))
    return float(head[0]), float(head[1])


def _crop(image: np.ndarray, cx: float, cy: float, half: int) -> np.ndarray | None:
    x0, x1 = int(round(cx)) - half, int(round(cx)) + half
    y0, y1 = int(round(cy)) - half, int(round(cy)) + half
    if x0 < 0 or y0 < 0 or x1 > image.shape[1] or y1 > image.shape[0]:
        return None
    return image[y0:y1, x0:x1]


def estimate_delivery_trace(
    frames: np.ndarray,
    host_timestamp_ns: np.ndarray,
) -> TraceResult:
    """Delivery-plane trace from one DTL capture (frames, per-frame host ns).

    trace = atan2(v_vertical, v_lateral) of the clubhead's transverse motion,
    measured by binary-mask correlation of the club component across the
    frame pairs bracketing impact.
    """
    import cv2  # pylint: disable=import-outside-toplevel

    if frames.ndim != 3 or len(frames) < 20:
        return TraceResult(status="error", detail="frames must be (n>=20, h, w)")
    background = np.median(frames[:15], axis=0)
    scene_p995, ball_threshold, bright_now, dark_bg = _adaptive_thresholds(background)
    if scene_p995 < SCENE_P995_MIN:
        return TraceResult(status="low_light", scene_p995=scene_p995)
    saturated = float(np.mean(background >= 250))
    if saturated > SCENE_SATURATION_MAX:
        return TraceResult(status="overexposed", scene_p995=scene_p995,
                           detail=f"saturated_frac={saturated:.3f}")
    try:
        ball = detect_reference_ball(frames, brightness_threshold=int(ball_threshold))
    except ValueError:
        return TraceResult(status="no_ball", scene_p995=scene_p995)
    impact_idx = _detect_impact_index(frames, ball)
    if impact_idx is None:
        return TraceResult(status="no_impact", scene_p995=scene_p995)

    masks: dict[int, np.ndarray] = {}
    heads: dict[int, tuple[float, float] | None] = {}
    for offset in range(min(PAIR_OFFSETS), max(PAIR_OFFSETS) + 2):
        idx = impact_idx + offset
        if 0 <= idx < len(frames):
            masks[idx] = _club_mask(frames[idx], background, bright_now, dark_bg)
            heads[idx] = _head_end(masks[idx], ball)

    velocities: list[tuple[float, float]] = []
    scores: list[float] = []
    for offset in PAIR_OFFSETS:
        first, second = impact_idx + offset, impact_idx + offset + 1
        if first not in masks or second not in masks or heads.get(first) is None:
            continue
        head = heads[first]
        template = _crop(masks[first].astype(np.float32), head[0], head[1], PATCH_HALF_PX)
        window = _crop(masks[second].astype(np.float32), head[0], head[1], SEARCH_HALF_PX)
        if template is None or window is None or template.sum() < 30:
            continue
        response = cv2.matchTemplate(window, template, cv2.TM_CCORR_NORMED)
        _, score, _, location = cv2.minMaxLoc(response)
        if score < MIN_MATCH_SCORE:
            continue
        dx = location[0] - (SEARCH_HALF_PX - PATCH_HALF_PX)
        dy = location[1] - (SEARCH_HALF_PX - PATCH_HALF_PX)
        if math.hypot(dx, dy) < MIN_PAIR_STEP_PX:
            continue  # self-match on a static blob, not club motion
        dt_s = (int(host_timestamp_ns[second]) - int(host_timestamp_ns[first])) / 1e9
        if dt_s <= 0:
            continue
        velocities.append((dx / dt_s, dy / dt_s))
        scores.append(float(score))

    if len(velocities) < MIN_PAIRS:
        return TraceResult(
            status="insufficient_pairs",
            n_pairs=len(velocities),
            scene_p995=scene_p995,
            impact_frame=impact_idx,
        )
    v_x = float(np.median([v[0] for v in velocities]))
    v_y = float(np.median([v[1] for v in velocities]))
    trace_deg = math.degrees(math.atan2(-v_y, abs(v_x)))
    return TraceResult(
        status="ok",
        trace_deg=round(trace_deg, 2),
        n_pairs=len(velocities),
        match_scores=tuple(round(s, 3) for s in scores),
        scene_p995=round(scene_p995, 1),
        impact_frame=impact_idx,
    )


def fuse_club_delivery(
    radar_attack_angle_deg: float | None,
    trace: TraceResult,
    club: ClubType,
) -> FusedDelivery:
    """Offset-correct the radar AoA candidate and derive club path.

    tan(path) = tan(AoA_corrected) / tan(trace). Both outputs are
    experimental until the per-club offsets are TrackMan-validated.
    """
    offset_deg, offset_source = aoa_offset_for_club(club)
    if radar_attack_angle_deg is None:
        return FusedDelivery(
            status="no_radar_aoa",
            aoa_offset_deg=offset_deg,
            offset_source=offset_source,
            trace_deg=trace.trace_deg,
        )
    attack_angle_deg = radar_attack_angle_deg + offset_deg
    if trace.status != "ok" or trace.trace_deg is None:
        return FusedDelivery(
            status=f"trace_{trace.status}",
            attack_angle_deg=round(attack_angle_deg, 1),
            aoa_offset_deg=offset_deg,
            offset_source=offset_source,
        )
    lo, hi = TRACE_ABS_RANGE_DEG
    if not lo <= abs(trace.trace_deg) <= hi:
        return FusedDelivery(
            status="trace_out_of_range",
            attack_angle_deg=round(attack_angle_deg, 1),
            aoa_offset_deg=offset_deg,
            offset_source=offset_source,
            trace_deg=trace.trace_deg,
        )
    # trace = atan2(tan AoA, tan path), so its sign IS the AoA's sign. A
    # mismatch means the camera locked onto the wrong mover (e.g. the flying
    # ball), so the path would be fabricated.
    if math.copysign(1.0, trace.trace_deg) != math.copysign(1.0, attack_angle_deg):
        return FusedDelivery(
            status="trace_aoa_sign_mismatch",
            attack_angle_deg=round(attack_angle_deg, 1),
            aoa_offset_deg=offset_deg,
            offset_source=offset_source,
            trace_deg=trace.trace_deg,
        )
    path_deg = math.degrees(
        math.atan(
            math.tan(math.radians(attack_angle_deg)) / math.tan(math.radians(trace.trace_deg))
        )
    )
    return FusedDelivery(
        status="fused",
        attack_angle_deg=round(attack_angle_deg, 1),
        club_path_deg=round(path_deg, 1),
        aoa_offset_deg=offset_deg,
        offset_source=offset_source,
        trace_deg=trace.trace_deg,
    )
