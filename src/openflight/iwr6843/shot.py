"""Per-shot pipeline: dump bytes in, measured shot out.

This is the productized form of the weekend field harness — the same chain
the TrackMan validation will run. It deliberately emits EVERYTHING it knows
(all three launch-angle estimators, quality evidence, geometry) and leaves
display/selection policy to the caller; a ``to_shot()`` adapter to the
server's Shot dataclass comes after TrackMan blesses the numbers.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from openflight.iwr6843 import doa, tracking, trajectory
from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.tracking import BallTrack, Geometry
from openflight.iwr6843.trajectory import TrajectoryFit

DEFAULT_FRAME_PERIOD_S = 0.012   # header field is 0 on pre-v3 firmware dumps

# Two-ray configuration per club class, split-half TrackMan-scored on the
# 2026-07-14 session (holdout MAE in comments). PROVISIONAL: one session of
# evidence; revalidate at the next truth session before trusting further.
TH_GATE_RAD = -0.0436            # -2.5 deg: late-track corrupted zone starts
CLUB_POLICY: dict[str, dict] = {
    # wedges never climb out of the clean zone; the exact floor-referenced
    # tee anchor fixes their short lever arm                    (SW 1.31 deg)
    "wedge": {"anchor_tee": True},
    # mid irons climb past boresight mid-track: gate the corrupted zone and
    # restore the lever arm with the anchor              (9i 2.50 / 7i 3.17)
    "mid_iron": {"anchor_tee": True, "th_gate_rad": TH_GATE_RAD},
    # long irons sit near the MTI notch; keep every clean snapshot,
    # downweight mid-mix ones, skip the anchor (track starts can be
    # notch-damaged)                                            (5i 2.43 deg)
    "long_iron": {"dominance": True, "x_max_m": 3.8},
    "driver": {"dominance": True, "x_max_m": 3.8},          # (Dr 0.92 deg)
    "default": {"dominance": True, "x_max_m": 3.8},         # (pooled 2.96)
}
# slowest RADIAL speed a real ball can read, per class: must sit ABOVE the
# class's post-impact club (~0.7x ball) and the flying tee (~25 m/s), or
# they steal the RANSAC track (the 2026-07-14 driver/SW live bug)
CLUB_MIN_BALL_MS: dict[str, float] = {
    "wedge": 26.5,        # slowest real SW ball ~27.5 radial
    "mid_iron": 34.0,     # 9i ball >=39; 9i club ~31
    "long_iron": 40.0,    # 5i ball >=47; 5i club ~37
    "driver": 50.0,       # ball >=58; club ~45
    "default": 30.0,
}

# no-measurement gate: junk captures (ring overwritten, notch-broken tracks)
# produce thin ragged tracks whose angles AND speed are wrong; a product
# says "no read" instead (2026-07-14: 3 driver + 2 5i captures)
REJECT_RMS_BINS = 0.50
REJECT_MIN_INLIERS = 28
REJECT_MIN_SPAN_S = 0.018


def club_class(club: str | None) -> str:
    """Free-text club name -> policy class (best effort, default on miss)."""
    if not club:
        return "default"
    text = club.strip().lower()
    if "wedge" in text or text[:2] in ("sw", "lw", "gw", "pw"):
        return "wedge"
    if "driver" in text or "wood" in text:
        return "driver"
    if "hybrid" in text:
        return "long_iron"
    digits = [ch for ch in text if ch.isdigit()]
    if digits:
        num = int(digits[0])
        if 1 <= num <= 5:
            return "long_iron"
        if 6 <= num <= 9:
            return "mid_iron"
    return "default"


@dataclass
class ShotMeasurement:
    """Everything the radar knows about one swing."""

    geometry: Geometry
    ball_found: bool
    track: BallTrack | None = None
    fits: dict[str, TrajectoryFit] = field(default_factory=dict)
    n_angle_points: int = 0
    quality: str = "ok"                    # ok | low | reject
    ball_speed_corrected_mph: float | None = None
    club: str | None = None

    @property
    def ball_speed_mph(self) -> float | None:
        """Ball speed from the range walk, mph (raw radial)."""
        return self.track.speed_mph if self.track else None

    @property
    def launch_angle_deg(self) -> float | None:
        """Headline launch angle: policy two-ray, then tee, then free.

        Order set by the 2026-07-14 TrackMan session (two-ray beat the
        line fits on every club class once weighted/anchored/gated).
        """
        for method in ("two_ray", "tee", "free"):
            fit = self.fits.get(method)
            if fit is not None:
                return fit.launch_angle_deg
        return None

    def to_dict(self) -> dict:
        """JSON-serializable record for session logging / later pairing."""
        from dataclasses import asdict
        return {
            "geometry": asdict(self.geometry),
            "ball_found": self.ball_found,
            "track": asdict(self.track) if self.track else None,
            "ball_speed_mph": self.ball_speed_mph,
            "ball_speed_corrected_mph": self.ball_speed_corrected_mph,
            "launch_angle_deg": self.launch_angle_deg,
            "fits": {k: asdict(v) for k, v in self.fits.items()},
            "n_angle_points": self.n_angle_points,
            "quality": self.quality,
            "club": self.club,
        }

    def summary(self) -> str:
        """One human line for live display at the tee."""
        if not self.ball_found or self.track is None:
            return "no ball detected"
        if self.quality == "reject":
            return (f"capture rejected (thin/ragged track: "
                    f"{self.track.n_inliers} inliers, "
                    f"rms {self.track.rms_bins:.2f})")
        parts = [f"BALL {self.track.speed_ms:5.1f} m/s"
                 f" = {self.track.speed_mph:5.1f} mph"]
        if self.ball_speed_corrected_mph is not None:
            parts.append(f"(cos-corr {self.ball_speed_corrected_mph:5.1f})")
        angle = self.launch_angle_deg
        if angle is not None:
            parts.append(f"LA {angle:4.1f} deg")
            others = [f"{m}:{f.launch_angle_deg:.1f}"
                      for m, f in self.fits.items()]
            parts.append("(" + " ".join(others) + ")")
        else:
            parts.append("LA n/a")
        if self.track.low_confidence:
            parts.append("[LOW CONF]")
        return "  ".join(parts)


def geometry_from_header(meta: dict) -> Geometry:
    """Dump-header dict -> Geometry (period falls back for pre-v3 dumps)."""
    period_us = meta.get("frame_period_us", 0)
    return Geometry(n_frames=meta["n_frames"],
                    chirps_per_frame=meta["chirps_per_frame"],
                    n_rx=meta["n_rx"], n_samples=meta["n_samples"],
                    frame_period_s=(period_us / 1e6 if period_us
                                    else DEFAULT_FRAME_PERIOD_S),
                    trigger_frame=meta["trigger_frame"])


def process_dump(raw: bytes, cal: Calibration, *,
                 coherent_loops: int = 4,
                 two_ray: bool = True,
                 net_range_m: float | None = None,
                 club: str | None = None) -> ShotMeasurement:
    """Full pipeline on one dump's bytes.

    ``coherent_loops`` trades point count for per-point SNR (see doa);
    ``two_ray`` adds the height-hypothesis estimator (the headline since
    2026-07-15); ``club`` selects the TrackMan-scored per-class two-ray
    configuration (free text, e.g. "7i", "SandWedge"; None = default).
    """
    meta, cube = parse_dump(raw)
    # header n_frames may exceed what actually arrived on a stalled transfer
    geo = geometry_from_header(meta)
    frame_values = geo.chirps_per_frame * geo.n_rx * geo.n_samples
    got_frames = cube.reshape(-1).size // frame_values
    if got_frames < geo.n_frames:
        geo.n_frames = got_frames
        cube = cube[:got_frames]
    mti = tracking.mti_filter(cube)
    # keep everything 25 cm short of the net: a ball riding up the net is
    # an upward mover that tilts every angle fit high (user setup: net
    # ~3 m past the tee)
    max_r = (net_range_m - 0.25) if net_range_m else None
    klass = club_class(club)
    track = tracking.find_ball(mti, geo, max_range_m=max_r,
                               min_ball_ms=CLUB_MIN_BALL_MS[klass])
    result = ShotMeasurement(geometry=geo, ball_found=track is not None,
                             track=track, club=club)
    if track is None:
        return result
    if (track.rms_bins >= REJECT_RMS_BINS
            or track.n_inliers < REJECT_MIN_INLIERS
            or (track.t_last - track.t_first) < REJECT_MIN_SPAN_S):
        result.quality = "reject"
        return result
    if track.low_confidence:
        result.quality = "low"
    # Line fits use K=1: the plentiful per-loop points are what produced
    # the winning consistency in the 2026-07-13 variant shootout. The strict
    # coherent series (K>1) starves typical shots below min_points — it is
    # reserved for two-ray, where few ultra-clean snapshots win.
    points = doa.angle_points(mti, track, geo, cal, coherent_loops=1)
    result.n_angle_points = len(points)
    for fit in (trajectory.fit_free(points, cal),
                trajectory.fit_tee(points, cal)):
        if fit is not None:
            result.fits[fit.method] = fit
    if two_ray:
        series = doa.snapshot_series(mti, track, geo, cal, coherent_loops=1)
        snaps = [(t, r, s) for t, r, s, _snr in series]
        fit = trajectory.fit_two_ray(snaps, cal, min_explained=0.70,
                                     weighted=True, **CLUB_POLICY[klass])
        if fit is not None:
            result.fits[fit.method] = fit
    angle = result.launch_angle_deg
    if angle is not None:
        res_m = geo.range_res_m
        r_first = track.range_at(track.t_first, res_m) - cal.range_bias_m
        r_last = track.range_at(track.t_last, res_m) - cal.range_bias_m
        factor = trajectory.cosine_speed_factor(angle, r_first, r_last, cal)
        result.ball_speed_corrected_mph = float(track.speed_mph / factor)
    return result


def movers_by_slot(raw: bytes) -> list[tuple[int, float, float]]:
    """Diagnostic: per ring slot, (slot, strongest mover range m, power)."""
    meta, cube = parse_dump(raw)
    geo = geometry_from_header(meta)
    mti = tracking.mti_filter(cube)
    power = (np.abs(mti) ** 2).sum(axis=(1, 2, 3))
    res = geo.range_res_m
    lo_bin = max(3, int(0.35 / res))
    out = []
    for slot in range(power.shape[0]):
        best = lo_bin + int(np.argmax(power[slot, lo_bin:]))
        out.append((slot, best * res, float(power[slot, best])))
    return out
