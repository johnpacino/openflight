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


@dataclass
class ShotMeasurement:
    """Everything the radar knows about one swing."""

    geometry: Geometry
    ball_found: bool
    track: BallTrack | None = None
    fits: dict[str, TrajectoryFit] = field(default_factory=dict)
    n_angle_points: int = 0

    @property
    def ball_speed_mph(self) -> float | None:
        """Ball speed from the range walk, mph."""
        return self.track.speed_mph if self.track else None

    @property
    def launch_angle_deg(self) -> float | None:
        """Headline launch angle: tee-constrained when available, else free.

        Provisional policy until TrackMan ranks the estimators.
        """
        for method in ("tee", "free", "two_ray"):
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
            "launch_angle_deg": self.launch_angle_deg,
            "fits": {k: asdict(v) for k, v in self.fits.items()},
            "n_angle_points": self.n_angle_points,
        }

    def summary(self) -> str:
        """One human line for live display at the tee."""
        if not self.ball_found or self.track is None:
            return "no ball detected"
        parts = [f"BALL {self.track.speed_ms:5.1f} m/s"
                 f" = {self.track.speed_mph:5.1f} mph"]
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
                 two_ray: bool = True) -> ShotMeasurement:
    """Full pipeline on one dump's bytes.

    ``coherent_loops`` trades point count for per-point SNR (see doa);
    ``two_ray`` adds the (slower) height-hypothesis estimator.
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
    track = tracking.find_ball(mti, geo)
    result = ShotMeasurement(geometry=geo, ball_found=track is not None,
                             track=track)
    if track is None:
        return result
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
        series = doa.snapshot_series(mti, track, geo, cal,
                                     coherent_loops=coherent_loops)
        snaps = [(t, r, s) for t, r, s, _snr in series]
        fit = trajectory.fit_two_ray(snaps, cal)
        if fit is not None:
            result.fits[fit.method] = fit
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
