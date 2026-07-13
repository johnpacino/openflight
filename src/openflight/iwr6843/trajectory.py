"""Launch-angle estimation from the angle/snapshot series of one shot.

Three estimators, deliberately kept side by side until TrackMan arbitrates:

- ``fit_free``     — unconstrained line through the (x, h) points.
- ``fit_tee``      — line forced through the tape-measured launch point;
                     removes the intercept, where multipath does its damage.
- ``fit_two_ray``  — the physics fix: per snapshot, hypothesize the ball
                     height and model direct + floor-image arrivals JOINTLY
                     (the image angle is geometrically determined by the
                     height hypothesis), picking the height that best explains
                     the 8-element data. Uses the reflection as signal.

All fits work in ground coordinates: x = horizontal meters from the radar,
h = meters above the RADAR PLANE (tilt already applied). The 2026-07-13
finding that h(x) tracks are CURVED by multipath is why no line fit is
trusted absolutely yet.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.doa import AnglePoint
from openflight.iwr6843.music import steer


@dataclass
class TrajectoryFit:
    """One launch-angle estimate with its quality evidence."""

    method: str
    launch_angle_deg: float
    n_points: int
    h_rms_m: float                 # scatter about the fit
    launch_cross_m: float          # where the fit meets the radar plane


def _ground_xy(points: list[AnglePoint], cal: Calibration
               ) -> tuple[np.ndarray, np.ndarray]:
    """AnglePoints -> (x horizontal, h above radar plane)."""
    theta = np.array([p.theta_rad for p in points]) + cal.tilt_rad
    rng = np.array([p.range_m for p in points])
    return rng * np.cos(theta), rng * np.sin(theta)


def fit_free(points: list[AnglePoint], cal: Calibration,
             min_points: int = 8) -> TrajectoryFit | None:
    """Unconstrained least-squares line fit."""
    if len(points) < min_points:
        return None
    x_m, h_m = _ground_xy(points, cal)
    slope, icpt = np.polyfit(x_m, h_m, 1)
    resid = h_m - (slope * x_m + icpt)
    cross = float(-icpt / slope) if slope else float("nan")
    return TrajectoryFit(method="free",
                         launch_angle_deg=float(np.degrees(np.arctan(slope))),
                         n_points=len(points),
                         h_rms_m=float(np.sqrt((resid ** 2).mean())),
                         launch_cross_m=cross)


def fit_tee(points: list[AnglePoint], cal: Calibration,
            min_points: int = 8) -> TrajectoryFit | None:
    """Line forced through the measured launch point (cal.tee_range_m)."""
    if len(points) < min_points or cal.tee_range_m is None:
        return None
    x_0, h_0 = cal.tee_range_m, cal.tee_height_m
    x_m, h_m = _ground_xy(points, cal)
    d_x = x_m - x_0
    slope = float(np.sum(d_x * (h_m - h_0)) / np.sum(d_x * d_x))
    resid = h_m - (h_0 + slope * d_x)
    return TrajectoryFit(method="tee",
                         launch_angle_deg=float(np.degrees(np.arctan(slope))),
                         n_points=len(points),
                         h_rms_m=float(np.sqrt((resid ** 2).mean())),
                         launch_cross_m=float(x_0 - h_0 / slope) if slope
                         else float("nan"))


def _two_ray_height(snapshot: np.ndarray, x_m: float, radar_height_m: float,
                    tilt_rad: float, grid_m: np.ndarray
                    ) -> tuple[float, float]:
    """Best-fit ball height ABOVE THE FLOOR for one snapshot.

    For each hypothesized height the direct and floor-image arrival angles
    are fixed by geometry; solve the two complex amplitudes by least squares
    and keep the hypothesis with the smallest residual. Returns
    (height_m, explained_fraction).
    """
    n_el = len(snapshot)
    power = float(np.vdot(snapshot, snapshot).real) + 1e-12
    best_h, best_expl = float("nan"), -1.0
    for h_b in grid_m:
        theta_d = np.arctan2(h_b - radar_height_m, x_m) - tilt_rad
        theta_i = np.arctan2(-(h_b + radar_height_m), x_m) - tilt_rad
        basis = np.column_stack([steer(theta_d, n_el), steer(theta_i, n_el)])
        coef, *_ = np.linalg.lstsq(basis, snapshot, rcond=None)
        resid = snapshot - basis @ coef
        expl = 1.0 - float(np.vdot(resid, resid).real) / power
        if expl > best_expl:
            best_h, best_expl = float(h_b), expl
    return best_h, best_expl


def fit_two_ray(snap_points: list[tuple[float, float, np.ndarray]],
                cal: Calibration, *, radar_height_m: float | None = None,
                grid_step_m: float = 0.01, min_points: int = 8,
                min_explained: float = 0.85) -> TrajectoryFit | None:
    """Two-ray trajectory: per-snapshot height solve, then a line fit.

    ``snap_points`` is [(t_s, range_m, calibrated 8-el snapshot), ...] from
    :func:`openflight.iwr6843.doa.snapshot_series`. Heights are estimated
    above the FLOOR; the returned fit is converted back to radar-plane
    coordinates so it is comparable with the line fits.
    """
    if radar_height_m is None:
        radar_height_m = float(cal.meta.get("radar_height_m", 0.152))
    grid = np.arange(-0.02, 1.30, grid_step_m)
    xs: list[float] = []
    hs: list[float] = []
    for _t, rng_m, snap in snap_points:
        x_m = float(rng_m)                 # slant ~ horizontal at these angles
        height, explained = _two_ray_height(snap, x_m, radar_height_m,
                                            cal.tilt_rad, grid)
        if explained >= min_explained and np.isfinite(height):
            xs.append(x_m)
            hs.append(height)
    if len(xs) < min_points:
        return None
    x_a, h_a = np.asarray(xs), np.asarray(hs)
    slope, icpt = np.polyfit(x_a, h_a, 1)
    resid = h_a - (slope * x_a + icpt)
    # launch_cross in radar-plane coords: floor height == radar_height below
    cross = float((radar_height_m - icpt) / slope) if slope else float("nan")
    return TrajectoryFit(method="two_ray",
                         launch_angle_deg=float(np.degrees(np.arctan(slope))),
                         n_points=len(xs),
                         h_rms_m=float(np.sqrt((resid ** 2).mean())),
                         launch_cross_m=cross)
