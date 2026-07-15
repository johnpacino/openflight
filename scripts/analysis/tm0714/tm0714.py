"""Shared loaders + truth model for the 2026-07-14 TrackMan session analysis.

Builds on the aligned CSV (TM spine) + local dumps. Truth model: TM launch
params (LA, speed, direction) + gravity over the radar's 1.6-4.9 m window,
per-block mount geometry. All analyses import from here.
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from openflight.iwr6843 import doa, tracking
from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.shot import geometry_from_header
from openflight.iwr6843.tracking import BallTrack, Geometry, LOOP_PRI_S
from openflight.iwr6843.music import LAM

DATA = Path.home() / "openflight_sessions" / "tm_0714"
CSV = DATA / "session_aligned_2026-07-14.csv"
TM_JSON = Path("/Users/john.pacino/Projects/openflight/firmware/l3_dump/"
               "trackman_session_data_7_14.json")
CAL_JSON = Path("/Users/john.pacino/Projects/openflight/config/"
                "iwr6843_cal_20260712.json")
G = 9.81
MPH = 2.23694

# mount geometry per shot-number block: (tee_slant_m, radar_height_m, tilt_deg)
BLOCKS = {
    "A": dict(lo=62, hi=124, tee=1.575, rh=0.1524, tilt=10.404948650719305),
    "B": dict(lo=130, hi=137, tee=1.98, rh=0.5715, tilt=6.6),
    "C": dict(lo=142, hi=174, tee=1.92, rh=0.2667, tilt=3.8),
}
Y0_M = 0.064          # ball ~2.5 in off antenna centerline (user, sign +)


def block_for(num: int) -> str | None:
    for name, b in BLOCKS.items():
        if b["lo"] <= num <= b["hi"]:
            return name
    return None


@dataclass
class TruthShot:
    """One aligned shot: TM truth + geometry + radar file."""

    num: int
    club: str
    file: Path
    block: str
    tee_slant: float
    rh: float
    tilt_deg: float
    tm_la: float
    tm_ball_ms: float
    tm_dir_deg: float
    tm_club_mph: float
    tm_spin: float
    tm_attack: float
    of_ball_mph: float | None
    live: dict = field(default_factory=dict)   # live CSV LA columns

    @property
    def z0(self) -> float:
        """Ball launch height above floor."""
        return 0.065 if self.club.startswith("Driver") else 0.04

    @property
    def x_tee(self) -> float:
        """Horizontal tee distance (slant tape corrected for height)."""
        dz = self.z0 - self.rh
        return math.sqrt(max(self.tee_slant ** 2 - dz ** 2, 0.25))

    # ---- truth trajectory in world frame (radar at origin, x downrange) ----
    def z_at(self, x: float) -> float:
        """Ball height above floor at horizontal distance x."""
        la = math.radians(self.tm_la)
        d = x - self.x_tee
        t = d / (self.tm_ball_ms * math.cos(la) + 1e-9)
        return self.z0 + math.tan(la) * d - 0.5 * G * t * t

    def y_at(self, x: float) -> float:
        return Y0_M + (x - self.x_tee) * math.tan(
            math.radians(self.tm_dir_deg))

    def r_at(self, x: float) -> float:
        z, y = self.z_at(x), self.y_at(x)
        return math.sqrt(x * x + y * y + (z - self.rh) ** 2)

    def vr_at(self, x: float) -> float:
        """True instantaneous radial speed (m/s, receding +) at distance x."""
        la = math.radians(self.tm_la)
        dr = math.radians(self.tm_dir_deg)
        d = x - self.x_tee
        t = d / (self.tm_ball_ms * math.cos(la) + 1e-9)
        pos = np.array([x, self.y_at(x), self.z_at(x) - self.rh])
        vel = self.tm_ball_ms * np.array(
            [math.cos(la) * math.cos(dr), math.cos(la) * math.sin(dr),
             math.sin(la)])
        vel[2] -= G * t
        return float(pos @ vel / (np.linalg.norm(pos) + 1e-9))

    def x_from_r(self, r: float) -> float:
        lo, hi = 0.8, 6.0
        for _ in range(48):
            mid = 0.5 * (lo + hi)
            if self.r_at(mid) < r:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def angles_at(self, x: float) -> tuple[float, float]:
        """(theta_direct, theta_image) rad, relative to boresight."""
        z, y = self.z_at(x), self.y_at(x)
        xh = math.hypot(x, y)
        tilt = math.radians(self.tilt_deg)
        th_d = math.atan2(z - self.rh, xh) - tilt
        th_i = math.atan2(-(z + self.rh), xh) - tilt
        return th_d, th_i


def load_shots() -> list[TruthShot]:
    """Aligned CSV + TM JSON (for LaunchDirection etc.) -> TruthShot list."""
    strokes = {}
    tm = json.loads(TM_JSON.read_text())
    for grp in tm["StrokeGroups"]:
        for s in grp["Strokes"]:
            m = s["Measurement"]
            strokes[round(m["LaunchAngle"], 6)] = m
    shots = []
    with open(CSV, encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if not row["ti_file"]:
                continue
            num = int(Path(row["ti_file"]).stem.split("_")[-1])
            blk = block_for(num)
            if blk is None:
                continue
            b = BLOCKS[blk]
            m = strokes.get(round(float(row["tm_la_deg"]), 6), {})
            shots.append(TruthShot(
                num=num, club=row["club"],
                file=DATA / Path(row["ti_file"]).name, block=blk,
                tee_slant=b["tee"], rh=b["rh"], tilt_deg=b["tilt"],
                tm_la=float(row["tm_la_deg"]),
                tm_ball_ms=float(row["tm_ball_mph"]) / MPH,
                tm_dir_deg=float(m.get("LaunchDirection", 0.0)),
                tm_club_mph=float(row["tm_club_mph"] or 0),
                tm_spin=float(row["tm_spin_rpm"] or 0),
                tm_attack=float(m.get("AttackAngle", 0.0)),
                of_ball_mph=float(row["of_ball_mph"])
                if row["of_ball_mph"] else None,
                live={k: row[k] for k in
                      ("ti_la_free", "ti_la_tee", "ti_la_two_ray",
                       "ti_ball_mph_fixed", "time_et")},
            ))
    return shots


def find_guided(mti: np.ndarray, geo: Geometry, tm_ms: float,
                max_range_m: float = 4.95, seed: int = 1) -> BallTrack | None:
    """RANSAC track constrained to a speed window around TM truth.

    Selection window only — the returned speed stays radar-measured.
    """
    power = tracking.loop_power(mti)
    li, bins = tracking._detections(power, geo, max_range_m=max_range_m)
    if li.size < 8:
        return None
    res = geo.range_res_m
    nl = geo.n_loops
    times = np.array([geo.loop_time(i // nl, i % nl) for i in li])
    # tight truth window: post-impact club (~0.72x ball) and flying tee
    # (~0.6x) fall below; the honest radial reading (cos factor >=0.93,
    # minus drag) stays inside
    lo, hi = 0.80 * tm_ms, 1.08 * tm_ms
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(3000):
        i, j = rng.choice(times.size, 2, replace=False)
        dt = times[i] - times[j]
        if abs(dt) < 3e-3:
            continue
        slope = (bins[i] - bins[j]) / dt
        if not lo <= slope * res <= hi:
            continue
        icpt = bins[i] - slope * times[i]
        inl = np.abs(bins - (slope * times + icpt)) < 1.2
        if inl.sum() < 8 or (best is not None and inl.sum() <= best[0]):
            continue
        dsn = np.vstack([times[inl], np.ones(inl.sum())]).T
        (s2, c2), *_ = np.linalg.lstsq(dsn, bins[inl], rcond=None)
        if not lo <= s2 * res <= hi:
            continue
        rms = float(np.sqrt(((bins[inl] - (s2 * times[inl] + c2)) ** 2)
                            .mean()))
        best = (int(inl.sum()), s2, c2, rms,
                float(times[inl].min()), float(times[inl].max()))
    if best is None:
        return None
    n, s, c, rms, t0, t1 = best
    trk = BallTrack(speed_ms=s * res, slope_bins=s, intercept_bins=c,
                    rms_bins=rms, n_inliers=n, t_first=t0, t_last=t1,
                    low_confidence=bool(rms >= 0.45 or (t1 - t0) < 0.012))
    # quadratic refit of the same inliers -> local radial speed v_r(t);
    # captures the real radial acceleration a straight line averages away
    inl = np.abs(bins - (s * times + c)) < 1.2
    quad = None
    if inl.sum() >= 10:
        q = np.polyfit(times[inl], bins[inl], 2)
        accel = 2 * q[0] * res
        if abs(accel) < 200.0:            # sanity: |radial accel| m/s^2
            quad = q
    trk.quad = quad                        # bins(t) = q0*t^2 + q1*t + q2
    return trk


def raw_snapshots(mti: np.ndarray, track: BallTrack, geo: Geometry,
                  range_bias_m: float, *, local_vr: bool = True
                  ) -> list[dict]:
    """Per-loop (K=1) UNCALIBRATED snapshots along the track, ungated.

    Vectors are post-flip, post-TDM-phase, so any element correction can be
    applied later by elementwise multiply. Stores snr for offline gating.
    ``local_vr`` uses the quadratic track's local slope for the per-snapshot
    TDM/loop Doppler phases (truth-free instantaneous velocity).
    """
    sign = doa.measure_tdm_sign(mti, track, geo)
    quad = getattr(track, "quad", None)
    res = geo.range_res_m
    noise = float(np.median(np.abs(mti) ** 2))

    def vr_at(t_s: float) -> float:
        if local_vr and quad is not None:
            return float((2 * quad[0] * t_s + quad[1]) * res)
        return track.speed_ms

    out = []
    for frame in range(geo.n_frames):
        for loop in range(geo.n_loops):
            t_s = geo.loop_time(frame, loop)
            if not track.t_first - 2e-3 <= t_s <= track.t_last + 2e-3:
                continue
            rbin = int(round(track.bin_at(t_s)))
            if not 2 <= rbin < geo.n_samples - 1:
                continue
            v_r = vr_at(t_s)
            tdm_phase = sign * 4 * np.pi * v_r * doa.TDM_TAU_S / LAM
            loop_phase = sign * 4 * np.pi * v_r * LOOP_PRI_S / LAM
            snap = np.concatenate([mti[frame, 0, loop, :, rbin],
                                   mti[frame, 1, loop, :, rbin]]).copy()
            snap[4:] *= np.exp(-1j * tdm_phase)
            snap = snap[::-1]                       # physical orientation
            snr = float((np.abs(snap) ** 2).mean() / noise)
            r_true = track.range_at(t_s, geo.range_res_m) - range_bias_m
            out.append(dict(t=t_s, frame=frame, loop=loop, rbin=rbin,
                            r=r_true, snr=snr, vec=snap, v_r=v_r,
                            loop_phase=loop_phase, sign=sign))
    return out


def load_mti(path: Path, window: str | None = None
             ) -> tuple[np.ndarray, Geometry]:
    raw = path.read_bytes()
    meta, cube = parse_dump(raw)
    geo = geometry_from_header(meta)
    fv = geo.chirps_per_frame * geo.n_rx * geo.n_samples
    got = cube.reshape(-1).size // fv
    if got < geo.n_frames:
        geo.n_frames = got
        cube = cube[:got]
    if window:
        win = getattr(np, window)(cube.shape[-1]).astype(cube.dtype
                                                         if cube.dtype.kind
                                                         == "f" else float)
        cube = cube * win[None, None, None, :]
    return tracking.mti_filter(cube), geo


def load_cal() -> Calibration:
    return Calibration.load(str(CAL_JSON))
