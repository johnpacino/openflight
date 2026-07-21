#!/usr/bin/env python3
"""Experimental IWR6843 club-path probe from L3 dumps.

This is intentionally an offline diagnostic, not production selection policy.
It asks one narrow question: do the 3TX captures contain a repeatable pre-impact
clubhead ridge whose horizontal angle changes over range like a club path?

The estimate is a proxy:
  1. Run the normal ball tracker to anchor approximate impact time.
  2. Search near the tee, just before impact, for strong moving returns.
  3. Use TX2 vs the TX1/TX3 vertical reference to estimate horizontal bearing.
  4. Fit lateral position versus downrange position; atan(dy/dx) is the path.

Positive follows TrackMan-style convention: rightward / in-to-out for a
right-handed golfer, assuming the radar is pointed down the target line.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from openflight.iwr6843 import doa
from openflight.iwr6843.calibration import DEFAULT_CAL_PATH, Calibration
from openflight.iwr6843.dump import parse_dump, project_tx_pair
from openflight.iwr6843.lcmf import _circular_median, _phase_to_angle_deg, _weighted_circular_mean
from openflight.iwr6843.shot import TX2_LOOP_PERIOD_S, TX2_VERTICAL_TDM_TAU_S, process_dump
from openflight.iwr6843.tracking import Geometry

MPH_PER_MS = 2.23694


@dataclass(frozen=True)
class CaptureRow:
    """One IWR capture joined with any OPS shot fields found in the session."""

    shot_number: int
    capture_path: Path
    ball_speed_mph: float | None
    club_speed_mph: float | None
    launch_v_deg: float | None
    launch_h_deg: float | None


@dataclass(frozen=True)
class ClubPoint:
    """One candidate clubhead point near impact."""

    t_s: float
    range_m: float
    range_bin: float
    horizontal_deg: float
    weight: float
    snr: float


def _float_or_none(value: Any) -> float | None:
    try:
        if value is None or value == "":
            return None
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _load_session(path: Path) -> list[CaptureRow]:
    """Load iwr6843_capture rows and join shot_detected by shot number."""
    shots: dict[int, dict[str, Any]] = {}
    captures: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            if not line.strip():
                continue
            entry = json.loads(line)
            event = entry.get("event") or entry.get("type")
            data = entry.get("data", entry)
            if event == "shot_detected" and data.get("shot_number") is not None:
                shots[int(data["shot_number"])] = data
            elif event == "iwr6843_capture" and data.get("capture_path"):
                captures.append(data)

    rows: list[CaptureRow] = []
    for cap in captures:
        number = int(cap["shot_number"])
        shot = shots.get(number, {})
        measurement = cap.get("measurement") or {}
        rows.append(
            CaptureRow(
                shot_number=number,
                capture_path=Path(cap["capture_path"]).expanduser(),
                ball_speed_mph=_float_or_none(cap.get("ball_speed_mph"))
                or _float_or_none(shot.get("ball_speed_mph")),
                club_speed_mph=_float_or_none(shot.get("club_speed_mph")),
                launch_v_deg=_float_or_none(shot.get("launch_angle_vertical"))
                or _float_or_none(measurement.get("launch_angle_deg")),
                launch_h_deg=_float_or_none(shot.get("launch_angle_horizontal"))
                or _float_or_none(measurement.get("horizontal_deg")),
            )
        )
    return rows


def _subbin_peak(power: np.ndarray, idx: int, gate_lo: int, gate_hi: int) -> float:
    """Parabolic sub-bin interpolation around a local peak."""
    peak = float(idx)
    if gate_lo < idx < gate_hi - 1:
        y0, y1, y2 = power[idx - 1], power[idx], power[idx + 1]
        den = y0 - 2.0 * y1 + y2
        if den < 0:
            off = (y0 - y2) / (2.0 * den)
            if abs(off) < 1.0:
                peak += float(off)
    return peak


def _ransac_line(
    times: np.ndarray,
    ranges: np.ndarray,
    weights: np.ndarray,
    *,
    iterations: int = 800,
    seed: int = 7,
    tol_m: float = 0.08,
    min_speed_ms: float = -25.0,
    max_speed_ms: float = 65.0,
) -> tuple[float, float, np.ndarray, float] | None:
    """Robust range-vs-time fit for noisy club detections."""
    if len(times) < 5:
        return None
    rng = np.random.default_rng(seed)
    best: tuple[float, int, float, float, np.ndarray] | None = None
    for _ in range(iterations):
        i, j = rng.choice(len(times), 2, replace=False)
        dt = times[i] - times[j]
        if abs(dt) < 1e-4:
            continue
        slope = (ranges[i] - ranges[j]) / dt
        if not min_speed_ms <= slope <= max_speed_ms:
            continue
        intercept = ranges[i] - slope * times[i]
        residual = np.abs(ranges - (slope * times + intercept))
        inliers = residual < tol_m
        count = int(inliers.sum())
        if count < 5:
            continue
        score = float(weights[inliers].sum())
        if best is None or score > best[0]:
            best = (score, count, slope, intercept, inliers)
    if best is None:
        return None
    _score, _count, slope, intercept, inliers = best
    design = np.vstack([times[inliers], np.ones(int(inliers.sum()))]).T
    coeff, *_ = np.linalg.lstsq(design, ranges[inliers], rcond=None)
    slope, intercept = float(coeff[0]), float(coeff[1])
    residual = ranges[inliers] - (slope * times[inliers] + intercept)
    rms = float(np.sqrt(np.mean(residual**2)))
    return slope, intercept, inliers, rms


def _tx2_horizontal_at(
    mti: np.ndarray,
    *,
    frame: int,
    loop: int,
    range_bin: int,
    velocity_ms: float,
    tdm_sign: int,
) -> tuple[float | None, float | None]:
    """TX2 horizontal bearing proxy for one range/time snapshot."""
    tx1 = mti[frame, loop, 0, :, range_bin]
    tx2 = mti[frame, loop, 1, :, range_bin] * np.exp(
        -1j * tdm_sign * 4.0 * np.pi * velocity_ms * doa.TDM_TAU_S / doa.LAM
    )
    tx3 = mti[frame, loop, 2, :, range_bin] * np.exp(
        -1j * tdm_sign * 4.0 * np.pi * velocity_ms * TX2_VERTICAL_TDM_TAU_S / doa.LAM
    )
    reference = 0.5 * (tx1 + tx3)
    phases = [
        float(np.angle(np.conj(reference[rx]) * tx2[rx]))
        for rx in range(reference.size)
        if abs(reference[rx]) * abs(tx2[rx]) > 0
    ]
    if not phases:
        return None, None
    phase = _circular_median(phases)
    # Same sign convention as HLCMF-v0: positive means starts/moves right.
    return -_phase_to_angle_deg(phase), float(abs(np.mean(np.exp(1j * np.asarray(phases)))))


def _geometry(meta: dict) -> Geometry:
    return Geometry(
        n_frames=meta["n_frames"],
        chirps_per_frame=meta["chirps_per_frame"],
        n_tx=meta["n_tx"],
        n_rx=meta["n_rx"],
        n_samples=meta["n_samples"],
        frame_period_s=(meta.get("frame_period_us", 0) / 1e6) or 0.012,
        trigger_frame=meta["trigger_frame"],
        loop_period_s=TX2_LOOP_PERIOD_S,
    )


def estimate_club_path(
    raw: bytes,
    cal: Calibration,
    *,
    tee_m: float,
    net_m: float | None,
    club: str | None,
    tx_order: str,
    pre_ms: float,
    post_ms: float,
    gate_half_m: float,
    snr_min: float,
) -> dict[str, Any]:
    """Return club-path proxy and evidence fields for one dump."""
    ball_raw = project_tx_pair(raw, (0, 2))
    shot = process_dump(
        ball_raw,
        cal,
        net_range_m=net_m,
        club=club,
        tx_order=tx_order,
        tdm_sign_policy="positive",
        loop_period_s=TX2_LOOP_PERIOD_S,
        tdm_tau_s=TX2_VERTICAL_TDM_TAU_S,
    )
    if shot.track is None:
        return {"status": "no_ball_track"}

    meta, cube = parse_dump(raw)
    if meta["n_tx"] != 3:
        return {"status": f"needs_3tx_got_{meta['n_tx']}"}
    geo = _geometry(meta)
    n_frames, chirps_per_frame, n_rx, n_samples = cube.shape
    loops = chirps_per_frame // meta["n_tx"]
    tdm = cube.reshape(n_frames, loops, meta["n_tx"], n_rx, n_samples)
    rfft = np.fft.fft(tdm, axis=-1)
    mti = rfft - rfft.mean(axis=1, keepdims=True)

    ball_first_range_m = shot.track.range_at(shot.track.t_first, geo.range_res_m)
    impact_t_s = shot.track.t_first - ((ball_first_range_m - tee_m) / shot.track.speed_ms)
    window_lo = impact_t_s - pre_ms / 1000.0
    window_hi = impact_t_s + post_ms / 1000.0

    gate_lo = max(2, int((tee_m - gate_half_m) / geo.range_res_m))
    gate_hi = min(n_samples // 2 - 2, int((tee_m + gate_half_m) / geo.range_res_m))
    if gate_hi <= gate_lo + 3:
        return {"status": "club_gate_empty"}

    raw_points: list[tuple[int, int, float, float, float, float, float]] = []
    for frame in range(geo.n_frames):
        for loop in range(geo.n_loops):
            t_s = geo.loop_time(frame, loop)
            if not window_lo <= t_s <= window_hi:
                continue
            power = np.sum(np.abs(mti[frame, loop, :, :, :]) ** 2, axis=(0, 1))
            gate = power[gate_lo:gate_hi]
            baseline = float(np.median(gate) + 1e-12)
            rel_idx = int(np.argmax(gate))
            snr = float(gate[rel_idx] / baseline)
            if snr < snr_min:
                continue
            peak_bin = _subbin_peak(power, gate_lo + rel_idx, gate_lo, gate_hi)
            raw_points.append(
                (
                    frame,
                    loop,
                    t_s,
                    cal.true_range(peak_bin * geo.range_res_m),
                    peak_bin,
                    float(gate[rel_idx]),
                    snr,
                )
            )

    if len(raw_points) < 5:
        return {
            "status": "insufficient_club_candidates",
            "ball_speed_mph": shot.track.speed_mph,
            "impact_time_ms": impact_t_s * 1000.0,
            "candidate_count": len(raw_points),
        }

    times = np.asarray([p[2] for p in raw_points])
    ranges = np.asarray([p[3] for p in raw_points])
    weights = np.asarray([p[5] for p in raw_points])
    fit = _ransac_line(times, ranges, weights)
    if fit is None:
        return {
            "status": "club_range_fit_failed",
            "ball_speed_mph": shot.track.speed_mph,
            "impact_time_ms": impact_t_s * 1000.0,
            "candidate_count": len(raw_points),
        }
    radial_speed_ms, range_intercept_m, inliers, range_rms_m = fit
    _ = range_intercept_m

    points: list[ClubPoint] = []
    for inlier, raw_point in zip(inliers, raw_points, strict=True):
        if not inlier:
            continue
        frame, loop, t_s, range_m, peak_bin, weight, snr = raw_point
        range_bin = int(round(peak_bin))
        if not 2 <= range_bin < n_samples - 2:
            continue
        h_deg, coherence = _tx2_horizontal_at(
            mti,
            frame=frame,
            loop=loop,
            range_bin=range_bin,
            velocity_ms=radial_speed_ms,
            tdm_sign=+1,
        )
        if h_deg is None or coherence is None or coherence < 0.15:
            continue
        points.append(
            ClubPoint(
                t_s=t_s,
                range_m=range_m,
                range_bin=peak_bin,
                horizontal_deg=h_deg,
                weight=weight,
                snr=snr,
            )
        )

    if len(points) < 5:
        return {
            "status": "insufficient_horizontal_points",
            "ball_speed_mph": shot.track.speed_mph,
            "impact_time_ms": impact_t_s * 1000.0,
            "candidate_count": len(raw_points),
            "range_inliers": int(inliers.sum()),
            "range_fit_rms_m": range_rms_m,
            "club_radial_mph": radial_speed_ms * MPH_PER_MS,
        }

    x = np.asarray([p.range_m for p in points])
    h = np.radians([p.horizontal_deg for p in points])
    y = x * np.tan(h)
    point_weights = np.asarray([max(1.0, p.snr) for p in points])
    design = np.vstack([x, np.ones_like(x)]).T
    sqrt_w = np.sqrt(point_weights)
    coeff, *_ = np.linalg.lstsq(design * sqrt_w[:, None], y * sqrt_w, rcond=None)
    path_deg = float(np.degrees(np.arctan(coeff[0])))
    y_fit = design @ coeff
    path_rms_m = float(np.sqrt(np.average((y - y_fit) ** 2, weights=point_weights)))
    phase_rad, coherence = _weighted_circular_mean(
        [math.radians(p.horizontal_deg) for p in points],
        [p.weight for p in points],
    )

    return {
        "status": "accepted" if path_rms_m <= 0.20 else "high_path_rms",
        "ball_speed_mph": shot.track.speed_mph,
        "ball_track_rms_bins": shot.track.rms_bins,
        "impact_time_ms": impact_t_s * 1000.0,
        "candidate_count": len(raw_points),
        "range_inliers": int(inliers.sum()),
        "horizontal_points": len(points),
        "club_radial_mph": radial_speed_ms * MPH_PER_MS,
        "range_span_m": float(np.max(x) - np.min(x)),
        "range_fit_rms_m": range_rms_m,
        "club_path_deg": path_deg,
        "club_h_median_deg": float(np.median([p.horizontal_deg for p in points])),
        "club_h_mean_deg": float(np.average([p.horizontal_deg for p in points], weights=point_weights)),
        "club_h_coherence": coherence,
        "path_fit_rms_m": path_rms_m,
        "phase_mean_rad": phase_rad,
    }


def _format(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.3f}"
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--session", type=Path, help="OpenFlight session JSONL with iwr6843_capture rows")
    source.add_argument("--dump", type=Path, help="Single .l3dump file")
    parser.add_argument("--cal", default=DEFAULT_CAL_PATH, help="IWR6843 calibration JSON")
    parser.add_argument("--tee-m", type=float, required=True, help="Radar-to-ball slant range in meters")
    parser.add_argument("--net-m", type=float, default=None, help="Radar-to-net range in meters")
    parser.add_argument("--club", default="9i", help="Club label for ball tracker speed floor")
    parser.add_argument("--tx-order", default="normal", choices=sorted(doa.TX_ORDERS))
    parser.add_argument("--pre-ms", type=float, default=28.0, help="Club search window before estimated impact")
    parser.add_argument("--post-ms", type=float, default=5.0, help="Club search window after estimated impact")
    parser.add_argument("--gate-half-m", type=float, default=0.65, help="Search tee +/- this range")
    parser.add_argument("--snr-min", type=float, default=8.0, help="Candidate club peak gate SNR")
    parser.add_argument("--out", type=Path, default=None, help="Optional CSV output path")
    args = parser.parse_args()

    cal = Calibration.load(args.cal)
    if args.dump:
        rows = [
            CaptureRow(
                shot_number=1,
                capture_path=args.dump.expanduser(),
                ball_speed_mph=None,
                club_speed_mph=None,
                launch_v_deg=None,
                launch_h_deg=None,
            )
        ]
    else:
        rows = _load_session(args.session.expanduser())

    output_rows: list[dict[str, Any]] = []
    for row in rows:
        try:
            raw = row.capture_path.read_bytes()
            result = estimate_club_path(
                raw,
                cal,
                tee_m=args.tee_m,
                net_m=args.net_m,
                club=args.club,
                tx_order=args.tx_order,
                pre_ms=args.pre_ms,
                post_ms=args.post_ms,
                gate_half_m=args.gate_half_m,
                snr_min=args.snr_min,
            )
        except FileNotFoundError:
            result = {"status": "missing_dump"}
        except Exception as exc:  # pragma: no cover - diagnostic script should keep going
            result = {"status": f"error:{type(exc).__name__}", "error": str(exc)}
        output_rows.append(
            {
                "shot": row.shot_number,
                "dump": str(row.capture_path),
                "ops_ball_mph": row.ball_speed_mph,
                "ops_club_mph": row.club_speed_mph,
                "live_launch_v_deg": row.launch_v_deg,
                "live_launch_h_deg": row.launch_h_deg,
                **result,
            }
        )

    fieldnames = [
        "shot",
        "status",
        "ops_ball_mph",
        "ops_club_mph",
        "ball_speed_mph",
        "club_radial_mph",
        "club_path_deg",
        "club_h_median_deg",
        "club_h_mean_deg",
        "club_h_coherence",
        "horizontal_points",
        "range_inliers",
        "candidate_count",
        "range_span_m",
        "range_fit_rms_m",
        "path_fit_rms_m",
        "ball_track_rms_bins",
        "impact_time_ms",
        "live_launch_v_deg",
        "live_launch_h_deg",
        "dump",
        "error",
    ]
    destination = args.out.open("w", newline="", encoding="utf-8") if args.out else sys.stdout
    try:
        writer = csv.DictWriter(destination, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for output in output_rows:
            writer.writerow({key: _format(output.get(key)) for key in fieldnames})
    finally:
        if args.out:
            destination.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
