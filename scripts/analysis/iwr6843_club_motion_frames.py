#!/usr/bin/env python3
"""Inspect early-frame club motion from 3TX IWR6843 L3 dumps.

This is a microscope, not an estimator. It summarizes the first few frames of
each capture so we can ask whether the clubhead-like reflector moves downrange
and laterally through impact.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

from openflight.iwr6843 import doa
from openflight.iwr6843.calibration import DEFAULT_CAL_PATH, Calibration
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.lcmf import _circular_median, _phase_to_angle_deg
from openflight.iwr6843.music import est_bartlett, est_music_fbss_high
from openflight.iwr6843.shot import TX2_LOOP_PERIOD_S, TX2_VERTICAL_TDM_TAU_S
from openflight.iwr6843.tracking import Geometry


def _float_or_none(value: Any) -> float | None:
    try:
        if value in (None, ""):
            return None
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _load_captures(path: Path) -> list[dict[str, Any]]:
    shots: dict[int, dict[str, Any]] = {}
    captures: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            if not line.strip():
                continue
            entry = json.loads(line)
            event = entry.get("event") or entry.get("type")
            if event == "shot_detected" and entry.get("shot_number") is not None:
                shots[int(entry["shot_number"])] = entry
            elif event == "iwr6843_capture" and entry.get("capture_path"):
                captures.append(entry)
    for capture in captures:
        shot = shots.get(int(capture["shot_number"]), {})
        measurement = capture.get("measurement") or {}
        capture["ops_club_mph"] = _float_or_none(shot.get("club_speed_mph"))
        capture["ops_ball_mph"] = _float_or_none(shot.get("ball_speed_mph"))
        capture["live_h_deg"] = _float_or_none(shot.get("launch_angle_horizontal")) or _float_or_none(
            measurement.get("horizontal_deg")
        )
    return captures


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


def _horizontal_at(mti: np.ndarray, frame: int, loop: int, rbin: int, velocity_ms: float) -> float | None:
    tx1 = mti[frame, loop, 0, :, rbin]
    tx2 = mti[frame, loop, 1, :, rbin] * np.exp(
        -1j * 4.0 * np.pi * velocity_ms * doa.TDM_TAU_S / doa.LAM
    )
    tx3 = mti[frame, loop, 2, :, rbin] * np.exp(
        -1j * 4.0 * np.pi * velocity_ms * TX2_VERTICAL_TDM_TAU_S / doa.LAM
    )
    reference = 0.5 * (tx1 + tx3)
    phases = [
        float(np.angle(np.conj(reference[rx]) * tx2[rx]))
        for rx in range(reference.size)
        if abs(reference[rx]) * abs(tx2[rx]) > 0
    ]
    if not phases:
        return None
    return -_phase_to_angle_deg(_circular_median(phases))


def _vertical_at(
    mti: np.ndarray,
    cal: Calibration,
    frame: int,
    loop: int,
    rbin: int,
    velocity_ms: float,
) -> float | None:
    """Calibrated vertical bearing in ground coordinates for TX1/TX3."""
    tx1 = mti[frame, loop, 0, :, rbin]
    tx3 = mti[frame, loop, 2, :, rbin] * np.exp(
        -1j * 4.0 * np.pi * velocity_ms * TX2_VERTICAL_TDM_TAU_S / doa.LAM
    )
    snapshot = np.concatenate((tx1, tx3), axis=0)[::-1]
    corrected = cal.apply(snapshot)
    noise = float(np.median(np.abs(mti) ** 2))
    theta = est_music_fbss_high(corrected, noise)
    if abs(theta - est_bartlett(corrected)) > math.radians(12.0):
        return None
    return float(math.degrees(theta + cal.tilt_rad))


def _frame_points(
    raw: bytes,
    cal: Calibration,
    *,
    tee_m: float,
    frame_count: int,
    gate_half_m: float,
    snr_min: float,
) -> list[dict[str, Any]]:
    meta, cube = parse_dump(raw)
    if meta["n_tx"] != 3:
        raise ValueError(f"needs 3TX dump, got {meta['n_tx']}TX")
    geo = _geometry(meta)
    n_frames, chirps_per_frame, n_rx, n_samples = cube.shape
    loops = chirps_per_frame // meta["n_tx"]
    tdm = cube.reshape(n_frames, loops, meta["n_tx"], n_rx, n_samples)
    rfft = np.fft.fft(tdm, axis=-1)
    mti = rfft - rfft.mean(axis=1, keepdims=True)

    lo = max(2, int((tee_m - gate_half_m) / geo.range_res_m))
    hi = min(n_samples // 2 - 2, int((tee_m + gate_half_m) / geo.range_res_m))
    rows: list[dict[str, Any]] = []
    ordered_frames = sorted(range(geo.n_frames), key=lambda frame: geo.loop_time(frame, 0))
    for frame in ordered_frames[:frame_count]:
        points: list[tuple[float, float, float, float, float | None]] = []
        for loop in range(geo.n_loops):
            power = np.sum(np.abs(mti[frame, loop, :, :, :]) ** 2, axis=(0, 1))
            gate = power[lo:hi]
            baseline = float(np.median(gate) + 1e-12)
            rel_idx = int(np.argmax(gate))
            snr = float(gate[rel_idx] / baseline)
            if snr < snr_min:
                continue
            rbin = lo + rel_idx
            range_m = cal.true_range(rbin * geo.range_res_m)
            h_deg = _horizontal_at(mti, frame, loop, rbin, velocity_ms=0.0)
            if h_deg is None:
                continue
            v_deg = _vertical_at(mti, cal, frame, loop, rbin, velocity_ms=0.0)
            points.append((geo.loop_time(frame, loop) * 1000.0, range_m, h_deg, snr, v_deg))
        if not points:
            rows.append({"frame": frame, "n": 0})
            continue
        times = np.asarray([p[0] for p in points])
        ranges = np.asarray([p[1] for p in points])
        h = np.asarray([p[2] for p in points])
        snrs = np.asarray([p[3] for p in points])
        v_values = np.asarray([p[4] for p in points if p[4] is not None])
        rows.append(
            {
                "frame": frame,
                "n": len(points),
                "time_ms": float(np.median(times)),
                "range_m": float(np.median(ranges)),
                "h_deg": float(np.median(h)),
                "v_deg": float(np.median(v_values)) if v_values.size else None,
                "h_first_deg": float(h[0]),
                "h_last_deg": float(h[-1]),
                "range_first_m": float(ranges[0]),
                "range_last_m": float(ranges[-1]),
                "snr_med": float(np.median(snrs)),
            }
        )
    return rows


def _fmt(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return "" if not math.isfinite(value) else f"{value:.3f}"
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--cal", default=DEFAULT_CAL_PATH)
    parser.add_argument("--tee-m", type=float, required=True)
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--gate-half-m", type=float, default=0.65)
    parser.add_argument("--snr-min", type=float, default=8.0)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    cal = Calibration.load(args.cal)
    captures = _load_captures(args.session.expanduser())
    output: list[dict[str, Any]] = []
    for capture in captures:
        try:
            frame_rows = _frame_points(
                Path(capture["capture_path"]).read_bytes(),
                cal,
                tee_m=args.tee_m,
                frame_count=args.frames,
                gate_half_m=args.gate_half_m,
                snr_min=args.snr_min,
            )
        except Exception as exc:  # pragma: no cover - keep diagnostics going
            frame_rows = [{"frame": "", "n": 0, "error": f"{type(exc).__name__}: {exc}"}]
        previous = None
        for row in frame_rows:
            if previous and row.get("n", 0) and previous.get("n", 0):
                row["delta_range_m"] = row["range_m"] - previous["range_m"]
                row["delta_h_deg"] = row["h_deg"] - previous["h_deg"]
                if row.get("v_deg") is not None and previous.get("v_deg") is not None:
                    row["delta_v_deg"] = row["v_deg"] - previous["v_deg"]
            previous = row
            output.append(
                {
                    "shot": capture["shot_number"],
                    "ops_ball_mph": capture.get("ops_ball_mph"),
                    "ops_club_mph": capture.get("ops_club_mph"),
                    "live_h_deg": capture.get("live_h_deg"),
                    **row,
                }
            )

    fields = [
        "shot",
        "frame",
        "n",
        "time_ms",
        "range_m",
        "h_deg",
        "v_deg",
        "delta_range_m",
        "delta_h_deg",
        "delta_v_deg",
        "h_first_deg",
        "h_last_deg",
        "range_first_m",
        "range_last_m",
        "snr_med",
        "ops_ball_mph",
        "ops_club_mph",
        "live_h_deg",
        "error",
    ]
    target = args.out.open("w", newline="", encoding="utf-8") if args.out else sys.stdout
    try:
        writer = csv.DictWriter(target, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in output:
            writer.writerow({field: _fmt(row.get(field)) for field in fields})
    finally:
        if args.out:
            target.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
