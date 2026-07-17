#!/usr/bin/env python3
"""Replay the frozen LCMF-v1 estimator on truth-free L3 dump sessions."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import fast_time_multipath_audit as fast_audit
import multipath_channel_audit as channel_audit
import numpy as np

from openflight.iwr6843 import Calibration, doa, process_dump, tracking
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.music import LAM
from openflight.iwr6843.shot import geometry_from_header

HERE = Path(__file__).parent
DEFAULT_CAL = Path(__file__).parents[3] / "config/iwr6843_cal_20260712.json"
FROZEN = HERE / "frozen_candidate_20260716.json"
MPH = 2.23694
CHANNEL_MODELS = ("two8", "four4_path_tdm")
FAST_MODELS = ("direct1", "two2", "four4")


def _csv_index(path: Path | None, value_field: str | None = None) -> dict[str, dict[str, str]]:
    if path is None:
        return {}
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    indexed: dict[str, dict[str, str]] = {}
    for row in rows:
        key = row.get("key") or row.get("file") or ""
        if key and (value_field is None or row.get(value_field)):
            indexed[key] = row
    return indexed


def _row_for(index: dict[str, dict[str, str]], key: str, filename: str) -> dict[str, str]:
    return index.get(key, index.get(filename, {}))


def _balanced_indices(cache: dict[str, np.ndarray]) -> np.ndarray:
    indices = np.nonzero((cache["snr"] >= 8.0) & (cache["r"] <= 4.70))[0]
    selected: list[int] = []
    for frame in np.unique(cache["frame"][indices]):
        frame_indices = indices[cache["frame"][indices] == frame]
        order = np.argsort(cache["snr"][frame_indices])[::-1]
        selected.extend(frame_indices[order[:4]])
    return np.asarray(selected, dtype=int)


def _snapshot_cache(raw: bytes, shot, cal: Calibration, tx_order: str, tdm_sign: int):
    meta, cube = parse_dump(raw)
    geo = geometry_from_header(meta)
    frame_values = geo.chirps_per_frame * geo.n_rx * geo.n_samples
    got_frames = cube.reshape(-1).size // frame_values
    if got_frames < geo.n_frames:
        geo.n_frames = got_frames
        cube = cube[:got_frames]
    scope = "window" if shot.notch_recovered else "burst"
    mti = tracking.mti_filter(cube, scope=scope)
    noise = float(np.median(np.abs(mti) ** 2))
    track = shot.track
    values: dict[str, list] = {
        "t": [],
        "frame": [],
        "loop": [],
        "r": [],
        "snr": [],
        "vr": [],
        "vec": [],
    }
    for frame in range(geo.n_frames):
        for loop in range(geo.n_loops):
            time_s = geo.loop_time(frame, loop)
            if not track.t_first - 2e-3 <= time_s <= track.t_last + 2e-3:
                continue
            rbin = int(round(track.bin_at(time_s)))
            if not 2 <= rbin < geo.n_samples - 1:
                continue
            velocity = track.speed_ms_at(time_s, geo.range_res_m)
            tdm_phase = tdm_sign * 4.0 * np.pi * velocity * doa.TDM_TAU_S / LAM
            uncalibrated = doa.canonicalize_tx_blocks(
                mti[frame, 0, loop, :, rbin],
                mti[frame, 1, loop, :, rbin],
                tdm_phase=tdm_phase,
                tx_order=tx_order,
            )
            values["t"].append(time_s)
            values["frame"].append(frame)
            values["loop"].append(loop)
            values["r"].append(cal.true_range(track.range_at(time_s, geo.range_res_m)))
            values["snr"].append(float(np.mean(np.abs(uncalibrated) ** 2) / noise))
            values["vr"].append(velocity)
            values["vec"].append(cal.apply(uncalibrated))
    return {name: np.asarray(data) for name, data in values.items()}, geo


def _channel_estimates(
    cache: dict[str, np.ndarray], indices: np.ndarray, rec: dict, grid_deg: np.ndarray
) -> dict[str, float]:
    frames = cache["frame"][indices]
    if len(indices) < 12 or len(np.unique(frames)) < 3:
        raise ValueError("insufficient channel snapshots")
    vectors = cache["vec"][indices]
    range_m = cache["r"][indices]
    estimates: dict[str, float] = {}
    for model in CHANNEL_MODELS:
        objective = np.asarray(
            [
                channel_audit._objective(
                    model,
                    np.radians(angle),
                    range_m,
                    vectors,
                    frames,
                    rec,
                )
                for angle in grid_deg
            ]
        )
        estimates[f"channel_{model}_deg"] = channel_audit._refine_grid(grid_deg, objective)
    return estimates


def _fast_estimates(
    path: Path,
    cache: dict[str, np.ndarray],
    indices: np.ndarray,
    rec: dict,
    cal: Calibration,
    grid_deg: np.ndarray,
    tx_order: str,
    tdm_sign: int,
) -> dict[str, float]:
    ordered = indices[np.argsort(cache["t"][indices])]
    indices = ordered[len(ordered) // 2 :]
    frames = cache["frame"][indices]
    if len(indices) < 6 or len(np.unique(frames)) < 2:
        raise ValueError("insufficient late-flight snapshots")
    n_fft = 512
    data_fft, geometry, window = fast_audit._prepared_fft(
        path,
        indices,
        cache,
        cal.elem_correction,
        n_fft=n_fft,
        window="hann",
        tx_order=tx_order,
        tdm_sign=tdm_sign,
    )
    oversample = n_fft / geometry.n_samples
    range_m = cache["r"][indices]
    track_bins = (range_m + cal.range_bias_m) / geometry.range_res_m * oversample
    centers = fast_audit._center_bins(
        data_fft,
        track_bins,
        center_source="peak",
        oversample=oversample,
    )
    offsets = np.arange(-int(np.ceil(2.0 * oversample)), int(np.ceil(6.0 * oversample)) + 1)
    local_data, selected_bins = fast_audit._local_data(data_fft, centers, offsets)
    center_native = centers / oversample
    estimates: dict[str, float] = {}
    for model in FAST_MODELS:
        objective = []
        for angle in grid_deg:
            design = fast_audit._design(
                model,
                np.radians(angle),
                range_m,
                center_native,
                selected_bins,
                rec,
                n_samples=geometry.n_samples,
                n_fft=n_fft,
                range_res_m=geometry.range_res_m,
                window=window,
                tdm_residual="none",
            )
            errors, _ = fast_audit._fit_error(local_data, design)
            objective.append(fast_audit._objective(errors, frames))
        estimates[f"fast_{model}_deg"] = channel_audit._refine_grid(grid_deg, np.asarray(objective))
    return estimates


def _configure_calibration(args) -> tuple[Calibration, float]:
    cal = Calibration.load(args.cal)
    cal.tee_range_m = args.tee_m
    cal.tilt_rad = np.radians(args.tilt_deg)
    cal.tee_ball_height_m = args.ball_height_m
    cal.meta["radar_height_m"] = args.radar_height_m
    dz = args.ball_height_m - args.radar_height_m
    x_tee = math.sqrt(max(args.tee_m**2 - dz**2, 0.25))
    return cal, x_tee


def _analyze(path: Path, group: str, tx_order: str, args, labels, ops_speeds, cal, x_tee):
    key = f"{group}/{path.name}"
    label_row = _row_for(labels, key, path.name)
    raw = path.read_bytes()
    shot = process_dump(
        raw,
        cal,
        club=args.club,
        net_range_m=args.net_m,
        tx_order=tx_order,
        tdm_sign_policy=args.tdm_sign,
    )
    row: dict[str, object] = {
        "key": key,
        "session": label_row.get("session", group),
        "file": path.name,
        "tx_order": tx_order,
        "label": label_row.get("label", ""),
        "status": "rejected_by_ball_tracker",
        "production_angle_deg": shot.launch_angle_deg,
        "tracker_quality": shot.quality,
        "track_rms_bins": shot.track.rms_bins if shot.track is not None else None,
    }
    if shot.track is None:
        return row
    speed_row = _row_for(ops_speeds, key, path.name)
    if speed_row.get("ops_ball_mph"):
        candidate_mph = float(speed_row["ops_ball_mph"])
        speed_source = "ops"
    else:
        candidate_mph = shot.track.speed_mph / 0.96
        speed_source = "ti_radial_div_0.96"
    row.update(
        {
            "ti_radial_speed_mph": shot.track.speed_mph,
            "candidate_ball_speed_mph": candidate_mph,
            "speed_source": speed_source,
        }
    )
    tdm_sign = 1 if args.tdm_sign == "positive" else -1
    cache, _geometry = _snapshot_cache(raw, shot, cal, tx_order, tdm_sign)
    indices = _balanced_indices(cache)
    rec = {
        "candidate_speed_ms": candidate_mph / MPH,
        "x_tee": x_tee,
        "z0": args.ball_height_m,
        "rh": args.radar_height_m,
        "tilt": args.tilt_deg,
        "tx_order": tx_order,
    }
    grid_deg = np.arange(-5.0, 45.0 + args.grid_step_deg / 2.0, args.grid_step_deg)
    try:
        components = _channel_estimates(cache, indices, rec, grid_deg)
        components.update(
            _fast_estimates(
                path,
                cache,
                indices,
                rec,
                cal,
                grid_deg,
                tx_order,
                tdm_sign,
            )
        )
    except (ValueError, np.linalg.LinAlgError) as error:
        row["status"] = str(error).replace(" ", "_")
        return row
    frozen = json.loads(FROZEN.read_text())
    raw_angle = float(np.mean(list(components.values())))
    row.update(components)
    row.update(
        {
            "n_balanced_snapshots": len(indices),
            "n_balanced_frames": len(np.unique(cache["frame"][indices])),
            "lcmf_v1_raw_deg": raw_angle,
            "lcmf_v1_angle_deg": raw_angle + float(frozen["angle_correction_deg"]),
            "status": (
                "accepted_track_quality_warning" if shot.quality == "reject" else "accepted"
            ),
        }
    )
    return row


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--normal", action="append", type=Path, default=[])
    parser.add_argument("--reversed", action="append", type=Path, default=[])
    parser.add_argument("--labels", type=Path)
    parser.add_argument("--ops-speeds", type=Path)
    parser.add_argument("--output", type=Path, default=Path("lcmf_v1_replay.csv"))
    parser.add_argument("--cal", type=Path, default=DEFAULT_CAL)
    parser.add_argument("--tee-m", type=float, default=1.575)
    parser.add_argument("--tilt-deg", type=float, default=10.40495)
    parser.add_argument("--radar-height-m", type=float, default=0.1524)
    parser.add_argument("--ball-height-m", type=float, default=0.040)
    parser.add_argument("--club", default="9i")
    parser.add_argument("--net-m", type=float, default=4.6)
    parser.add_argument("--tdm-sign", choices=("positive", "negative"), default="positive")
    parser.add_argument("--grid-step-deg", type=float, default=0.5)
    args = parser.parse_args()
    if not args.normal and not args.reversed:
        parser.error("provide at least one --normal or --reversed directory")

    labels = _csv_index(args.labels)
    ops_speeds = _csv_index(args.ops_speeds, "ops_ball_mph")
    cal, x_tee = _configure_calibration(args)
    rows: list[dict[str, object]] = []
    for tx_order, directories in (("normal", args.normal), ("reversed", args.reversed)):
        for directory in directories:
            for path in sorted(directory.glob("*.l3dump")):
                row = _analyze(
                    path,
                    directory.name,
                    tx_order,
                    args,
                    labels,
                    ops_speeds,
                    cal,
                    x_tee,
                )
                rows.append(row)
                angle = row.get("lcmf_v1_angle_deg")
                display = f"{float(angle):6.2f}" if angle is not None else "    --"
                print(
                    f"{row['session']:3s} {path.stem:13s} {tx_order:8s} "
                    f"{str(row['label']):7s} {display}  {row['status']}"
                )
    fields = list(dict.fromkeys(field for row in rows for field in row))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    accepted = sum(str(row["status"]).startswith("accepted") for row in rows)
    print(f"wrote {args.output} ({accepted}/{len(rows)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
