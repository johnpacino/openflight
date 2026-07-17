#!/usr/bin/env python3
"""Truth-free comparison of normal and reversed IWR6843 TX chirp order.

The output deliberately does not treat launch-angle similarity as accuracy.
Use manual strike labels to separate skulls, normal strikes, high strikes, and
obvious mishits; TrackMan truth can be joined to the per-shot CSV later.
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

from openflight.iwr6843 import Calibration, doa, process_dump, tracking, trajectory
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.music import est_bartlett, steer
from openflight.iwr6843.shot import TH_GATE_RAD, geometry_from_header

DEFAULT_CAL = Path(__file__).parents[2] / "config/iwr6843_cal_20260712.json"
GRID_M = np.arange(-0.02, 1.30, 0.01)


def _finite(value: float | None) -> float:
    return float(value) if value is not None and np.isfinite(value) else math.nan


def _percentile(values: list[float], q: float) -> float:
    clean = [value for value in values if np.isfinite(value)]
    return float(np.percentile(clean, q)) if clean else math.nan


def _circular_stats(angles: list[float]) -> tuple[float, float]:
    """Return absolute circular mean and circular scatter, in degrees."""
    if not angles:
        return math.nan, math.nan
    z = np.mean(np.exp(1j * np.asarray(angles)))
    mean_abs = abs(float(np.degrees(np.angle(z))))
    scatter = np.degrees(np.sqrt(max(0.0, -2.0 * np.log(max(abs(z), 1e-9)))))
    return mean_abs, float(scatter)


def _snapshot_model(snap: np.ndarray, x_m: float, cal: Calibration) -> tuple[float, float, float]:
    """Return two-ray explained fraction, image fraction, and TX seam."""
    radar_height_m = cal.radar_height_m
    height, explained, image_fraction = trajectory._two_ray_solve(
        snap, x_m, radar_height_m, cal.tilt_rad, GRID_M
    )
    theta_direct = np.arctan2(height - radar_height_m, x_m) - cal.tilt_rad
    theta_image = np.arctan2(-(height + radar_height_m), x_m) - cal.tilt_rad
    model = np.column_stack((steer(theta_direct, 8), steer(theta_image, 8)))
    coefficients, *_ = np.linalg.lstsq(model, snap, rcond=None)
    prediction = model @ coefficients
    good = np.abs(prediction) > 0.15 * np.abs(prediction).max()
    ratio = np.where(good, snap / np.where(good, prediction, 1.0), 0.0)
    weight = np.where(good, np.abs(prediction) ** 2, 0.0)
    first = np.sum(weight[:4] * ratio[:4])
    second = np.sum(weight[4:] * ratio[4:])
    seam = float(np.angle(first * np.conj(second))) if first and second else math.nan
    return explained, image_fraction, seam


def _split_agreement(snaps: list[tuple[float, float, np.ndarray]], cal: Calibration) -> float:
    """Absolute launch-angle difference between early and late snapshots."""
    if len(snaps) < 12:
        return math.nan
    ordered = sorted(snaps, key=lambda item: item[0])
    middle = len(ordered) // 2
    fits = [
        trajectory.fit_two_ray(
            part,
            cal,
            min_points=3,
            min_explained=0.70,
            weighted=True,
            anchor_tee=True,
            th_gate_rad=TH_GATE_RAD,
        )
        for part in (ordered[:middle], ordered[middle:])
    ]
    if any(fit is None for fit in fits):
        return math.nan
    return abs(fits[0].launch_angle_deg - fits[1].launch_angle_deg)


def _labels(path: Path | None) -> dict[str, tuple[str, str]]:
    if path is None or not path.exists():
        return {}
    out: dict[str, tuple[str, str]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            key = row.get("key") or row.get("file") or ""
            if key:
                out[key] = (row.get("label", ""), row.get("notes", ""))
    return out


def analyze(
    path: Path,
    group: str,
    tx_order: str,
    cal: Calibration,
    club: str,
    net_m: float | None,
    tdm_sign_policy: str,
    labels: dict[str, tuple[str, str]],
) -> dict[str, object]:
    key = f"{group}/{path.name}"
    label, notes = labels.get(key, labels.get(path.name, ("", "")))
    raw = path.read_bytes()
    shot = process_dump(
        raw,
        cal,
        club=club,
        net_range_m=net_m,
        tx_order=tx_order,
        tdm_sign_policy=tdm_sign_policy,
    )
    row: dict[str, object] = {
        "key": key,
        "group": group,
        "file": path.name,
        "tx_order": tx_order,
        "tdm_sign_policy": shot.tdm_sign_policy,
        "tdm_sign_used": shot.tdm_sign_used,
        "label": label,
        "notes": notes,
        "ball_found": int(shot.ball_found),
        "quality": shot.quality,
        "policy": shot.policy,
        "notch_recovered": int(shot.notch_recovered),
        "ball_speed_mph": _finite(shot.ball_speed_mph),
        "launch_angle_deg": _finite(shot.launch_angle_deg),
        "angle_points": shot.n_angle_points,
    }
    for name in ("free", "tee", "two_ray"):
        fit = shot.fits.get(name)
        row[f"{name}_deg"] = _finite(fit.launch_angle_deg if fit else None)
        row[f"{name}_rms_m"] = _finite(fit.h_rms_m if fit else None)
    fit_angles = [float(row[f"{name}_deg"]) for name in ("free", "tee", "two_ray")]
    finite_angles = [value for value in fit_angles if np.isfinite(value)]
    row["estimator_spread_deg"] = (
        max(finite_angles) - min(finite_angles) if len(finite_angles) >= 2 else math.nan
    )
    if shot.track is None:
        return row
    row.update(
        {
            "track_inliers": shot.track.n_inliers,
            "track_rms_bins": shot.track.rms_bins,
            "track_span_ms": 1000.0 * (shot.track.t_last - shot.track.t_first),
        }
    )
    if shot.quality == "reject":
        return row

    meta, cube = parse_dump(raw)
    geo = geometry_from_header(meta)
    mti = tracking.mti_filter(cube, scope="window" if shot.notch_recovered else "burst")
    series = doa.snapshot_series(
        mti,
        shot.track,
        geo,
        cal,
        coherent_loops=1,
        tx_order=tx_order,
        tdm_sign=shot.tdm_sign_used,
    )
    snaps = [(time_s, range_m, snap) for time_s, range_m, snap, _ in series]
    fit_kwargs = {"min_points": 3, "min_explained": 0.70, "weighted": True, "anchor_tee": True}
    gated = trajectory.fit_two_ray(snaps, cal, th_gate_rad=TH_GATE_RAD, **fit_kwargs)
    ungated = trajectory.fit_two_ray(snaps, cal, **fit_kwargs)
    gated_deg = _finite(gated.launch_angle_deg if gated else None)
    ungated_deg = _finite(ungated.launch_angle_deg if ungated else None)
    bartlett = [float(est_bartlett(snap)) for _, _, snap, _ in series]
    models = [_snapshot_model(snap, range_m, cal) for _, range_m, snap, _ in series]
    explained = [model[0] for model in models]
    image_fraction = [model[1] for model in models]
    seams = [model[2] for model in models if np.isfinite(model[2])]
    seam_abs, seam_scatter = _circular_stats(seams)
    row.update(
        {
            "snapshots": len(series),
            "bartlett_median_deg": _percentile([np.degrees(x) for x in bartlett], 50),
            "bartlett_iqr_deg": (
                _percentile([np.degrees(x) for x in bartlett], 75)
                - _percentile([np.degrees(x) for x in bartlett], 25)
            ),
            "corrupt_zone_pct": (
                100.0 * np.mean(np.asarray(bartlett) > TH_GATE_RAD) if bartlett else math.nan
            ),
            "explained_median": _percentile(explained, 50),
            "explained_p10": _percentile(explained, 10),
            "model_pass_pct": (
                100.0 * np.mean(np.asarray(explained) >= 0.70) if explained else math.nan
            ),
            "image_fraction_median": _percentile(image_fraction, 50),
            "tx_seam_abs_deg": seam_abs,
            "tx_seam_scatter_deg": seam_scatter,
            "early_late_delta_deg": _split_agreement(snaps, cal),
            "two_ray_gated_deg": gated_deg,
            "two_ray_ungated_deg": ungated_deg,
            "gate_delta_deg": (
                gated_deg - ungated_deg
                if np.isfinite(gated_deg) and np.isfinite(ungated_deg)
                else math.nan
            ),
        }
    )
    return row


def _median(rows: list[dict[str, object]], field: str) -> float:
    values = [float(row.get(field, math.nan)) for row in rows]
    return _percentile(values, 50)


def print_summary(rows: list[dict[str, object]]) -> None:
    groups: dict[tuple[str, str], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        groups[(str(row["tx_order"]), str(row["label"]) or "unlabeled")].append(row)
    print("\nTruth-free comparison (lower is better for seam/delta/spread):")
    print("order     label       n read% expl  pass% seam scatter early/late spread")
    for (order, label), group in sorted(groups.items()):
        read_pct = (
            100.0
            * sum(row["quality"] != "reject" and row["ball_found"] for row in group)
            / len(group)
        )
        print(
            f"{order:9s} {label[:10]:10s} {len(group):2d} {read_pct:5.1f} "
            f"{_median(group, 'explained_median'):5.3f} "
            f"{_median(group, 'model_pass_pct'):5.1f} "
            f"{_median(group, 'tx_seam_abs_deg'):5.1f} "
            f"{_median(group, 'tx_seam_scatter_deg'):7.1f} "
            f"{_median(group, 'early_late_delta_deg'):10.2f} "
            f"{_median(group, 'estimator_spread_deg'):6.2f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--normal",
        action="append",
        type=Path,
        default=[],
        help="normal-order dump directory (repeatable)",
    )
    parser.add_argument(
        "--reversed",
        action="append",
        type=Path,
        default=[],
        help="reversed-order dump directory (repeatable)",
    )
    parser.add_argument("--labels", type=Path, help="CSV with key,label,notes columns")
    parser.add_argument("--output", type=Path, default=Path("tx_order_metrics.csv"))
    parser.add_argument("--cal", type=Path, default=DEFAULT_CAL)
    parser.add_argument("--tee-m", type=float, default=1.575)
    parser.add_argument("--tilt-deg", type=float, default=10.40495)
    parser.add_argument("--radar-height-m", type=float, default=0.1524)
    parser.add_argument("--ball-height-m", type=float, default=0.040)
    parser.add_argument("--club", default="9i")
    parser.add_argument("--net-m", type=float)
    parser.add_argument(
        "--tdm-sign",
        choices=("auto", "positive", "negative"),
        default="positive",
        help="Doppler phase sign policy used for replay (default: positive)",
    )
    args = parser.parse_args()
    if not args.normal and not args.reversed:
        parser.error("provide at least one --normal or --reversed directory")

    cal = Calibration.load(args.cal)
    cal.tee_range_m = args.tee_m
    cal.tilt_rad = np.radians(args.tilt_deg)
    cal.tee_ball_height_m = args.ball_height_m
    cal.meta["radar_height_m"] = args.radar_height_m
    labels = _labels(args.labels)
    rows: list[dict[str, object]] = []
    for tx_order, directories in (("normal", args.normal), ("reversed", args.reversed)):
        for directory in directories:
            for path in sorted(directory.glob("*.l3dump")):
                rows.append(
                    analyze(
                        path,
                        directory.name,
                        tx_order,
                        cal,
                        args.club,
                        args.net_m,
                        args.tdm_sign,
                        labels,
                    )
                )
                print(f"{tx_order:8s} {directory.name}/{path.name}")
    if not rows:
        parser.error("no .l3dump files found")

    fields = list(dict.fromkeys(key for row in rows for key in row))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    template = args.output.with_name(args.output.stem + "_labels.csv")
    if not template.exists():
        with template.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=("key", "label", "notes"))
            writer.writeheader()
            for row in rows:
                writer.writerow({"key": row["key"], "label": row["label"], "notes": row["notes"]})
    print_summary(rows)
    print(f"\nwrote {args.output}")
    print(f"label template: {template}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
