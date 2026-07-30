#!/usr/bin/env python3
"""Compare club-path evidence after removing TDM loops offline."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path

from openflight.iwr6843.club import estimate_club_path
from openflight.iwr6843.dump import parse_header, select_tdm_loops
from openflight.iwr6843.lcmf import estimate_lcmf_v1
from openflight.iwr6843.monitor import tx_order_from_config
from openflight.iwr6843.replay import build_replay_calibration, inputs_from_session


@dataclass(frozen=True)
class AblationRecord:
    """One loop-selection result for one shot."""

    shot_number: int | None
    variant: str
    source_loops: int
    kept_loops: int
    loop_start: int
    ball_status: str
    club_status: str
    path_deg: float | None
    range_rate_ms: float | None
    n_frames: int
    n_snapshots: int
    capture_path: str


def loop_variants(source_loops: int, keep_loops: int) -> list[tuple[str, int, int]]:
    """Full capture plus first/center/last contiguous loop selections."""
    if keep_loops <= 0 or keep_loops > source_loops:
        raise ValueError(f"--keep-loops must be between 1 and {source_loops}")
    variants = [("full", 0, source_loops)]
    candidates = (
        ("first", 0),
        ("center", (source_loops - keep_loops) // 2),
        ("last", source_loops - keep_loops),
    )
    seen = {0} if keep_loops == source_loops else set()
    for name, start in candidates:
        if start in seen:
            continue
        variants.append((name, start, keep_loops))
        seen.add(start)
    return variants


def club_speeds_from_session(session_path: Path) -> dict[int, float]:
    """OPS club speed keyed by shot number."""
    speeds: dict[int, float] = {}
    with session_path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            entry = json.loads(line)
            if entry.get("type") != "shot_detected":
                continue
            shot_number = entry.get("shot_number")
            speed = entry.get("club_speed_mph")
            if shot_number is not None and speed is not None:
                speeds[int(shot_number)] = float(speed)
    return speeds


def resolve_capture_path(capture_path: Path, dump_dir: Path | None) -> Path:
    """Use the session path when local, otherwise find its basename in dump-dir."""
    if capture_path.is_file() or dump_dir is None:
        return capture_path
    return dump_dir / capture_path.name


def summarize(records: list[AblationRecord]) -> None:
    """Print coverage and paired path deltas by loop-selection variant."""
    baseline = {
        record.shot_number: record
        for record in records
        if record.variant == "full" and record.shot_number is not None
    }
    variants = list(dict.fromkeys(record.variant for record in records))
    print("\nsummary")
    for variant in variants:
        group = [record for record in records if record.variant == variant]
        detected = [record for record in group if record.range_rate_ms is not None]
        accepted = [record for record in group if record.path_deg is not None]
        deltas = [
            abs(record.path_deg - baseline[record.shot_number].path_deg)
            for record in accepted
            if record.shot_number in baseline and baseline[record.shot_number].path_deg is not None
        ]
        delta_text = ""
        if deltas:
            ordered = sorted(deltas)
            p90_index = min(len(ordered) - 1, int(0.9 * len(ordered)))
            delta_text = (
                f", mean |delta|={statistics.fmean(deltas):.2f} deg"
                f", p90 |delta|={ordered[p90_index]:.2f} deg"
            )
        print(
            f"{variant:>6}: club detected {len(detected)}/{len(group)}, "
            f"path accepted {len(accepted)}/{len(group)}{delta_text}"
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Replay one IWR6843 session at full loops and deterministic "
            "first/center/last loop subsets."
        )
    )
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument(
        "--dump-dir",
        type=Path,
        default=None,
        help="Local dump directory when the session contains Pi absolute paths",
    )
    parser.add_argument("--keep-loops", type=int, default=10)
    parser.add_argument("--club", default=None, help="Club label applied to every capture")
    parser.add_argument("--tee-m", required=True, type=float)
    parser.add_argument("--net-m", type=float, default=None)
    parser.add_argument("--tilt-deg", type=float, default=None)
    parser.add_argument("--radar-height-m", type=float, default=None)
    parser.add_argument("--ball-height-m", type=float, default=0.040)
    parser.add_argument("--azimuth-offset-deg", type=float, default=0.0)
    parser.add_argument(
        "--cal",
        default="config/iwr6843_calibration_reference.json",
    )
    parser.add_argument(
        "--cfg",
        default="config/iwr6843_l3dump_vTX2_configurable.cfg",
    )
    parser.add_argument(
        "--tx-order",
        choices=("auto", "normal", "reversed"),
        default="auto",
    )
    parser.add_argument("--out", type=Path, default=None, help="Optional detailed CSV output")
    return parser


def main() -> int:
    args = _parser().parse_args()
    session_path = args.session.expanduser()
    dump_dir = args.dump_dir.expanduser() if args.dump_dir is not None else None
    configured_order = tx_order_from_config(args.cfg)
    tx_order = configured_order if args.tx_order == "auto" else args.tx_order
    if tx_order != configured_order:
        raise SystemExit(
            f"--tx-order {tx_order} conflicts with {Path(args.cfg).name} ({configured_order})"
        )

    calibration = build_replay_calibration(
        args.cal,
        tee_range_m=args.tee_m,
        tilt_deg=args.tilt_deg,
        radar_height_m=args.radar_height_m,
        ball_height_m=args.ball_height_m,
    )
    club_speeds = club_speeds_from_session(session_path)
    records: list[AblationRecord] = []

    print(
        "shot  variant  loops  club status                         "
        "path     radial m/s  frames/snaps"
    )
    for replay_input in inputs_from_session(session_path, club=args.club):
        shot_number = replay_input.shot_number
        club_speed = club_speeds.get(int(shot_number)) if shot_number is not None else None
        capture_path = resolve_capture_path(replay_input.capture_path, dump_dir)
        if club_speed is None:
            print(f"{str(shot_number):>4}  skipped: no OPS club speed")
            continue
        if not capture_path.is_file():
            print(f"{str(shot_number):>4}  skipped: capture not found: {capture_path}")
            continue

        raw = capture_path.read_bytes()
        meta = parse_header(raw)
        if meta["n_tx"] <= 0 or meta["chirps_per_frame"] % meta["n_tx"] != 0:
            print(f"{str(shot_number):>4}  skipped: invalid TDM geometry")
            continue
        source_loops = meta["chirps_per_frame"] // meta["n_tx"]
        variants = loop_variants(source_loops, args.keep_loops)
        ball = estimate_lcmf_v1(
            raw,
            calibration,
            ball_speed_mph=replay_input.ball_speed_mph,
            club=replay_input.club,
            net_range_m=args.net_m,
            tx_order=tx_order,
        )
        tdm_sign = ball.tdm_sign_used if ball.tdm_sign_used in (-1, 1) else 1

        for variant, loop_start, kept_loops in variants:
            candidate = (
                raw
                if variant == "full"
                else select_tdm_loops(raw, start=loop_start, count=kept_loops)
            )
            result = estimate_club_path(
                candidate,
                calibration,
                ops_club_speed_mph=club_speed,
                impact_t_s=ball.impact_t_s,
                aim_offset_deg=args.azimuth_offset_deg,
                tdm_sign=tdm_sign,
            )
            record = AblationRecord(
                shot_number=shot_number,
                variant=variant,
                source_loops=source_loops,
                kept_loops=kept_loops,
                loop_start=loop_start,
                ball_status=ball.status,
                club_status=result.status,
                path_deg=result.path_deg,
                range_rate_ms=result.range_rate_ms,
                n_frames=result.n_frames,
                n_snapshots=result.n_snapshots,
                capture_path=str(capture_path),
            )
            records.append(record)
            path_text = f"{result.path_deg:+7.2f}" if result.path_deg is not None else "    n/a"
            speed_text = (
                f"{result.range_rate_ms:10.2f}"
                if result.range_rate_ms is not None
                else "       n/a"
            )
            print(
                f"{str(shot_number):>4}  {variant:>7}  "
                f"{kept_loops:>2}@{loop_start:<2}  "
                f"{result.status[:31]:<31} {path_text}  {speed_text}  "
                f"{result.n_frames:>2}/{result.n_snapshots:<3}"
            )

    if not records:
        raise SystemExit("no captures had both a local dump and OPS club speed")
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(asdict(records[0]).keys()))
            writer.writeheader()
            writer.writerows(asdict(record) for record in records)
        print(f"\nwrote {len(records)} rows to {args.out}")
    summarize(records)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
