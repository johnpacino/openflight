#!/usr/bin/env python3
"""Validate on-chip shadow candidates with static-reflector or ball tests."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

from openflight.gpio_factory import ensure_lgpio_pin_factory
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dump import compute_shadow_candidate, parse_dump


def _capture_stage(
    radar: IWR6843Radar,
    button,
    *,
    label: str,
    count: int,
    outdir: Path,
    gate_start: int,
    gate_count: int,
    min_power: int,
    auto_interval_s: float | None,
    log_handle,
) -> list[dict]:
    events = []
    for capture in range(1, count + 1):
        action = "hit one ball" if label == "ball" else "clap once"
        if auto_interval_s is None:
            print(f"[{label} {capture}/{count}] ARMED - {action}")
            button.wait_for_press()
            button.wait_for_release(timeout=2.0)
        else:
            print(
                f"[{label} {capture}/{count}] AUTO - "
                f"capturing in {auto_interval_s:g} seconds"
            )
            time.sleep(auto_interval_s)
        raw = radar.read_dump(timeout_s=45.0)
        path = outdir / f"{label}_{capture:02d}.l3dump"
        path.write_bytes(raw)
        meta, cube = parse_dump(raw)
        recorded = meta.get("shadow_candidates")
        if recorded is None:
            raise RuntimeError("dump has no shadow candidates; flash the shadow firmware")

        mismatches = []
        for frame, candidate in enumerate(recorded):
            expected = compute_shadow_candidate(
                cube[frame, ..., : meta["range_bin_counts"][frame]],
                n_tx=meta["n_tx"],
                range_bin_start=meta["range_bin_starts"][frame],
                gate_start=gate_start,
                gate_count=gate_count,
                min_power=min_power,
            )
            if candidate != expected:
                mismatches.append(frame)

        valid = [candidate for candidate in recorded if candidate["valid"]]
        strongest = max(valid, key=lambda item: item["power"], default=None)
        timeline = []
        for frame, candidate in enumerate(recorded):
            timeline.append(
                {
                    "frame": frame,
                    "time_offset_us": meta["frame_time_offsets_us"][frame],
                    "range_bin_start": meta["range_bin_starts"][frame],
                    "range_bin_count": meta["range_bin_counts"][frame],
                    **candidate,
                }
            )
        event = {
            "type": "shadow_reflector_capture",
            "stage": label,
            "capture": capture,
            "path": str(path),
            "bytes": len(raw),
            "frames": meta["n_frames"],
            "valid_candidates": len(valid),
            "parity_mismatch_frames": mismatches,
            "strongest": strongest,
            "candidates": timeline,
            "shadowstats": radar.cmd("shadowstats", 2.0).strip(),
            "stats": radar.stats().strip(),
        }
        log_handle.write(json.dumps(event) + "\n")
        log_handle.flush()
        strongest_text = (
            "none"
            if strongest is None
            else f"bin={strongest['range_bin']} tx={strongest['tx_index']} "
            f"power={strongest['power']}"
        )
        print(
            f"  {len(raw)} bytes, valid={len(valid)}/{meta['n_frames']}, "
            f"parity_mismatches={len(mismatches)}, strongest={strongest_text}"
        )
        events.append(event)
    return events


def _print_ball_summary(events: list[dict]) -> None:
    empty_events = [event for event in events if event["stage"].startswith("empty")]
    ball_events = [event for event in events if event["stage"] == "ball"]
    empty_powers = [
        candidate["power"]
        for event in empty_events
        for candidate in event["candidates"]
        if candidate["valid"]
    ]
    baseline_max = max(empty_powers, default=0)

    print("\nBall candidate summary")
    print(f"Empty-scene maximum power: {baseline_max}")
    for event in ball_events:
        valid = [candidate for candidate in event["candidates"] if candidate["valid"]]
        strongest = max(valid, key=lambda item: item["power"], default=None)
        above_baseline = sum(candidate["power"] > baseline_max for candidate in valid)
        if strongest is None:
            detail = "no valid candidates"
        else:
            ratio = strongest["power"] / max(baseline_max, 1)
            detail = (
                f"peak frame={strongest['frame']} bin={strongest['range_bin']} "
                f"tx={strongest['tx_index']} power={strongest['power']} "
                f"({ratio:.1f}x empty max), frames_above_empty={above_baseline}"
            )
        print(f"  shot {event['capture']:02d}: {detail}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None)
    parser.add_argument(
        "--cfg",
        default="config/iwr6843_l3dump_shadow_static.cfg",
    )
    parser.add_argument("--trigger-pin", type=int, default=17)
    parser.add_argument("--gate-start", type=int, default=26)
    parser.add_argument("--gate-count", type=int, default=14)
    parser.add_argument("--min-power", type=int, default=0)
    parser.add_argument(
        "--mode",
        choices=("reflector", "ball"),
        default="reflector",
        help="run the static-reflector diagnostic or capture struck balls",
    )
    parser.add_argument("--empty-captures", type=int, default=3)
    parser.add_argument("--reflector-captures", type=int, default=5)
    parser.add_argument("--shots", type=int, default=10)
    parser.add_argument(
        "--auto-interval-s",
        type=float,
        default=None,
        help="capture automatically after this delay instead of waiting for GPIO17",
    )
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path.home() / "openflight_sessions" / "iwr6843_shadow_reflector",
    )
    args = parser.parse_args()

    if args.gate_count <= 0 or args.gate_start < 0:
        parser.error("the range gate must be positive")
    if args.empty_captures < 1 or args.reflector_captures < 1 or args.shots < 1:
        parser.error("capture counts must be positive")
    if args.auto_interval_s is not None and args.auto_interval_s <= 0:
        parser.error("--auto-interval-s must be positive")

    button = None
    if args.auto_interval_s is None:
        ensure_lgpio_pin_factory()
        from gpiozero import Button  # pylint: disable=import-outside-toplevel

        button = Button(args.trigger_pin, pull_up=False, bounce_time=0.05)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = args.outdir / stamp
    outdir.mkdir(parents=True, exist_ok=True)
    log_path = outdir / "session.jsonl"

    print("OpenFlight must be stopped. Configuring the IWR6843...")
    try:
        with IWR6843Radar(port=args.port) as radar, log_path.open("w", encoding="utf-8") as log:
            radar.send_config(args.cfg)
            print(f"Radar running on {radar.port}; output: {outdir}")
            if args.mode == "ball":
                stages = (
                    ("empty_before", args.empty_captures, "Clear the hitting area and step away."),
                    (
                        "ball",
                        args.shots,
                        "Place a ball at the normal tee position. Keep the radar and mat fixed.",
                    ),
                    (
                        "empty_after",
                        args.empty_captures,
                        "Clear the hitting area without moving the radar or mat.",
                    ),
                )
            else:
                stages = (
                    ("empty_before", args.empty_captures, "Remove the reflector and step away."),
                    (
                        "reflector",
                        args.reflector_captures,
                        "Place the reflector at the marked 5 ft position, point it at the radar, "
                        "and step away.",
                    ),
                    (
                        "empty_after",
                        args.empty_captures,
                        "Remove the reflector without moving the radar.",
                    ),
                )
            events = []
            for label, count, prompt in stages:
                input(f"\n{prompt}\nPress Enter when ready...")
                events.extend(
                    _capture_stage(
                        radar,
                        button,
                        label=label,
                        count=count,
                        outdir=outdir,
                        gate_start=args.gate_start,
                        gate_count=args.gate_count,
                        min_power=args.min_power,
                        auto_interval_s=args.auto_interval_s,
                        log_handle=log,
                    )
                )
                time.sleep(0.25)
            if args.mode == "ball":
                _print_ball_summary(events)
    finally:
        if button is not None:
            button.close()

    print(f"Complete. Session log: {log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
