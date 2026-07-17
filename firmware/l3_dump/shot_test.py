#!/usr/bin/env python3
"""Shot test: SEN-14262 GPIO sound trigger -> L3 dump -> live speed + LA.

Thin field wrapper over the productized pipeline (openflight.iwr6843) — the
same code the server will eventually run. On the GPIO edge it waits
--delay-ms (ring fills with post-impact flight), dumps, and prints ball
speed plus all three launch-angle estimates (free / tee-constrained /
two-ray) with quality evidence. Dumps land in --outdir as
shot[_variant]_NNN.l3dump (variant tag from the --cfg name).

    uv run python firmware/l3_dump/shot_test.py --trigger-pin 17 --tee-m 1.57

Validate with a CLAP near the sensor first, then hit shots. Ctrl-C to stop.
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import math
import os
import re
import sys
import threading
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from openflight.iwr6843 import Calibration, IWR6843Radar, process_dump  # noqa: E402
from openflight.iwr6843.shot import movers_by_slot  # noqa: E402

DEFAULT_CFG = os.path.join(_ROOT, "config", "iwr6843_l3dump.cfg")
DEFAULT_CAL = os.path.join(_ROOT, "config", "iwr6843_cal_20260712.json")


def tx_order_from_cfg(cfg: str) -> str:
    """Infer the chirp order from the controlled-experiment cfg name."""
    name = os.path.basename(cfg)
    return "reversed" if re.search(r"(?:_vBR|_txrev)\.cfg$", name) else "normal"


def resolve_tx_order(cfg: str, requested: str) -> str:
    """Resolve auto ordering and reject an explicit config contradiction."""
    configured = tx_order_from_cfg(cfg)
    if requested == "auto":
        return configured
    if requested != configured:
        raise ValueError(
            f"--tx-order {requested} conflicts with {os.path.basename(cfg)} ({configured})"
        )
    return requested


def processing_fingerprint(paths: list[str]) -> str:
    """Short content hash for the exact config, calibration, and pipeline."""
    digest = hashlib.sha256()
    for path in paths:
        if not os.path.isfile(path):
            continue
        digest.update(os.path.basename(path).encode("utf-8"))
        with open(path, "rb") as handle:
            digest.update(handle.read())
    return digest.hexdigest()[:16]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--trigger-pin", type=int, required=True, help="BCM pin wired to SEN-14262 GATE"
    )
    ap.add_argument(
        "--delay-ms", type=float, default=50.0, help="wait after the edge before freezing the ring"
    )
    ap.add_argument(
        "--cfg", default=DEFAULT_CFG, help="RF config (variant builds need their own _vX.cfg)"
    )
    ap.add_argument("--cal", default=DEFAULT_CAL, help="calibration JSON")
    ap.add_argument(
        "--tee-m",
        type=float,
        default=None,
        help="tape-measured radar-face-to-ball distance (m); enables the tee-constrained LA fit",
    )
    ap.add_argument(
        "--net-m",
        type=float,
        default=None,
        help="radar-face-to-net distance (m); excludes ball-riding-the-net motion from all fits",
    )
    ap.add_argument("--coherent-loops", type=int, default=4)
    ap.add_argument("--club", default=None, help="club label used for the ball-track speed floor")
    ap.add_argument(
        "--tx-order",
        choices=("auto", "normal", "reversed"),
        default="auto",
        help="chirp TX order; auto recognizes *_vBR.cfg",
    )
    ap.add_argument(
        "--tdm-sign",
        choices=("auto", "positive", "negative"),
        default="auto",
        help="Doppler phase sign policy; use positive for the locked TrackMan experiment",
    )
    ap.add_argument("--tilt-deg", type=float, help="override mount tilt from calibration")
    ap.add_argument(
        "--radar-height-m", type=float, help="override antenna-center height from calibration"
    )
    ap.add_argument("--ball-height-m", type=float, help="ball center height above the floor/mat")
    ap.add_argument("--outdir", default=os.path.expanduser("~/openflight_shots"))
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument(
        "--no-config", action="store_true", help="skip sending the cfg (sensor already running)"
    )
    ap.add_argument(
        "--movers", action="store_true", help="also print per-slot strongest movers (diagnostic)"
    )
    a = ap.parse_args()
    try:
        tx_order = resolve_tx_order(a.cfg, a.tx_order)
    except ValueError as err:
        ap.error(str(err))
    os.makedirs(a.outdir, exist_ok=True)

    from gpiozero import Button  # lazy: Pi-only import

    cal = Calibration.load(a.cal) if os.path.exists(a.cal) else Calibration.identity()
    if a.tee_m is not None:
        cal.tee_range_m = a.tee_m
    if a.tilt_deg is not None:
        cal.tilt_rad = math.radians(a.tilt_deg)
    if a.radar_height_m is not None:
        cal.meta["radar_height_m"] = a.radar_height_m
    if a.ball_height_m is not None:
        cal.tee_ball_height_m = a.ball_height_m
    fingerprint = processing_fingerprint(
        [
            a.cfg,
            a.cal,
            os.path.join(_ROOT, "src", "openflight", "iwr6843", "shot.py"),
            os.path.join(_ROOT, "src", "openflight", "iwr6843", "doa.py"),
            os.path.join(_ROOT, "src", "openflight", "iwr6843", "tracking.py"),
            os.path.join(_ROOT, "src", "openflight", "iwr6843", "trajectory.py"),
        ]
    )
    print(
        f"calibration: {cal.source}"
        + (
            f"  tee @ {cal.tee_range_m} m"
            if cal.tee_range_m
            else "  (no --tee-m: tee-constrained fit disabled)"
        )
    )

    log_dir = os.path.expanduser("~/openflight_sessions")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, time.strftime("shot_test_%Y%m%d_%H%M%S.jsonl"))

    def log(entry: dict) -> None:
        entry["ts"] = time.time()
        with open(log_path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(entry) + "\n")

    radar = IWR6843Radar(port=a.port)
    print(f"radar on {radar.port}")
    if not a.no_config:
        print(f"configuring sensor ({os.path.basename(a.cfg)})...")
        radar.send_config(a.cfg)
        print("sensor running.")

    match = re.search(r"l3dump(_[A-Za-z0-9]+)\.cfg$", os.path.basename(a.cfg))
    prefix = "shot" + (match.group(1) if match else "")

    trigger = threading.Event()
    # bounce_time=None is load-bearing: lgpio debounce DELAYS the edge by the
    # full bounce time (50 ms cost us the ball flight on 2026-07-12).
    button = Button(a.trigger_pin, pull_up=False, bounce_time=None)
    button.when_pressed = trigger.set

    log(
        {
            "type": "session_start",
            "cfg": a.cfg,
            "cal": cal.source,
            "tee_m": cal.tee_range_m,
            "prefix": prefix,
            "coherent_loops": a.coherent_loops,
            "delay_ms": a.delay_ms,
            "net_m": a.net_m,
            "club": a.club,
            "tx_order": tx_order,
            "tdm_sign_policy": a.tdm_sign,
            "mount_tilt_deg": math.degrees(cal.tilt_rad),
            "antenna_center_height_m": cal.radar_height_m,
            "ball_height_m": cal.tee_ball_height_m,
            "range_bias_m": cal.range_bias_m,
            "processing_fingerprint": fingerprint,
            "outdir": os.path.abspath(a.outdir),
        }
    )
    print(f"session log: {log_path}")
    shot_no = len(glob.glob(os.path.join(a.outdir, prefix + "_*.l3dump")))
    print(
        f"\nARMED on BCM{a.trigger_pin} [{prefix}] — clap to test, "
        f"then hit. TX order: {tx_order}; TDM sign: {a.tdm_sign}. Ctrl-C to stop.\n"
    )
    try:
        while True:
            if not trigger.wait(timeout=1.0):
                continue
            trigger_ts = time.time()
            if a.delay_ms > 0:
                time.sleep(a.delay_ms / 1000.0)
            shot_no += 1
            log(
                {
                    "type": "trigger",
                    "n": shot_no,
                    "trigger_ts": trigger_ts,
                    "delay_ms": a.delay_ms,
                    "tx_order": tx_order,
                    "tdm_sign_policy": a.tdm_sign,
                }
            )
            print(f"[{prefix} {shot_no}] TRIGGER (delay {a.delay_ms:.0f} ms) — dumping...")
            t_0 = time.time()
            raw = radar.read_dump()
            t_dump = time.time() - t_0
            path = os.path.join(a.outdir, f"{prefix}_{shot_no:03d}.l3dump")
            with open(path, "wb") as out:
                out.write(raw)
            print(f"[{prefix} {shot_no}] {len(raw)} B in {t_dump:.2f} s -> {path}")
            t_0 = time.time()
            try:
                shot = process_dump(
                    raw,
                    cal,
                    coherent_loops=a.coherent_loops,
                    net_range_m=a.net_m,
                    club=a.club,
                    tx_order=tx_order,
                    tdm_sign_policy=a.tdm_sign,
                )
                print(
                    f"  >>> {shot.summary()}   "
                    f"(analysis {time.time() - t_0:.2f} s, "
                    f"{shot.n_angle_points} angle pts)"
                )
                if a.movers:
                    for slot, rng_m, pwr in movers_by_slot(raw):
                        print(f"      slot {slot}: {rng_m:4.2f} m  P {pwr:.2e}")
                log(
                    {
                        "type": "shot",
                        "n": shot_no,
                        "file": path,
                        "bytes": len(raw),
                        "dump_s": round(t_dump, 3),
                        "trigger_ts": trigger_ts,
                        **shot.to_dict(),
                    }
                )
            except (ValueError, IndexError) as err:
                print(f"  !! analysis failed: {err}")
                log(
                    {
                        "type": "shot_error",
                        "n": shot_no,
                        "file": path,
                        "bytes": len(raw),
                        "trigger_ts": trigger_ts,
                        "error": str(err),
                    }
                )
            health = radar.stats()
            for line in health.splitlines():
                if "=" in line or "Error" in line:
                    print(f"  [fw] {line.strip()}")
            if not raw:
                print("  !! zero-byte dump: power-cycle the radar USB and restart this script.")
            time.sleep(0.5)
            trigger.clear()
            print("\nARMED — next clap/shot when ready.\n")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        radar.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
