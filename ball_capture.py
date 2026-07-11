#!/usr/bin/env python3
"""Timestamped ball-session capture for the IWR6843LEVM (Stage-1b).

Streams the ball config to raw_uart.bin AND writes a wall-clock frame index
(frames_index.csv) so every TI frame can be aligned to the OPS + sound-trigger
timestamps in the OpenFlight session log. Both processes run on the same Pi, so
the Unix-epoch timestamps share one clock -> window-accurate alignment (known
cross-device jitter ~±60 ms = a handful of frames).

INTENT (for future sessions / Opus 4.8)
---------------------------------------
This is THE Stage-1b capture tool. Run it ALONGSIDE the OpenFlight software
(which drives the OPS + sound trigger and logs shot speed + trigger times):

    # terminal 1: OpenFlight (OPS + sound trigger + session log = the labels)
    # terminal 2:
    uv run python ball_capture.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
        --session ~/openflight_sessions/stage1/<date>_ball --seconds 3600

Hit shots; Ctrl-C when done. Each OpenFlight trigger timestamp marks an impact
in this stream. Analyze on the Mac: for each trigger time, find the frame
window in frames_index.csv, then characterize the real ball/club signature
(points/frame, SNR, range-walk) and compare detection/speed vs the OPS.

Reuses iwr6843_uart.SerialReader for config-send + framing (no duplication);
this tool only adds the timestamp index, a long-running loop, and sensorStop
on exit. The raw byte stream is ALWAYS saved (bring-up doc rule).
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import date
from pathlib import Path

import numpy as np

import iwr6843_uart as uart

INDEX_HEADER = "frame_number,t_epoch,cpu_cycles,n_det,nearest_range_m,nearest_doppler"

META_TEMPLATE = {
    "date": "", "test": "ball", "firmware": "oob-demo-3.5.0.4",
    "cfg_file": "", "range_res_m": 0.0469,  # ball cfg (100 MHz/us slope)
    "radar_height_m": "EDIT_ME (6in target = 0.152)",
    "board_tilt_deg": "EDIT_ME (~10 up; MEASURE it)",
    "tilt_method": "digital level", "tee_distance_m": "EDIT_ME",
    "net_distance_m": "EDIT_ME", "floor": "EDIT_ME",
    "rotated_90deg": True, "antenna_end_up": True, "ops_rig_running": True,
    # path to the OpenFlight session .jsonl whose trigger_event/shot_detected
    # timestamps label this capture (fill in for offline alignment):
    "openflight_session_log": "EDIT_ME",
}


def frame_index_row(frame: uart.Frame, t: float) -> str:
    """One CSV index line: frame_number, host wall-clock epoch, the sensor's own
    frame timestamp (``cpu_cycles`` — gives jitter-free inter-frame Δt, immune to
    host read-batching, so range-walk *rate* is trustworthy), detected-point
    count, and the nearest detected point's range + doppler.

    Pure and hardware-free so it can be unit-tested against synthetic frames.
    Empty fields when the frame carries no detected-points TLV / no points."""
    n_det = 0
    rng = dop = ""
    payload = frame.tlvs.get(uart.TLV_DETECTED_POINTS)
    if payload:
        pts = uart.parse_detected_points(payload)
        n_det = len(pts)
        if n_det:
            i = int(np.argmin(np.hypot(pts[:, 0], pts[:, 1])))
            rng = f"{np.hypot(pts[i, 0], pts[i, 1]):.3f}"
            dop = f"{pts[i, 3]:+.2f}"
    return (f"{frame.header.frame_number},{t:.4f},"
            f"{frame.header.time_cpu_cycles},{n_det},{rng},{dop}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cli", help="config/CLI port (115200); auto-detected if omitted")
    ap.add_argument("--data", help="data port (921600); auto-detected if omitted")
    ap.add_argument("--cfg", default="config/iwr6843_levm_ball.cfg",
                    help="ball .cfg to send before capture")
    ap.add_argument("--no-config", action="store_true",
                    help="skip cfg send (sensor already streaming)")
    ap.add_argument("--session", required=True, help="session dir (created)")
    ap.add_argument("--seconds", type=float, default=3600.0,
                    help="max run time; Ctrl-C ends early (default 1 h)")
    ap.add_argument("--overwrite", action="store_true",
                    help="replace an existing non-empty raw_uart.bin")
    args = ap.parse_args(argv)

    sess = Path(args.session).expanduser()
    sess.mkdir(parents=True, exist_ok=True)
    raw_path = sess / "raw_uart.bin"
    if raw_path.exists() and raw_path.stat().st_size > 0:
        if not args.overwrite:
            ap.error(f"{raw_path} already has data — use a new --session or "
                     "--overwrite (captures append otherwise).")
        raw_path.unlink()

    meta_path = sess / "session_meta.json"
    if not meta_path.exists():
        meta = dict(META_TEMPLATE)
        meta["date"] = date.today().isoformat()
        meta["cfg_file"] = args.cfg
        meta_path.write_text(json.dumps(meta, indent=2))
        print(f"created {meta_path} — EDIT geometry + openflight_session_log")

    cli, data = args.cli, args.data
    if not (cli and data):
        cli, data = uart.find_levm_ports()
        print(f"auto-detected ports: CLI={cli}  data={data}")
    rdr = uart.SerialReader(cli, data)
    if args.cfg and not args.no_config:
        print(f"sending {args.cfg} ...")
        rdr.send_config(args.cfg)

    index = open(sess / "frames_index.csv", "w")
    index.write(INDEX_HEADER + "\n")
    print(f"capturing -> {raw_path} + frames_index.csv   (Ctrl-C to stop)")

    n, t0, last = 0, time.time(), time.time()
    try:
        for frame in rdr.frames(raw_sink=str(raw_path), duration_s=args.seconds):
            t = time.time()
            index.write(frame_index_row(frame, t) + "\n")
            n += 1
            if t - last > 2.0:
                index.flush()
                print(f"\r{n} frames | {n / max(t - t0, 1e-9):.0f} fps | "
                      f"{t - t0:.0f}s | last #{frame.header.frame_number}   ",
                      end="", flush=True)
                last = t
    except KeyboardInterrupt:
        pass
    finally:
        index.close()
        try:                       # be polite: stop the sensor on the way out
            rdr.cli.write(b"sensorStop\n")
            time.sleep(0.5)
        except Exception:
            pass

    dt = time.time() - t0
    print(f"\ndone: {n} frames in {dt:.0f}s ({n / max(dt, 1e-9):.0f} fps) -> "
          f"{raw_path} ({raw_path.stat().st_size} bytes) + frames_index.csv")
    print("align on the Mac: OpenFlight trigger times -> frames_index.csv rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
