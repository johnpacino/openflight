#!/usr/bin/env python3
"""Stage-1 hardware-day capture CLI for the IWR6843LEVM.

INTENT (for future sessions / Opus 4.8)
---------------------------------------
Entry point #1 on hardware day. Written and dry-tested BEFORE the board
arrived; the only unverified parts are the serial-port layer and the
VERIFY-ON-HW byte conventions in iwr6843_uart.py (A1-A3). Expected usage:

    uv run python stage1_capture.py --list-ports
    uv run python stage1_capture.py \
        --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
        --cfg config/iwr6843_levm_static.cfg \
        --session ~/openflight_sessions/stage1/2026-07-08_static_h250 \
        --seconds 30 --verify 10

Expected outcome: raw bytes land in <session>/raw_uart.bin, a
session_meta.json skeleton is created (EDIT the geometry fields before
analysis!), a live status line ticks per frame, and --verify N runs the
A1 (TLV length reconciliation) and A3 (phase-ramp linearity) checks from
docs/plans/stage1_levm_bringup.md §5 on the first N heatmap frames.
A3 check meaning: a single strong reflector must give a LINEAR unwrapped
phase ramp across the 8 virtual antennas; RMS residual >> 10 deg means the
VIRT_ANT_ORDER assumption in iwr6843_uart.py is wrong — fix it there.

Port identification: the CP2105 exposes TWO ports. The *Enhanced* function
is the config CLI (115200); *Standard* is data (921600). On Linux/Pi these
are usually /dev/ttyUSB0 and /dev/ttyUSB1 respectively; on macOS look for
/dev/tty.SLAB_USBtoUART* or /dev/tty.usbserial* (install the Silicon Labs
VCP driver if nothing appears). If config lines get no 'Done' response,
the two ports are probably swapped — just swap the arguments.
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import date
from pathlib import Path

import numpy as np

import iwr6843_uart as uart

META_TEMPLATE = {
    "date": "", "test": "EDIT_ME: static|ball", "firmware": "oob-demo-3.x",
    "cfg_file": "", "range_res_m": 0.0488,  # static cfg; ball cfg is 0.0469
    "radar_height_m": "EDIT_ME", "board_tilt_deg": "EDIT_ME",
    "tilt_method": "digital level", "tee_distance_m": "EDIT_ME",
    "net_distance_m": "EDIT_ME", "floor": "EDIT_ME",
    "rotated_90deg": True, "antenna_end_up": True,
    "reflector_positions": [{"range_m": "EDIT_ME", "height_m": "EDIT_ME"}],
    "ops_rig_running": True, "trigger_timestamps": [],
}


def list_ports() -> None:
    from serial.tools import list_ports as lp
    ports = lp.comports()
    if not ports:
        print("No serial ports found. macOS: install Silicon Labs CP210x "
              "VCP driver; Linux: check dmesg for cp210x.")
    for p in ports:
        print(f"{p.device:32s} {p.description}")


def verify_frame(frame: uart.Frame, n_checked: int) -> None:
    """Run the runbook §5 first-frame checks (A1 + A3) and print verdicts."""
    a1 = "PASS" if frame.ok else "FAIL (see A1 note in iwr6843_uart.py)"
    line = f"[verify #{n_checked}] A1 length-reconcile: {a1}"
    payload = frame.tlvs.get(uart.TLV_AZIMUTH_STATIC_HEATMAP)
    if payload is not None:
        hm = uart.parse_azimuth_heatmap(payload)
        k = int(np.argmax(np.sum(np.abs(hm) ** 2, axis=1)))
        ph = np.unwrap(np.angle(hm[k]))
        coeffs = np.polyfit(np.arange(len(ph)), ph, 1)
        resid = np.degrees(np.sqrt(np.mean(
            (ph - np.polyval(coeffs, np.arange(len(ph)))) ** 2)))
        a3 = "PASS" if resid < 10.0 else "FAIL -> check VIRT_ANT_ORDER"
        line += (f" | A3 phase-ramp (bin {k}): slope "
                 f"{np.degrees(coeffs[0]):+.1f} deg/ant, "
                 f"residual {resid:.1f} deg RMS: {a3}")
    print(line)


def frame_status(frame: uart.Frame) -> str:
    bits = [f"frame {frame.header.frame_number}",
            f"tlvs {sorted(frame.tlvs.keys())}"]
    if uart.TLV_DETECTED_POINTS in frame.tlvs:
        pts = uart.parse_detected_points(frame.tlvs[uart.TLV_DETECTED_POINTS])
        bits.append(f"{len(pts)} pts")
        if len(pts):
            nearest = pts[np.argmin(np.hypot(pts[:, 0], pts[:, 1]))]
            bits.append(f"nearest r={np.hypot(nearest[0], nearest[1]):.2f}m "
                        f"v={nearest[3]:+.1f}m/s")
    if uart.TLV_AZIMUTH_STATIC_HEATMAP in frame.tlvs:
        hm = uart.parse_azimuth_heatmap(
            frame.tlvs[uart.TLV_AZIMUTH_STATIC_HEATMAP])
        p = np.sum(np.abs(hm) ** 2, axis=1)
        bits.append(f"heatmap peak bin {int(np.argmax(p))} "
                    f"({10 * np.log10(np.max(p) + 1e-12):.0f} dB)")
    return " | ".join(bits)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--cli", help="config/CLI serial port (115200)")
    ap.add_argument("--data", help="data serial port (921600)")
    ap.add_argument("--cfg", help=".cfg to send before capture")
    ap.add_argument("--no-config", action="store_true",
                    help="skip sending cfg (sensor already running)")
    ap.add_argument("--session", help="session directory (created if needed)")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--max-frames", type=int, default=0, help="0 = unlimited")
    ap.add_argument("--verify", type=int, default=5,
                    help="run A1/A3 checks on first N frames (0 = off)")
    ap.add_argument("--overwrite", action="store_true",
                    help="replace an existing non-empty raw_uart.bin "
                         "(default: refuse, so re-runs don't stack captures)")
    args = ap.parse_args()

    if args.list_ports:
        list_ports()
        return
    if not (args.cli and args.data and args.session):
        ap.error("--cli, --data and --session are required (or --list-ports)")

    sess = Path(args.session).expanduser()
    sess.mkdir(parents=True, exist_ok=True)
    meta_path = sess / "session_meta.json"
    if not meta_path.exists():
        meta = dict(META_TEMPLATE)
        meta["date"] = date.today().isoformat()
        meta["cfg_file"] = args.cfg or "none (sensor pre-configured)"
        meta_path.write_text(json.dumps(meta, indent=2))
        print(f"created {meta_path} — EDIT the geometry fields before analysis")

    rdr = uart.SerialReader(args.cli, args.data)
    if args.cfg and not args.no_config:
        print(f"sending {args.cfg} ...")
        rdr.send_config(args.cfg)

    raw_path = sess / "raw_uart.bin"
    if raw_path.exists() and raw_path.stat().st_size > 0:
        if not args.overwrite:
            ap.error(f"{raw_path} already has {raw_path.stat().st_size} bytes — "
                     "captures append and would stack. Use a new --session dir, "
                     "or pass --overwrite to replace it.")
        raw_path.unlink()
    print(f"capturing -> {raw_path}  ({args.seconds}s"
          f"{' / ' + str(args.max_frames) + ' frames' if args.max_frames else ''})")
    t0, n = time.time(), 0
    try:
        for frame in rdr.frames(raw_sink=str(raw_path),
                                duration_s=args.seconds):
            n += 1
            if args.verify and n <= args.verify:
                verify_frame(frame, n)
            else:
                print(f"\r{frame_status(frame)}   ", end="", flush=True)
            if time.time() - t0 > args.seconds:
                break
            if args.max_frames and n >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    dt = time.time() - t0
    print(f"\ndone: {n} frames in {dt:.1f}s ({n / max(dt, 1e-9):.1f} fps) "
          f"-> {raw_path} ({raw_path.stat().st_size} bytes)")
    print(f"next: uv run --extra analysis python stage1_analyze.py --session {sess}")


if __name__ == "__main__":
    main()
