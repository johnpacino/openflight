#!/usr/bin/env python3
"""Clap test: SEN-14262 GPIO sound trigger -> L3 dump, on the Pi.

The Pi-rig version of shot_trigger.py: instead of the Mac microphone, the
SparkFun SEN-14262's GATE output drives a Pi GPIO edge. On the edge the script
waits --delay-ms (so the 40 ms ring fills with post-impact ball flight), fires
`l3dump`, saves the dump, and prints per-frame range peaks + range-walk speed.

Wiring (same sensor as the OPS rig, GATE re-pointed at the Pi):
    SEN-14262 GATE -> BCM <trigger-pin>       SEN-14262 VCC -> Pi 3.3V
    SEN-14262 GND  -> Pi GND
IWR6843 LEVM on USB (both CP2105 UARTs enumerate as /dev/ttyUSB*).

    python3 firmware/l3_dump/clap_test.py --trigger-pin 17

Validate with a CLAP near the sensor first (hence the name), then hit shots.
Ctrl-C to stop. Dumps land in --outdir as clap_NNN.l3dump.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import l3host  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trigger-pin", type=int, required=True,
                    help="BCM pin wired to SEN-14262 GATE")
    ap.add_argument("--delay-ms", type=float, default=25.0,
                    help="wait after the edge before freezing the ring "
                         "(fills the 40 ms buffer with post-impact flight)")
    ap.add_argument("--cli", help="CLI UART (default: auto-detect)")
    ap.add_argument("--data", help="data UART (default: auto-detect)")
    ap.add_argument("--outdir", default=os.path.expanduser("~/openflight_shots"))
    ap.add_argument("--no-config", action="store_true",
                    help="skip sending the cfg (sensor already running)")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    # Lazy: Pi-only, and keeps this file importable off-Pi.
    from gpiozero import Button

    cli_port, data_port = (a.cli, a.data)
    if not cli_port or not data_port:
        cli_port, data_port = l3host.detect_ports()
    if not cli_port or not data_port:
        print("could not find the board (/dev/ttyUSB*) — on, flashed, v3?")
        return 1
    print(f"CLI={cli_port}  DATA={data_port}")
    cli = l3host.open_port(cli_port, l3host.CLI_BAUD)
    # v3 single-port firmware: CLI and dump share one UART -> one handle
    # (two handles on one tty would steal each other's bytes).
    data = cli if data_port == cli_port else l3host.open_port(
        data_port, l3host.DATA_BAUD)

    if not a.no_config:
        print("configuring sensor...")
        l3host.send_config(cli)
        print("sensor running.")

    trigger = threading.Event()
    # bounce_time=None is load-bearing: gpiozero's lgpio backend implements
    # debounce as "level must be stable for bounce_time before the edge is
    # reported", which DELAYS the trigger by the full bounce_time (50 ms cost
    # us the ball flight on the 2026-07-12 7i session). Re-arm suppression is
    # handled in software below (trigger.clear() after the dump), so contact
    # bounce can't double-fire anyway.
    button = Button(a.trigger_pin, pull_up=False, bounce_time=None)
    button.when_pressed = trigger.set

    shot = len(glob.glob(os.path.join(a.outdir, "clap_*.l3dump")))
    print(f"\nARMED on BCM{a.trigger_pin} — clap to test, then hit. Ctrl-C to stop.\n")
    try:
        while True:
            if not trigger.wait(timeout=1.0):
                continue
            if a.delay_ms > 0:
                time.sleep(a.delay_ms / 1000.0)
            data.reset_input_buffer()
            cli.write(b"l3dump\n")
            shot += 1
            print(f"[clap {shot}] TRIGGER (delay {a.delay_ms:.0f} ms) — dumping ~8 s...")
            raw = l3host.read_dump_besteffort(data)
            fn = os.path.join(a.outdir, f"clap_{shot:03d}.l3dump")
            with open(fn, "wb") as fh:
                fh.write(raw)
            print(f"[clap {shot}] {len(raw)}/{l3host.expected_len(raw)} B -> {fn}")
            l3host.analyse(raw)
            st = l3host.cmd(cli, "stats", 2.0)   # drain CLI + firmware health
            for ln in st.splitlines():
                if "=" in ln or "Error" in ln:
                    print(f"  [fw] {ln.strip()}")
            if not raw:
                print("  !! zero-byte dump = firmware refused l3dump (capture "
                      "inactive; RF restart failed on an earlier shot). "
                      "Power-cycle the radar USB and restart this script.")
            time.sleep(0.5)
            trigger.clear()                 # re-arm (ignore edges during dump)
            print("\nARMED — next clap/shot when ready.\n")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        cli.close()
        if data is not cli:
            data.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
