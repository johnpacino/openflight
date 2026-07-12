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
        print("could not find the CP2105 pair (/dev/ttyUSB*) — board on & flashed?")
        return 1
    print(f"CLI={cli_port}  DATA={data_port}")
    cli = l3host.open_port(cli_port, l3host.CLI_BAUD)
    data = l3host.open_port(data_port, l3host.DATA_BAUD)

    if not a.no_config:
        print("configuring sensor...")
        l3host.send_config(cli)
        print("sensor running.")

    trigger = threading.Event()
    button = Button(a.trigger_pin, pull_up=False, bounce_time=0.05)
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
            print(f"[clap {shot}] TRIGGER (delay {a.delay_ms:.0f} ms) — dumping ~15 s...")
            raw = l3host.read_dump_besteffort(data)
            fn = os.path.join(a.outdir, f"clap_{shot:03d}.l3dump")
            with open(fn, "wb") as fh:
                fh.write(raw)
            print(f"[clap {shot}] {len(raw)}/{l3host.FULL_DUMP} B -> {fn}")
            l3host.analyse(raw)
            l3host.cmd(cli, "stats", 2.0)   # drain CLI; sensor auto-restarts
            time.sleep(0.5)
            trigger.clear()                 # re-arm (ignore edges during dump)
            print("\nARMED — next clap/shot when ready.\n")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        cli.close()
        data.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
