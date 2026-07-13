#!/usr/bin/env python3
"""Shot test: SEN-14262 GPIO sound trigger -> L3 dump, on the Pi.

(Formerly clap_test.py.) The SparkFun SEN-14262's GATE output drives a Pi
GPIO edge. On the edge the script waits --delay-ms (so the ring fills with
post-impact ball flight), fires `l3dump`, saves the dump, and prints the
MTI movers + range-walk ball speed. Dumps are named shot[_variant]_NNN
(variant tag derived from the --cfg name, e.g. _vB).

Wiring (same sensor as the OPS rig, GATE re-pointed at the Pi):
    SEN-14262 GATE -> BCM <trigger-pin>       SEN-14262 VCC -> Pi 3.3V
    SEN-14262 GND  -> Pi GND
IWR6843 LEVM on USB (both CP2105 UARTs enumerate as /dev/ttyUSB*).

    python3 firmware/l3_dump/shot_test.py --trigger-pin 17

Validate with a CLAP near the sensor first, then hit shots.
Ctrl-C to stop.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import l3host  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trigger-pin", type=int, required=True,
                    help="BCM pin wired to SEN-14262 GATE")
    ap.add_argument("--delay-ms", type=float, default=50.0,
                    help="wait after the edge before freezing the ring "
                         "(fills the 72 ms buffer with post-impact flight; "
                         "~10 ms of pre-impact club lands at the front)")
    ap.add_argument("--cli", help="CLI UART (default: auto-detect)")
    ap.add_argument("--data", help="data UART (default: auto-detect)")
    ap.add_argument("--outdir", default=os.path.expanduser("~/openflight_shots"))
    ap.add_argument("--cfg", default=l3host.CFG_PATH,
                    help="RF config to send (variant builds need their own: "
                         "config/iwr6843_l3dump_vB.cfg / _vC.cfg)")
    ap.add_argument("--no-config", action="store_true",
                    help="skip sending the cfg (sensor already running)")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    # file prefix self-labels the variant: iwr6843_l3dump_vB.cfg -> shot_vB
    m = re.search(r"l3dump(_[A-Za-z0-9]+)\.cfg$", os.path.basename(a.cfg))
    prefix = "shot" + (m.group(1) if m else "")

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
        print(f"configuring sensor ({os.path.basename(a.cfg)})...")
        l3host.send_config(cli, a.cfg)
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

    shot = len(glob.glob(os.path.join(a.outdir, prefix + "_*.l3dump")))
    print(f"\nARMED on BCM{a.trigger_pin} [{prefix}] — clap to test, then hit. Ctrl-C to stop.\n")
    try:
        while True:
            if not trigger.wait(timeout=1.0):
                continue
            if a.delay_ms > 0:
                time.sleep(a.delay_ms / 1000.0)
            data.reset_input_buffer()
            cli.write(b"l3dump\n")
            shot += 1
            print(f"[{prefix} {shot}] TRIGGER (delay {a.delay_ms:.0f} ms) — dumping ~8 s...")
            t0 = time.time()
            raw = l3host.read_dump_besteffort(data)
            t_dump = time.time() - t0
            fn = os.path.join(a.outdir, f"{prefix}_{shot:03d}.l3dump")
            with open(fn, "wb") as fh:
                fh.write(raw)
            print(f"[{prefix} {shot}] {len(raw)}/{l3host.expected_len(raw)} B "
                  f"in {t_dump:.2f} s -> {fn}")
            t0 = time.time()
            l3host.analyse(raw)
            print(f"  (analysis {time.time() - t0:.2f} s)")
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
