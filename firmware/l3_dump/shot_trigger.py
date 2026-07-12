#!/usr/bin/env python3
"""Shot capture: microphone-triggered L3 dump (Mac bench, stand-in for SEN-14262).

RUN THIS IN A LOCAL TERMINAL ON THE MAC WITH THE BOARD (mic access is blocked
over SSH by macOS privacy). First run prompts for microphone permission.

    cd ~/Projects/openflight
    uv run python firmware/l3_dump/shot_trigger.py

Flow: configures + starts the sensor, calibrates mic noise, then arms. On an
impact transient it waits --delay-ms (so the 40 ms ring fills with ball
flight), fires `l3dump`, saves shot_NNN.l3dump, prints per-frame range peaks +
a crude range-walk speed. Loops for the next shot. Ctrl-C to stop.

Clap once to sanity-check the trigger before swinging. (On the Pi rig, use
clap_test.py — same flow, GPIO sound trigger instead of the mic.)
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np       # noqa: E402
import l3host            # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--delay-ms", type=float, default=20.0,
                    help="wait after impact before freezing (fills ring with flight)")
    ap.add_argument("--threshold", type=float, default=None,
                    help="mic RMS trigger level (default: 12x calibrated noise)")
    ap.add_argument("--outdir", default=os.path.expanduser("~/openflight_shots"))
    ap.add_argument("--no-config", action="store_true",
                    help="skip sending the cfg (sensor already running)")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    import sounddevice as sd   # lazy: Mac bench only

    cli_port, data_port = l3host.detect_ports()
    if not cli_port:
        print("no CLI port found — is the board on and flashed?")
        return 1
    print(f"CLI={cli_port}  DATA={data_port}")
    cli = l3host.open_port(cli_port, l3host.CLI_BAUD)
    data = l3host.open_port(data_port, l3host.DATA_BAUD)

    if not a.no_config:
        print("configuring sensor...")
        l3host.send_config(cli)
        print("sensor running.")

    fs = 48000
    print("calibrating mic noise (2 s, keep quiet)...")
    noise = sd.rec(2 * fs, samplerate=fs, channels=1, dtype="float32")
    sd.wait()
    nrms = float(np.sqrt((noise ** 2).mean()))
    thresh = a.threshold if a.threshold else max(12.0 * nrms, 0.02)
    print(f"noise RMS {nrms:.4f} -> trigger threshold {thresh:.4f}")

    shot = len(glob.glob(os.path.join(a.outdir, "shot_*.l3dump")))
    stream = sd.InputStream(samplerate=fs, channels=1, blocksize=256, dtype="float32")
    stream.start()
    print("\nARMED — clap to test, then hit. Ctrl-C to stop.\n")
    try:
        while True:
            block, _ = stream.read(256)
            if float(np.sqrt((block ** 2).mean())) < thresh:
                continue
            if a.delay_ms > 0:
                time.sleep(a.delay_ms / 1000.0)
            data.reset_input_buffer()
            cli.write(b"l3dump\n")
            shot += 1
            print(f"[shot {shot}] TRIGGER (delay {a.delay_ms:.0f} ms) — dumping ~15 s...")
            raw = l3host.read_dump_besteffort(data)
            fn = os.path.join(a.outdir, f"shot_{shot:03d}.l3dump")
            with open(fn, "wb") as fh:
                fh.write(raw)
            print(f"[shot {shot}] {len(raw)}/{l3host.FULL_DUMP} B -> {fn}")
            l3host.analyse(raw)
            l3host.cmd(cli, "stats", 2.0)   # drain CLI; sensor auto-restarts
            time.sleep(1.0)
            stream.read(stream.read_available or 1)   # flush stale audio
            print("\nARMED — next shot when ready.\n")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        stream.stop()
        cli.close()
        data.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
