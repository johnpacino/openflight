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

Clap once to sanity-check the trigger before swinging.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import numpy as np                       # noqa: E402
import serial                            # noqa: E402
import sounddevice as sd                 # noqa: E402
import iwr6843_l3dump as l3               # noqa: E402

FULL_DUMP = 20 + 5 * 131072   # header + 5 frames
RANGE_RES_M = 0.0469
FRAME_S = 0.008


def _open(port, baud, timeout=0.3):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, timeout
    s.dtr = False
    s.rts = False
    s.open()
    return s


def _cmd(cli, line, window=1.5):
    cli.reset_input_buffer()
    cli.write((line + "\n").encode())
    resp = b""
    t = time.time()
    while time.time() - t < window:
        resp += cli.read(512)
        if b"Done" in resp or b"Error" in resp:
            break
    return resp.decode(errors="replace")


def detect_ports():
    ports = sorted(glob.glob("/dev/tty.SLAB_USBtoUART*"))
    for p in ports:
        try:
            s = _open(p, 115200)
        except Exception:  # noqa: BLE001
            continue
        r = _cmd(s, "help")
        s.close()
        if "sensorStart" in r:
            return p, next((q for q in ports if q != p), None)
    return None, None


def read_dump_besteffort(data, timeout_s=30.0):
    """Greedy drain; returns whatever arrives (partial tail loss tolerated)."""
    buf = b""
    t = time.time()
    last = t
    while time.time() - t < timeout_s:
        n = data.in_waiting
        c = data.read(n if n else 1)
        if c:
            buf += c
            last = time.time()
        elif buf and time.time() - last > 1.5:
            break
        if len(buf) >= FULL_DUMP:
            break
    return buf


def analyse(raw: bytes) -> None:
    nf = (len(raw) - 20) // 131072
    if nf < 1:
        print("  !! dump too short to analyse")
        return
    body = np.frombuffer(raw, dtype="<i2", offset=20, count=nf * 65536).astype(float)
    cube = (body[1::2] + 1j * body[0::2]).reshape(nf, 64, 4, 128)   # ImRe
    print(f"  {nf} frames  absmax {np.abs(cube).max():.0f}")
    peaks = []
    for f in range(nf):
        rfft = np.fft.fft(cube[f], axis=-1)
        prof = np.sum(np.abs(rfft[:, :, 3:64]) ** 2, axis=(0, 1))
        pk = int(np.argmax(prof)) + 3
        peaks.append(pk)
        print(f"  frame {f}: peak bin {pk:3d} = {pk * RANGE_RES_M:5.2f} m"
              f"   P {prof[pk - 3]:.2e}")
    if nf >= 3:
        walk = np.diff(peaks)
        v = np.mean(walk) * RANGE_RES_M / FRAME_S
        if np.all(walk >= 1):
            print(f"  >>> outbound range walk: ~{v:.1f} m/s ({v * 2.237:.0f} mph)")
        else:
            print(f"  (no clean outbound walk; peak deltas {list(walk)})")


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

    cli_port, data_port = detect_ports()
    if not cli_port:
        print("no CLI port found — is the board on and flashed?")
        return 1
    print(f"CLI={cli_port}  DATA={data_port}")
    cli = _open(cli_port, 115200)
    data = _open(data_port, 460800)

    if not a.no_config:
        print("configuring sensor...")
        _cmd(cli, "sensorStop", 3.0)
        for rawline in open(os.path.join(_ROOT, "config/iwr6843_l3dump.cfg")):
            line = rawline.strip()
            if not line or line.startswith("%"):
                continue
            resp = _cmd(cli, line, 6.0 if line.startswith("sensorStart") else 1.5)
            if "Error" in resp:
                print(f"config rejected: {line}")
                return 1
        print("sensor running.")

    # Mic calibration
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
            t0 = time.time()
            if a.delay_ms > 0:
                time.sleep(a.delay_ms / 1000.0)
            data.reset_input_buffer()
            cli.write(b"l3dump\n")
            shot += 1
            print(f"[shot {shot}] TRIGGER (delay {a.delay_ms:.0f} ms) — dumping ~15 s...")
            raw = read_dump_besteffort(data)
            fn = os.path.join(a.outdir, f"shot_{shot:03d}.l3dump")
            with open(fn, "wb") as fh:
                fh.write(raw)
            print(f"[shot {shot}] {len(raw)}/{FULL_DUMP} B -> {fn}")
            analyse(raw)
            # drain CLI response + let the sensor restart, then re-arm
            _cmd(cli, "stats", 2.0)
            time.sleep(1.0)
            stream.read(stream.read_available or 1)   # flush stale audio
            print(f"\nARMED — next shot when ready.\n")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        stream.stop()
        cli.close()
        data.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
