#!/usr/bin/env python3
"""Step-3 wave-test / manual-trigger dump harness (Mac-side, no Pi/GPIO).

Mimics the sound trigger: configures + starts the sensor over the CLI, confirms
capture is running via `stats`, then fires `l3dump` on demand and analyses the
real ADC for motion (a waving hand should light up a range bin and vary frame
to frame). This is the on-the-bench validation before moving to the Pi.

    uv run python firmware/l3_dump/wave_test.py --config          # cfg + start + stats
    uv run python firmware/l3_dump/wave_test.py --dump --save w.l3dump   # capture + analyse

Ports auto-detected (CLI = the port that answers `help` with our commands);
override with --cli/--data. read_dump gets a generous timeout since a 5-frame
dump is ~640 KB over UART.
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

import numpy as np                                   # noqa: E402
import serial                                        # noqa: E402
import iwr6843_l3dump as l3                           # noqa: E402
from iwr6843_runtime import DUMP_CMD                  # noqa: E402


def read_dump_greedy(ser, timeout_s):
    """Frame one dump, draining in_waiting greedily (avoids RX overflow on the
    big 5-frame transfer that the fixed-size read_dump loop drops)."""
    buf = bytearray()
    deadline = time.time() + timeout_s
    while len(buf) < l3.HEADER.size and time.time() < deadline:
        n = ser.in_waiting
        buf += ser.read(n if n else 1)
    need = l3.HEADER.size + l3.payload_nbytes(l3.parse_header(bytes(buf)))
    while len(buf) < need and time.time() < deadline:
        n = ser.in_waiting
        buf += ser.read(n if n else (need - len(buf)))
    if len(buf) < need:
        raise TimeoutError(f"dump timeout ({len(buf)}/{need} B)")
    return bytes(buf)


def _open(port: str, baud: int, timeout: float = 0.3) -> serial.Serial:
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, timeout
    s.dtr = False
    s.rts = False
    s.open()
    return s


def _cmd(cli: serial.Serial, line: str, window: float = 1.5) -> str:
    cli.reset_input_buffer()
    cli.write((line + "\n").encode())
    resp = b""
    t = time.time()
    while time.time() - t < window:
        resp += cli.read(512)
        if b"Done" in resp or b"Error" in resp:
            break
    return resp.decode(errors="replace")


def detect_ports(cli_arg, data_arg):
    ports = sorted(glob.glob("/dev/tty.SLAB_USBtoUART*"))
    cli = cli_arg
    if cli is None:
        for p in ports:
            try:
                s = _open(p, 115200)
            except Exception:  # noqa: BLE001
                continue
            r = _cmd(cli=s, line="help")
            s.close()
            if "sensorStart" in r:
                cli = p
                break
    data = data_arg or next((p for p in ports if p != cli), None)
    return cli, data


def send_config(cli: serial.Serial, path: str) -> None:
    for raw in open(path):
        line = raw.strip()
        if not line or line.startswith("%"):
            continue
        window = 6.0 if line.startswith("sensorStart") else 1.5
        resp = _cmd(cli, line, window)
        flat = " | ".join(x.strip() for x in resp.splitlines() if x.strip())
        print(f">> {line}\n   {flat}")
        if "Error" in resp:
            raise RuntimeError(f"config rejected: {line!r}")


def analyse(raw: bytes) -> None:
    meta, cube = l3.parse_dump(raw)                       # [nf, cpf, nrx, ns]
    nf = meta["n_frames"]
    nz = int(np.count_nonzero(cube))
    print(f"meta: {meta}")
    print(f"payload: {cube.size} samples, nonzero {nz} "
          f"({100.0*nz/cube.size:.1f}%), absmax {float(np.abs(cube).max()):.0f}")
    rfft = l3.range_fft(cube)                             # [nf, cpf, nrx, nrange]
    half = rfft.shape[-1] // 2
    # Per-frame range profile: power summed over chirps + RX, bins 1..half.
    prof = np.sum(np.abs(rfft[:, :, :, 1:half]) ** 2, axis=(1, 2))   # [nf, nrange-1]
    print("frame :  peak_bin   peak_mag     total_energy")
    for f in range(nf):
        pk = int(np.argmax(prof[f])) + 1
        print(f"  {f:2d}  :   {pk:4d}     {prof[f, pk-1]:.3e}    {prof[f].sum():.3e}")
    if nf > 1:
        # Motion metric: mean frame-to-frame change in the range profile,
        # normalised by mean energy. Static scene -> ~0; a wave -> clearly > 0.
        d = np.abs(np.diff(prof, axis=0)).mean()
        base = prof.mean() + 1e-9
        print(f"motion metric (frame-to-frame delta / mean): {d/base:.3f}  "
              f"({'MOTION' if d/base > 0.2 else 'mostly static'})")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cli")
    ap.add_argument("--data")
    ap.add_argument("--config", action="store_true",
                    help="send config/iwr6843_l3dump.cfg (configure + start)")
    ap.add_argument("--cfg-path", default="config/iwr6843_l3dump.cfg")
    ap.add_argument("--stats", action="store_true", help="print stats x3")
    ap.add_argument("--dump", action="store_true", help="capture + analyse a dump")
    ap.add_argument("--save")
    ap.add_argument("--timeout", type=float, default=25.0)
    a = ap.parse_args()

    cli_port, data_port = detect_ports(a.cli, a.data)
    if not cli_port:
        print("no CLI port found (is step-3 firmware running?)")
        return 1
    print(f"CLI={cli_port}  DATA={data_port}")
    cli = _open(cli_port, 115200)
    data = _open(data_port, 921600, timeout=0.3)
    try:
        if a.config:
            send_config(cli, a.cfg_path)
        if a.stats or a.config:
            for _ in range(3):
                print("stats:", " | ".join(x.strip() for x in
                      _cmd(cli, "stats").splitlines() if x.strip()))
                time.sleep(1.0)
        if a.dump:
            data.reset_input_buffer()
            cli.write(DUMP_CMD)
            t0 = time.time()
            raw = read_dump_greedy(data, timeout_s=a.timeout)
            print(f"dump: {len(raw)} bytes in {time.time()-t0:.1f}s")
            if a.save:
                with open(a.save, "wb") as fh:
                    fh.write(raw)
                print(f"saved -> {a.save}")
            analyse(raw)
    finally:
        cli.close()
        data.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
