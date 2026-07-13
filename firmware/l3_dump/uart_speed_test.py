#!/usr/bin/env python3
"""UART speed test for firmware v3 (single-port, 1.04 Mbaud).

v3 moves the dump onto UARTA = the CP2105 ENHANCED interface at 1,041,667
baud (exact divisor IWR-side, +0.17% CP2105-side), sharing the port with the
CLI. This harness configures the sensor, fires N dumps back-to-back, and
reports per-dump throughput + integrity. No sound trigger needed.

    python3 firmware/l3_dump/uart_speed_test.py            # auto-detect port
    python3 firmware/l3_dump/uart_speed_test.py --n 10 --save-last

NOTE: only works against v3 firmware (clap_test/l3host still speak v2's
two-port 115200/460800 arrangement — if this script finds no CLI, you're
probably running v2, and vice versa).
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
from iwr6843_l3dump import HEADER, parse_header, payload_nbytes  # noqa: E402

V3_BAUD = 1041667
MAGIC = b"ILD1"


def find_cli(baud):
    for p in sorted(glob.glob("/dev/ttyUSB*")) or sorted(
            glob.glob("/dev/tty.SLAB_USBtoUART*")):
        try:
            s = l3host.open_port(p, baud)
        except Exception:  # noqa: BLE001
            continue
        if "sensorStart" in l3host.cmd(s, "help"):
            return s, p
        s.close()
    return None, None


def timed_dump(ser, timeout_s=30.0):
    """Fire l3dump; return (bytes, seconds from write to last byte).

    CLI echo precedes the binary payload on the shared port, so sync on the
    ILD1 magic, then read to the header-declared length.
    """
    ser.reset_input_buffer()
    t0 = time.time()
    ser.write(b"l3dump\n")
    buf = b""
    expected = None
    last = t0
    while time.time() - t0 < timeout_s:
        n = ser.in_waiting
        c = ser.read(n if n else 1)
        if c:
            buf += c
            last = time.time()
        elif buf and time.time() - last > 4.0:
            break
        if expected is None:
            i = buf.find(MAGIC)
            if i >= 0:
                buf = buf[i:]                      # drop CLI echo bytes
                if len(buf) >= HEADER.size:
                    expected = HEADER.size + payload_nbytes(parse_header(buf))
        elif len(buf) >= expected:
            break
    return buf[:expected] if expected else buf, time.time() - t0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="CLI/data port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=V3_BAUD)
    ap.add_argument("--n", type=int, default=5, help="number of dumps")
    ap.add_argument("--no-config", action="store_true")
    ap.add_argument("--save-last", action="store_true",
                    help="save the final dump to ~/openflight_shots/")
    a = ap.parse_args()

    if a.port:
        ser, port = l3host.open_port(a.port, a.baud), a.port
        if "sensorStart" not in l3host.cmd(ser, "help"):
            print(f"no CLI on {port} @ {a.baud} — v3 flashed?")
            return 1
    else:
        ser, port = find_cli(a.baud)
        if not ser:
            print(f"no CLI found @ {a.baud} — v3 flashed? (v2 speaks 115200)")
            return 1
    print(f"CLI+data on {port} @ {a.baud}")

    if not a.no_config:
        print("configuring sensor...")
        l3host.send_config(ser)
        print("sensor running.")

    results = []
    raw = b""
    for k in range(a.n):
        raw, dt = timed_dump(ser)
        n = len(raw)
        ok = "FULL" if n and n == 20 + ((n - 20) // 131072) * 131072 and \
            (n - 20) // 131072 == parse_header(raw)["n_frames"] else "SHORT"
        rate = n * 10 / dt / 1000 if dt else 0
        results.append((n, dt, ok))
        line = f"dump {k+1}: {n:7d} B in {dt:5.2f} s = {n/dt/1024:6.1f} KiB/s " \
               f"(~{rate:.0f} kbaud effective)  {ok}"
        if ok == "FULL":
            body = np.frombuffer(raw, dtype="<i2", offset=20).astype(float)
            absmax = float(np.abs(body).max())
            line += f"  absmax {absmax:.0f}  slot {parse_header(raw)['trigger_frame']}"
        print(line)
        l3host.cmd(ser, "stats", 2.0)
        time.sleep(0.5)

    full = [r for r in results if r[2] == "FULL"]
    print(f"\n{len(full)}/{a.n} full dumps; " +
          (f"median {np.median([r[1] for r in full]):.2f} s "
           f"(v2 @460800 was ~17 s)" if full else "ALL SHORT — check baud/wiring"))
    if a.save_last and raw:
        fn = os.path.expanduser("~/openflight_shots/uart_speed_last.l3dump")
        open(fn, "wb").write(raw)
        print(f"saved -> {fn}")
    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
