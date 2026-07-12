"""Shared host-side helpers for the L3-dump bench/test harnesses.

Used by shot_trigger.py (Mac mic trigger) and clap_test.py (Pi GPIO trigger).
Knows the firmware contract: CLI UART 115200 (commands, DTR/RTS-safe open),
data UART 460800 (dump stream), config/iwr6843_l3dump.cfg, and the dump
geometry (5 frames x 64 chirps x 4 rx x 128 samples, ImRe int16 pairs).
"""
from __future__ import annotations

import glob
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import numpy as np       # noqa: E402
import serial            # noqa: E402

CLI_BAUD = 115200
DATA_BAUD = 460800
FULL_DUMP = 20 + 5 * 131072   # header + 5 frames
RANGE_RES_M = 0.0469
FRAME_S = 0.008
CFG_PATH = os.path.join(_ROOT, "config", "iwr6843_l3dump.cfg")


def open_port(port: str, baud: int, timeout: float = 0.3) -> serial.Serial:
    """DTR/RTS-safe open (TI EVMs tie those lines to reset/boot-mode)."""
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, timeout
    s.dtr = False
    s.rts = False
    s.open()
    return s


def cmd(cli: serial.Serial, line: str, window: float = 1.5) -> str:
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
    """(cli, data) among the CP2105 pair; CLI = the port answering `help`.
    macOS: /dev/tty.SLAB_USBtoUART*; Linux/Pi: /dev/ttyUSB*."""
    candidates = (sorted(glob.glob("/dev/tty.SLAB_USBtoUART*"))
                  or sorted(glob.glob("/dev/ttyUSB*")))
    for p in candidates:
        try:
            s = open_port(p, CLI_BAUD)
        except Exception:  # noqa: BLE001
            continue
        r = cmd(s, "help")
        s.close()
        if "sensorStart" in r:
            return p, next((q for q in candidates if q != p), None)
    return None, None


def send_config(cli: serial.Serial, cfg_path: str = CFG_PATH,
                echo: bool = False) -> None:
    """sensorStop, then send the cfg (ends in sensorStart). Raises on Error."""
    cmd(cli, "sensorStop", 3.0)
    for rawline in open(cfg_path):
        line = rawline.strip()
        if not line or line.startswith("%"):
            continue
        resp = cmd(cli, line, 6.0 if line.startswith("sensorStart") else 1.5)
        if echo:
            flat = " | ".join(x.strip() for x in resp.splitlines() if x.strip())
            print(f">> {line}\n   {flat}")
        if "Error" in resp:
            raise RuntimeError(f"config rejected: {line!r}")


def read_dump_besteffort(data: serial.Serial, timeout_s: float = 30.0) -> bytes:
    """Greedy in_waiting drain; returns whatever arrives (tail loss tolerated)."""
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
    """Per-frame range peaks + crude outbound range-walk speed."""
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
        v = float(np.mean(walk)) * RANGE_RES_M / FRAME_S
        if np.all(walk >= 1):
            print(f"  >>> outbound range walk: ~{v:.1f} m/s ({v * 2.237:.0f} mph)")
        else:
            print(f"  (no clean outbound walk; peak deltas {list(walk)})")
