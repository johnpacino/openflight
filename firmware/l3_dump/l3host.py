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
# MUST match frameCfg periodicity in config/iwr6843_l3dump.cfg — the ball-speed
# fit times cross-frame range walk with it. 0.008 before 2026-07-12 evening
# (dumps clap_001-024), 0.012 after (60 ms window).
FRAME_S = 0.012
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


def read_dump_besteffort(data: serial.Serial, timeout_s: float = 40.0) -> bytes:
    """Greedy in_waiting drain; returns whatever arrives (tail loss tolerated).

    Expected size comes from the dump's own header (n_frames etc.), so this
    works across firmware versions (v1 = 5 frames, v2 = 6). Falls back to
    FULL_DUMP if the header doesn't parse.
    """
    from iwr6843_l3dump import HEADER, parse_header, payload_nbytes
    buf = b""
    expected = None
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
        if expected is None and len(buf) >= HEADER.size:
            try:
                expected = HEADER.size + payload_nbytes(parse_header(buf))
            except ValueError:
                expected = FULL_DUMP
        if expected and len(buf) >= expected:
            break
    return buf


LOOP_PRI_S = 90e-6            # per-TX chirp repeat (45 us x 2 TDM chirps)
# Ball-candidate gates start at 2.25 m: the golfer/club blob (1.5-2.3 m on the
# 7i session) wanders at 10-20 m/s and out-votes the fainter ball if included.
BALL_GATES = ((48, 80), (80, 118))             # 2.25-3.75, 3.75-5.5 m


def _mti_loop_profiles(cube: np.ndarray) -> np.ndarray:
    """Raw cube [nf,64,4,128] -> MTI residual power per loop [nf*32, 128].

    TDM split (even chirps TX0 / odd TX1), range FFT, then subtract each
    (frame,tx,rx,bin)'s mean over the 32 loops: static clutter cancels, movers
    survive. The static argmax is useless for the ball -- clutter always wins.
    """
    nf = cube.shape[0]
    tdm = cube.reshape(nf, 32, 2, 4, 128).transpose(0, 2, 1, 3, 4)
    rfft = np.fft.fft(tdm, axis=-1)
    mti = rfft - rfft.mean(axis=2, keepdims=True)
    return (np.abs(mti) ** 2).sum(axis=(1, 3)).reshape(nf * 32, 128)


def expected_len(raw: bytes) -> int:
    """Full dump length per the header; FULL_DUMP if it doesn't parse."""
    from iwr6843_l3dump import HEADER, parse_header, payload_nbytes
    try:
        return HEADER.size + payload_nbytes(parse_header(raw))
    except Exception:  # noqa: BLE001
        return FULL_DUMP


def analyse(raw: bytes) -> None:
    """Movers after clutter removal + RANSAC ball-speed fit on the range walk.

    Doppler is useless for ball speed here (a ~50 m/s ball aliases to ~+1 m/s
    with this chirp timing); the range walk across the 40 ms ring is
    unambiguous. Ring slots are streamed in MEMORY order = a rotation of time
    order, so the fit tries all 5 rotations and keeps the best line.
    Validated against the 2026-07-12 7i session (105/112/108 mph tracks).
    """
    nf = (len(raw) - 20) // 131072
    if nf < 1:
        print("  !! dump too short to analyse")
        return
    body = np.frombuffer(raw, dtype="<i2", offset=20, count=nf * 65536).astype(float)
    cube = (body[1::2] + 1j * body[0::2]).reshape(nf, 64, 4, 128)   # ImRe
    print(f"  {nf} frames  absmax {np.abs(cube).max():.0f}")
    try:
        from iwr6843_l3dump import parse_header
        meta = parse_header(raw)
        if meta["version"] >= 2:
            print(f"  header: time order starts slot {meta['trigger_frame']}")
    except ValueError:
        pass

    prof = _mti_loop_profiles(cube)
    nloop = nf * 32

    # per-slot strongest mover (context: golfer/club sits here, quasi-static)
    slot_pow = prof.reshape(nf, 32, 128).sum(axis=1)
    for f in range(nf):
        b = 8 + int(np.argmax(slot_pow[f, 8:]))
        print(f"  slot {f}: mover peak {b * RANGE_RES_M:4.2f} m  "
              f"P {slot_pow[f, b]:.2e}")

    # candidate detections: per-loop argmax in each sub-gate, SNR-gated
    ts, bs = [], []
    for g0, g1 in BALL_GATES:
        g = prof[:, g0:g1]
        base = np.median(g, axis=1) + 1e-12
        idx = np.argmax(g, axis=1)
        snr = g[np.arange(nloop), idx] / base
        for i in np.nonzero(snr > 4.0)[0]:
            b = float(g0 + idx[i])
            j = int(b)
            if g0 < j < g1 - 1:            # parabolic sub-bin refine
                y0, y1, y2 = prof[i, j - 1], prof[i, j], prof[i, j + 1]
                den = y0 - 2 * y1 + y2
                if den < 0 and abs((y0 - y2) / (2 * den)) < 1:
                    b += (y0 - y2) / (2 * den)
            ts.append(i)
            bs.append(b)
    if len(ts) < 8:
        print("  (no ball streak: too few moving detections)")
        return
    loop_i = np.array(ts)
    bins = np.array(bs)

    # try all ring rotations; RANSAC a line with an outbound-ball slope
    best = None
    rng = np.random.default_rng(1)
    for r in range(nf):
        slot_t = {s: ((s - r) % nf) * FRAME_S for s in range(nf)}
        t = np.array([slot_t[i // 32] + (i % 32) * LOOP_PRI_S for i in loop_i])
        for _ in range(1500):
            i, j = rng.choice(t.size, 2, replace=False)
            dt = t[i] - t[j]
            if abs(dt) < 3e-3:
                continue
            slope = (bins[i] - bins[j]) / dt
            v = slope * RANGE_RES_M
            if not 20 <= v <= 90:
                continue
            icpt = bins[i] - slope * t[i]
            inl = np.abs(bins - (slope * t + icpt)) < 1.2
            if inl.sum() >= 8 and (best is None or inl.sum() > best[0]):
                A = np.vstack([t[inl], np.ones(inl.sum())]).T
                (sl, ic), *_ = np.linalg.lstsq(A, bins[inl], rcond=None)
                if not 20 <= sl * RANGE_RES_M <= 90:   # refit drifted out
                    continue
                rms = float(np.sqrt(((bins[inl] - (sl * t[inl] + ic)) ** 2).mean()))
                best = (int(inl.sum()), r, sl, ic, rms,
                        t[inl].min(), t[inl].max())
    if best is None:
        print("  (no ball streak: no 20-90 m/s range walk found)")
        return
    ninl, r, sl, ic, rms, t0, t1 = best
    v = sl * RANGE_RES_M
    r0, r1 = (sl * t0 + ic) * RANGE_RES_M, (sl * t1 + ic) * RANGE_RES_M
    conf = "" if (rms < 0.45 and (t1 - t0) >= 0.012) else "   [LOW CONF]"
    print(f"  >>> BALL {v:5.1f} m/s = {v * 2.237:5.1f} mph   "
          f"seen {r0:.2f}->{r1:.2f} m over {(t1 - t0) * 1e3:.0f} ms   "
          f"({ninl} pts, rms {rms:.2f} bins, time order starts slot {r}){conf}")
