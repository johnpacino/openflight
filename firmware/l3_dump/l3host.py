"""Shared host-side helpers for the L3-dump bench/test harnesses.

Used by shot_trigger.py (Mac mic trigger) and clap_test.py (Pi GPIO trigger).
Firmware v3 contract (single-port): CLI commands AND the dump share UARTA =
the CP2105 Enhanced interface at 1,041,667 baud (exact divisor IWR-side,
+0.17% CP2105-side); the dump is framed by its "ILD1" magic + header length.
Dump geometry: n_frames x 64 chirps x 4 rx x 128 samples, ImRe int16 pairs
(frame count read from the header — 5 in v1/v2, 6 in v2+).
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

BAUD = 1041667                # v3 single-port rate (CLI + dump on one UART)
CLI_BAUD = BAUD               # back-compat aliases: same port, same rate
DATA_BAUD = BAUD
FULL_DUMP = 20 + 6 * 131072   # header + 6 frames (fallback; header is truth)
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
    """(cli, data) — with v3 single-port firmware BOTH are the same device
    (the port answering `help` at BAUD). Kept as a pair for caller compat.
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
            return p, p
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

    Syncs on the "ILD1" magic (on the shared v3 port the CLI's command echo
    precedes the binary payload) and sizes the read from the dump's own
    header, so it works across firmware versions. Falls back to FULL_DUMP
    if a header never parses.
    """
    from iwr6843_l3dump import HEADER, MAGIC, parse_header, payload_nbytes
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
        elif buf and time.time() - last > 4.0:
            # generous: the CP2105 has been seen to stall the stream for
            # >1.5 s mid-dump (cp210x -110 control timeouts) and then resume
            break
        if expected is None:
            i = buf.find(MAGIC)
            if i >= 0 and len(buf) - i >= HEADER.size:
                buf = buf[i:]                    # drop CLI echo before magic
                try:
                    expected = HEADER.size + payload_nbytes(parse_header(buf))
                except ValueError:
                    expected = FULL_DUMP
        elif len(buf) >= expected:
            break
    return buf if expected is None else buf[:expected]


LOOP_PRI_S = 90e-6            # per-TX chirp repeat (45 us x 2 TDM chirps);
                              # identical across v3/B/C variants
RANGE_SPAN_M = 6.0            # all cfg variants keep a 6 m unambiguous span,
                              # so bin size = 6.0 / n_samples
# Ball-candidate gates in METERS (converted per-dump): the golfer/club blob
# (1.5-2.3 m on the 7i session) wanders at 10-20 m/s and out-votes the
# fainter ball if included.
BALL_GATES_M = ((2.25, 3.75), (3.75, 5.5))


def _mti_loop_profiles(cube: np.ndarray) -> np.ndarray:
    """Raw cube [nf,cpf,4,ns] -> MTI residual power per loop [nf*loops, ns].

    TDM split (even chirps TX0 / odd TX1), range FFT, then subtract each
    (frame,tx,rx,bin)'s mean over the loops: static clutter cancels, movers
    survive. The static argmax is useless for the ball -- clutter always wins.
    """
    nf, cpf, nrx, ns = cube.shape
    tdm = cube.reshape(nf, cpf // 2, 2, nrx, ns).transpose(0, 2, 1, 3, 4)
    rfft = np.fft.fft(tdm, axis=-1)
    mti = rfft - rfft.mean(axis=2, keepdims=True)
    return (np.abs(mti) ** 2).sum(axis=(1, 3)).reshape(nf * (cpf // 2), ns)


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
    from iwr6843_l3dump import parse_header
    try:
        meta = parse_header(raw)
    except Exception:  # noqa: BLE001
        print("  !! dump header unparseable")
        return
    cpf, ns, nrx = meta["chirps_per_frame"], meta["n_samples"], meta["n_rx"]
    nloops = cpf // 2
    frame_bytes = cpf * nrx * ns * 4
    nf = (len(raw) - 20) // frame_bytes
    if nf < 1:
        print("  !! dump too short to analyse")
        return
    res = RANGE_SPAN_M / ns                       # 4.7 cm @128, 9.4 cm @64
    frame_s = (meta["frame_period_us"] / 1e6
               if meta["version"] >= 3 and meta["frame_period_us"] else FRAME_S)
    body = np.frombuffer(raw, dtype="<i2", offset=20,
                         count=nf * frame_bytes // 2).astype(float)
    cube = (body[1::2] + 1j * body[0::2]).reshape(nf, cpf, nrx, ns)   # ImRe
    print(f"  {nf} frames x {nloops} loops, {ns} samp ({res*100:.1f} cm bins), "
          f"{frame_s*1e3:.0f} ms period  absmax {np.abs(cube).max():.0f}")
    if meta["version"] >= 2:
        print(f"  header: time order starts slot {meta['trigger_frame']}")

    prof = _mti_loop_profiles(cube)
    nloop_tot = nf * nloops
    lo_bin = max(3, int(0.35 / res))

    # per-slot strongest mover (context: golfer/club sits here, quasi-static)
    slot_pow = prof.reshape(nf, nloops, ns).sum(axis=1)
    for f in range(nf):
        b = lo_bin + int(np.argmax(slot_pow[f, lo_bin:]))
        print(f"  slot {f}: mover peak {b * res:4.2f} m  P {slot_pow[f, b]:.2e}")

    # candidate detections: per-loop argmax in each sub-gate, SNR-gated
    gates = [(int(a / res), min(int(b / res), ns - 2)) for a, b in BALL_GATES_M]
    ts, bs = [], []
    for g0, g1 in gates:
        g = prof[:, g0:g1]
        base = np.median(g, axis=1) + 1e-12
        idx = np.argmax(g, axis=1)
        snr = g[np.arange(nloop_tot), idx] / base
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
        slot_t = {s: ((s - r) % nf) * frame_s for s in range(nf)}
        t = np.array([slot_t[i // nloops] + (i % nloops) * LOOP_PRI_S
                      for i in loop_i])
        for _ in range(1500):
            i, j = rng.choice(t.size, 2, replace=False)
            dt = t[i] - t[j]
            if abs(dt) < 3e-3:
                continue
            slope = (bins[i] - bins[j]) / dt
            v = slope * res
            if not 20 <= v <= 90:
                continue
            icpt = bins[i] - slope * t[i]
            inl = np.abs(bins - (slope * t + icpt)) < 1.2
            if inl.sum() >= 8 and (best is None or inl.sum() > best[0]):
                A = np.vstack([t[inl], np.ones(inl.sum())]).T
                (sl, ic), *_ = np.linalg.lstsq(A, bins[inl], rcond=None)
                if not 20 <= sl * res <= 90:   # refit drifted out
                    continue
                rms = float(np.sqrt(((bins[inl] - (sl * t[inl] + ic)) ** 2).mean()))
                best = (int(inl.sum()), r, sl, ic, rms,
                        t[inl].min(), t[inl].max())
    if best is None:
        print("  (no ball streak: no 20-90 m/s range walk found)")
        return
    ninl, r, sl, ic, rms, t0, t1 = best
    v = sl * res
    r0, r1 = (sl * t0 + ic) * res, (sl * t1 + ic) * res
    conf = "" if (rms < 0.45 and (t1 - t0) >= 0.012) else "   [LOW CONF]"
    print(f"  >>> BALL {v:5.1f} m/s = {v * 2.237:5.1f} mph   "
          f"seen {r0:.2f}->{r1:.2f} m over {(t1 - t0) * 1e3:.0f} ms   "
          f"({ninl} pts, rms {rms:.2f} bins, time order starts slot {r}){conf}")
