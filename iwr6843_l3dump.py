"""Pi-side pipeline for the Stage-1c L3 raw-ADC burst-dump (see
docs/plans/stage1c_l3_burst_dump.md).

Contract (firmware <-> Pi), little-endian:
  header (20 B): magic 'ILD1', u16 version, u16 n_frames, u16 chirps_per_frame,
                 u8 n_tx, u8 n_rx, u16 n_samples, u8 sample_fmt (0=int16 I/Q),
                 u8 pad, u16 trigger_frame, u16 pad2
  payload: per frame, per chirp, per rx: n_samples x (int16 I, int16 Q)

Chirp order in a frame is TDM-interleaved: chirp c -> tx = c % n_tx,
loop = c // n_tx. The rotated-board elevation virtual array is
[tx0.rx0..rx(nrx-1), tx1.rx0..] = n_tx*n_rx lambda/2 elements.

Pipeline: parse_dump -> range_fft -> ball_range_bin -> virtual_snapshot ->
est_music_fbss (music_core) -> per-frame elevation (deg). The launch-angle
trajectory fit is downstream (two_ray / fit_launch_angle), not here.

VERIFY-ON-HW (can't be nailed without the board): exact TX/RX virtual-element
ordering, per-element phase calibration, and TDM Doppler-phase correction across
TX for a MOVING target (this module coherently combines one loop; the moving-ball
correction lands with real captures). The parse/range-FFT/MUSIC integration IS
validated here against synthetic targets (tests/test_l3dump.py).
"""
from __future__ import annotations

import struct

import numpy as np

from music_core import est_music_fbss, steer

MAGIC = b"ILD1"
HEADER = struct.Struct("<4sHHHBBHBBHH")
SAMPLE_INT16_IQ = 0


def pack_dump(cube: np.ndarray, *, n_tx: int, trigger_frame: int = 0,
              version: int = 1) -> bytes:
    """Complex cube [n_frames, chirps_per_frame, n_rx, n_samples] -> dump bytes.

    Reference packer: this is the exact byte layout the firmware must emit, and
    the synthesis/test path uses it so the format has one executable definition.
    """
    n_frames, cpf, n_rx, n_samples = cube.shape
    hdr = HEADER.pack(MAGIC, version, n_frames, cpf, n_tx, n_rx, n_samples,
                      SAMPLE_INT16_IQ, 0, trigger_frame, 0)
    flat = cube.reshape(-1)
    iq = np.empty(flat.size * 2, dtype="<i2")
    iq[0::2] = np.clip(np.round(flat.real), -32768, 32767).astype("<i2")
    iq[1::2] = np.clip(np.round(flat.imag), -32768, 32767).astype("<i2")
    return hdr + iq.tobytes()


def parse_dump(raw: bytes):
    """Dump bytes -> (meta dict, complex cube [n_frames, cpf, n_rx, n_samples])."""
    (magic, ver, nf, cpf, ntx, nrx, ns,
     fmt, _pad, trig, _pad2) = HEADER.unpack_from(raw, 0)
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic!r} (expected {MAGIC!r})")
    if fmt != SAMPLE_INT16_IQ:
        raise ValueError(f"unsupported sample_fmt {fmt}")
    n = nf * cpf * nrx * ns
    body = np.frombuffer(raw, dtype="<i2", offset=HEADER.size, count=2 * n)
    if body.size < 2 * n:
        raise ValueError(f"short payload: {body.size} i16 < {2 * n} needed")
    iq = body.astype(np.float64)
    cube = (iq[0::2] + 1j * iq[1::2]).reshape(nf, cpf, nrx, ns)
    meta = dict(version=ver, n_frames=nf, chirps_per_frame=cpf, n_tx=ntx,
                n_rx=nrx, n_samples=ns, trigger_frame=trig)
    return meta, cube


def range_fft(cube: np.ndarray, n_fft: int | None = None) -> np.ndarray:
    """FFT over the ADC-sample axis -> [n_frames, cpf, n_rx, n_range]."""
    n_fft = n_fft or cube.shape[-1]
    return np.fft.fft(cube, n=n_fft, axis=-1)


def ball_range_bin(rfft: np.ndarray, frame: int, *,
                   lo_bin: int = 1, hi_bin: int | None = None) -> int:
    """Strongest range bin in a gate (power summed over all chirps + RX)."""
    hi_bin = hi_bin or rfft.shape[-1] // 2
    power = np.sum(np.abs(rfft[frame, :, :, lo_bin:hi_bin]) ** 2, axis=(0, 1))
    return lo_bin + int(np.argmax(power))


def virtual_snapshot(rfft: np.ndarray, frame: int, range_bin: int,
                     n_tx: int, n_rx: int, loop: int = 0) -> np.ndarray:
    """n_tx*n_rx-element complex snapshot at (frame, range_bin) for one loop.

    Elements ordered [tx0.rx0.., tx1.rx0.., ...] = the rotated elevation ULA.
    """
    snap = np.empty(n_tx * n_rx, dtype=complex)
    for tx in range(n_tx):
        chirp = loop * n_tx + tx
        for rx in range(n_rx):
            snap[tx * n_rx + rx] = rfft[frame, chirp, rx, range_bin]
    return snap


def frame_elevation(rfft: np.ndarray, frame: int, *, n_tx: int, n_rx: int,
                    noise_var: float = 1.0, lo_bin: int = 1,
                    hi_bin: int | None = None):
    """Per-frame elevation (deg) at the frame's strongest range bin.

    Returns (elev_deg, range_bin). noise_var scales the MUSIC source-count
    threshold; pass an estimate of the per-element noise power.
    """
    rb = ball_range_bin(rfft, frame, lo_bin=lo_bin, hi_bin=hi_bin)
    snap = virtual_snapshot(rfft, frame, rb, n_tx, n_rx)
    theta = est_music_fbss(snap, noise_var)
    return float(np.degrees(theta)), rb


# --- synthesis (tests + executable spec of the firmware format) -------------
def synth_target(elev_deg: float, range_bin: int, *, n_frames: int = 1,
                 n_tx: int = 2, n_rx: int = 4, n_samples: int = 128,
                 amp: float = 1000.0, noise: float = 0.0, rng=None) -> np.ndarray:
    """Raw-ADC cube for ONE static point target at (elev_deg, range_bin).

    Beat tone exp(j*2pi*range_bin*n/n_samples) -> range-FFT peaks at range_bin;
    steer(theta, n_tx*n_rx) sets the elevation phase across the virtual array.
    Returns cube [n_frames, n_tx (=cpf, 1 loop), n_rx, n_samples].
    """
    rng = np.random.default_rng(0) if rng is None else rng
    n_el = n_tx * n_rx
    a = steer(np.radians(elev_deg), n_el)
    n = np.arange(n_samples)
    tone = np.exp(2j * np.pi * range_bin * n / n_samples)
    cube = np.zeros((n_frames, n_tx, n_rx, n_samples), dtype=complex)
    for f in range(n_frames):
        for tx in range(n_tx):
            for rx in range(n_rx):
                cube[f, tx, rx] = amp * a[tx * n_rx + rx] * tone
    if noise:
        cube += (rng.standard_normal(cube.shape)
                 + 1j * rng.standard_normal(cube.shape)) * noise
    return cube
