#!/usr/bin/env python3
"""IWR6843 out-of-box demo UART parser (Stage 1 prep).

INTENT (for future sessions / Opus 4.8)
---------------------------------------
Written BEFORE the IWR6843LEVM arrived, validated only against synthetic
bytes (see stage1_dryrun.py and tests/test_iwr6843_uart.py). Expected
outcome when hardware arrives:

  1. `SerialReader` connects to the LEVM's two CP2105 ports, sends a .cfg
     to the CLI port, and yields parsed frames from the data port.
  2. `parse_azimuth_heatmap()` returns the per-virtual-antenna complex
     vector that feeds FBSS-MUSIC (music_stage0_sim.est_music_fbss).
     On the ROTATED board this "azimuth" axis is ELEVATION (launch angle).

Assumptions that MUST be verified against the first real frame (each is
flagged inline with `VERIFY-ON-HW`):
  A1. TLV `length` excludes the 8-byte TLV header (SDK 3.x convention).
  A2. Azimuth heatmap sample order is imag(int16) then real(int16).
  A3. Virtual antenna order is TX-major, RX-minor (TX1:RX1..4, TX3:RX1..4)
      giving 8 contiguous lambda/2 positions. Check: an off-boresight
      reflector must show a LINEAR phase ramp across the 8 antennas.
If a real frame violates A1, parse_frame() logs a reconciliation warning
rather than crashing — fix the convention here, rerun the dry run, and the
downstream pipeline is unaffected.

No hardware imports at module level: numpy only, so tests run anywhere.
pyserial is imported lazily inside SerialReader.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Iterator

import numpy as np

MAGIC = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])
FRAME_HEADER_LEN = 40          # magic(8) + 8 x uint32
TLV_HEADER_LEN = 8             # type(u32) + length(u32)

# TLV types emitted by the SDK 3.x xwr68xx mmw demo (guiMonitor selects them)
TLV_DETECTED_POINTS = 1        # numDetObj x (x, y, z, doppler) float32
TLV_RANGE_PROFILE = 2          # numRangeBins x uint16 (log2 magnitude, Q9)
TLV_NOISE_PROFILE = 3
TLV_AZIMUTH_STATIC_HEATMAP = 4  # zero-Doppler per-antenna complex — S1-A input
TLV_RANGE_DOPPLER_HEATMAP = 5
TLV_STATS = 6
TLV_SIDE_INFO = 7              # numDetObj x (snr, noise) int16, 0.1 dB units
TLV_TEMPERATURE_STATS = 9      # SDK 3.5 emits this alongside STATS when
                               # statsInfo=1 (seen on real 6843 frames 2026-07-07)

N_VIRT_ANT = 8                 # LEVM azimuth line: TX1+TX3 x RX1-4 (SWRU585 Fig 4-2)

# VERIFY-ON-HW (A3): mapping from stream order to physical lambda/2 position.
# Identity until proven otherwise by the phase-ramp check in the bring-up doc.
VIRT_ANT_ORDER = list(range(N_VIRT_ANT))


@dataclass
class FrameHeader:
    version: int
    total_packet_len: int
    platform: int
    frame_number: int
    time_cpu_cycles: int
    num_detected_obj: int
    num_tlvs: int
    subframe_number: int


@dataclass
class Frame:
    header: FrameHeader
    tlvs: dict          # {tlv_type: raw payload bytes}
    ok: bool            # False if length reconciliation failed (A1 suspect)


def find_magic(buf: bytes, start: int = 0) -> int:
    """Index of next frame magic word, or -1."""
    return buf.find(MAGIC, start)


def parse_frame_header(buf: bytes, offset: int = 0) -> FrameHeader:
    fields = struct.unpack_from("<8I", buf, offset + len(MAGIC))
    return FrameHeader(*fields)


def parse_frame(buf: bytes, offset: int = 0) -> tuple[Frame | None, int]:
    """Parse one frame at `offset` (which must point at MAGIC).

    Returns (frame, next_offset). frame is None if the buffer doesn't yet
    contain the complete packet (caller should read more bytes and retry).
    """
    if len(buf) - offset < FRAME_HEADER_LEN:
        return None, offset
    header = parse_frame_header(buf, offset)
    if len(buf) - offset < header.total_packet_len:
        return None, offset

    tlvs: dict[int, bytes] = {}
    pos = offset + FRAME_HEADER_LEN
    end = offset + header.total_packet_len
    ok = True
    for _ in range(header.num_tlvs):
        if pos + TLV_HEADER_LEN > end:
            ok = False        # ran past packet end -> A1 convention suspect
            break
        tlv_type, tlv_len = struct.unpack_from("<II", buf, pos)
        payload_start = pos + TLV_HEADER_LEN
        # VERIFY-ON-HW (A1): SDK 3.x length = payload only. If real frames
        # reconcile poorly, try tlv_len - TLV_HEADER_LEN here (SDK 1.x style).
        if payload_start + tlv_len > end:
            ok = False
            break
        tlvs[tlv_type] = buf[payload_start:payload_start + tlv_len]
        pos = payload_start + tlv_len
    return Frame(header, tlvs, ok), offset + header.total_packet_len


def iter_frames(buf: bytes) -> Iterator[Frame]:
    """Iterate complete frames in a byte buffer (e.g. a saved raw_uart.bin)."""
    offset = 0
    while True:
        offset = find_magic(buf, offset)
        if offset < 0:
            return
        frame, next_offset = parse_frame(buf, offset)
        if frame is None:
            return            # incomplete tail
        yield frame
        offset = next_offset


# ------------------------------------------------------------- TLV decoders

def parse_detected_points(payload: bytes) -> np.ndarray:
    """(N, 4) float32: x, y, z [m], doppler [m/s]. Demo axes: x=lateral,
    y=downrange, z=up — on the ROTATED board x and z swap roles."""
    return np.frombuffer(payload, dtype=np.float32).reshape(-1, 4)


def parse_range_profile(payload: bytes) -> np.ndarray:
    """uint16 log2-magnitude (Q9) per range bin."""
    return np.frombuffer(payload, dtype=np.uint16)


def parse_azimuth_heatmap(payload: bytes, n_virt_ant: int = N_VIRT_ANT) -> np.ndarray:
    """Zero-Doppler per-antenna complex matrix, shape (numRangeBins, n_virt_ant).

    This is THE Stage-1a data product: heatmap[k] is the 8-element complex
    snapshot for range bin k -> feed straight into est_music_fbss().
    VERIFY-ON-HW (A2): int16 pairs are (imag, real) per SDK docs.
    """
    raw = np.frombuffer(payload, dtype=np.int16).astype(np.float32)
    raw = raw.reshape(-1, n_virt_ant, 2)
    heatmap = (raw[:, :, 1] + 1j * raw[:, :, 0]).astype(np.complex64)
    return heatmap[:, VIRT_ANT_ORDER]


def parse_side_info(payload: bytes) -> np.ndarray:
    """(N, 2) int16: snr, noise in 0.1 dB units — feeds Gate S1-D."""
    return np.frombuffer(payload, dtype=np.int16).reshape(-1, 2)


# --------------------------------------------------------------- live serial

class SerialReader:
    """Live capture from the LEVM's dual CP2105 UARTs.

    Usage (hardware sessions):
        rdr = SerialReader(cli_port="/dev/ttyUSB0", data_port="/dev/ttyUSB1")
        rdr.send_config("config/iwr6843_levm_static.cfg")
        for frame in rdr.frames(raw_sink="raw_uart.bin"):
            ...
    """

    def __init__(self, cli_port: str, data_port: str,
                 cli_baud: int = 115200, data_baud: int = 921600):
        import serial  # lazy: keeps tests hardware-free
        self.cli = serial.Serial(cli_port, cli_baud, timeout=0.5)
        self.data = serial.Serial(data_port, data_baud, timeout=0.05)

    def send_config(self, cfg_path: str, echo: bool = True) -> None:
        """Send a .cfg line-by-line to the demo CLI, waiting for each line's
        'Done'. Uses a bounded read window (longer for sensorStart, which runs
        calibration) so a missing/slow response can never hang the caller.
        Raises RuntimeError on an explicit error reply."""
        with open(cfg_path) as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("%"):
                    continue
                self.cli.write((line + "\n").encode())
                window = 3.0 if line.startswith("sensorStart") else 1.0
                resp = b""
                t = time.time()
                while time.time() - t < window:
                    resp += self.cli.read(512)
                    if b"Done" in resp or b"Error" in resp:
                        break
                text = resp.decode(errors="replace")
                low = text.lower()
                if echo:
                    flat = " | ".join(s.strip() for s in text.splitlines()
                                      if s.strip())
                    print(f">> {line}\n   {flat}")
                if any(k in low for k in
                       ("error", "invalid", "not recognized", "failure")):
                    raise RuntimeError(f"config rejected: {line!r} -> {text!r}")
                if "done" not in low:
                    print(f"   WARN: no 'Done' from {line!r} within {window}s "
                          "(continuing)")

    def frames(self, raw_sink: str | None = None,
               duration_s: float | None = None) -> Iterator[Frame]:
        """Yield frames; optionally tee raw bytes to a file (ALWAYS pass
        raw_sink in real sessions — see bring-up doc rule).

        duration_s bounds the total capture wall-clock. It is checked every
        read (~20 Hz) regardless of whether a frame completed, so a silent or
        stalled stream can never hang the caller (the 2026-07-07 power bug)."""
        buf = b""
        sink = open(raw_sink, "ab") if raw_sink else None
        t_start = time.time()
        try:
            while True:
                if duration_s is not None and time.time() - t_start > duration_s:
                    return
                chunk = self.data.read(4096)
                if chunk:
                    if sink:
                        sink.write(chunk)
                    buf += chunk
                start = find_magic(buf)
                if start < 0:
                    buf = buf[-len(MAGIC):]
                    continue
                frame, nxt = parse_frame(buf, start)
                if frame is None:
                    continue
                if not frame.ok:
                    print("WARN: TLV length reconciliation failed "
                          "(see A1 note in module docstring)")
                buf = buf[nxt:]
                yield frame
        finally:
            if sink:
                sink.close()


# --------------------------------------------------- synthetic frame builder
# Used by tests and stage1_dryrun.py. Kept here so the byte conventions have
# exactly one home: if a VERIFY-ON-HW assumption changes, fix it once and
# both the parser and the synthesizer stay consistent.

def build_azimuth_heatmap_payload(heatmap: np.ndarray) -> bytes:
    """Inverse of parse_azimuth_heatmap (imag first, int16)."""
    n_bins, n_ant = heatmap.shape
    out = np.empty((n_bins, n_ant, 2), dtype=np.int16)
    out[:, :, 0] = np.round(heatmap.imag)
    out[:, :, 1] = np.round(heatmap.real)
    return out.tobytes()


def build_frame(tlvs: dict[int, bytes], frame_number: int = 1) -> bytes:
    """Pack TLVs into a demo-format frame (SDK 3.x conventions)."""
    body = b""
    for tlv_type, payload in tlvs.items():
        body += struct.pack("<II", tlv_type, len(payload)) + payload
    total = FRAME_HEADER_LEN + len(body)
    header = MAGIC + struct.pack(
        "<8I", 0x03060000, total, 0xA6843, frame_number, 0,
        0, len(tlvs), 0)
    return header + body
