"""Decode IWR6843 rolling-ring captures on the Raspberry Pi.

Contract (firmware <-> Pi), little-endian:
  header (20 B): magic 'ILD1', u16 version, u16 n_frames, u16 chirps_per_frame,
                 u8 n_tx, u8 n_rx, u16 n_samples, u8 sample_fmt (0=int16 I/Q),
                 u8 pad, u16 trigger_frame, u16 pad2
  payload: per frame, per chirp, per rx: complex (int16 Q, int16 I)
  (TI ADCBUF native complex order is IMAG-first ["ImRe"] -- VERIFIED ON HW
  2026-07-12: parsing Re-first put the ceiling/hand at negative range bins)

Chirp order in a frame is TDM-interleaved: chirp c -> tx = c % n_tx,
loop = c // n_tx. The rotated-board elevation virtual array is
[tx0.rx0..rx(nrx-1), tx1.rx0..] = n_tx*n_rx lambda/2 elements.

Version 5 stores a (start bin, valid bin count) pair for each frame, followed
by only that frame's valid bins. Version 6 adds the elapsed microseconds since
the previous retained frame, allowing dense pre-impact frames and decimated
post-impact frames in one capture. The parser expands variable-width frames to
the header-declared maximum width and leaves the invalid tail as zeros. Earlier
dump versions remain parseable so recorded sessions can still be replayed.
Fixed-width version 5 and configurable-capture version 7 append a temperature
report. The older variable-width v5/v6 formats remain parseable without one.
Experimental sample_fmt=5 keeps the version 6 timed descriptors but stores each
frame as int8 I/Q plus a uint16 frame scale.
Versions 8/9 append one fixed-width on-chip shadow candidate per retained frame;
version 9 also carries the temperature extension.
"""

from __future__ import annotations

import struct

import numpy as np

from openflight.iwr6843.music import est_music_fbss, steer

MAGIC = b"ILD1"
HEADER = struct.Struct("<4sHHHBBHBBHH")
TEMP_REPORT = struct.Struct("<Ihhhhhhhhhh")
SHADOW_CANDIDATE = struct.Struct("<BBBBIhhhhhhhh")
MAX_SUPPORTED_DUMP_VERSION = 9
# TI mmWaveLink rlRfTempData_t temperature fields are signed, 1 LSB = 1 deg C.
TEMP_REPORT_KEYS = (
    "device_time_ms",
    "rx0_c",
    "rx1_c",
    "rx2_c",
    "rx3_c",
    "tx0_c",
    "tx1_c",
    "tx2_c",
    "pm_c",
    "dig0_c",
    "dig1_c",
)
SAMPLE_INT16_IQ = 0
SAMPLE_RANGE_FFT_IQ16 = 1
SAMPLE_RANGE_FFT_IQ16_WINDOWED = 2
SAMPLE_RANGE_FFT_IQ16_VARIABLE = 3
SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED = 4
SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED = 5
TIMED_FRAME_DESCRIPTOR = struct.Struct("<BBH")

_VARIABLE_SAMPLE_FORMATS = (
    SAMPLE_RANGE_FFT_IQ16_VARIABLE,
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
)
_TIMED_SAMPLE_FORMATS = (
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
)


def _has_temperature_extension(version: int, sample_fmt: int) -> bool:
    """Identify schemas that append the temperature report after the header."""
    return version in (7, 9) or (version == 5 and sample_fmt not in _VARIABLE_SAMPLE_FORMATS)


def _has_shadow_extension(version: int) -> bool:
    """Versions 8/9 carry one fixed-width candidate for every retained frame."""
    return version in (8, 9)


def _pack_shadow_candidates(candidates, n_frames: int) -> bytes:
    if candidates is None or len(candidates) != n_frames:
        raise ValueError("shadow dumps require one candidate per frame")
    packed = bytearray()
    for candidate in candidates:
        rx_iq = tuple(candidate["rx_iq"])
        if len(rx_iq) != 4 or any(len(pair) != 2 for pair in rx_iq):
            raise ValueError("shadow candidate rx_iq must contain four (Im, Re) pairs")
        packed.extend(
            SHADOW_CANDIDATE.pack(
                int(bool(candidate["valid"])),
                int(candidate["range_bin"]),
                int(candidate["tx_index"]),
                int(candidate["peak_loop"]),
                int(candidate["power"]),
                *(int(value) for pair in rx_iq for value in pair),
            )
        )
    return bytes(packed)


def pack_dump(
    cube: np.ndarray,
    *,
    n_tx: int,
    trigger_frame: int = 0,
    version: int = 1,
    frame_period_us: int = 0,
    sample_fmt: int = SAMPLE_INT16_IQ,
    range_bin_start: int = 0,
    range_bin_starts: tuple[int, ...] | list[int] | None = None,
    range_bin_counts: tuple[int, ...] | list[int] | None = None,
    frame_time_offsets_us: tuple[int, ...] | list[int] | None = None,
    temperature_report: dict[str, int] | None = None,
    shadow_candidates: tuple[dict, ...] | list[dict] | None = None,
) -> bytes:
    """Complex cube [n_frames, chirps_per_frame, n_rx, n_samples] -> dump bytes.

    Reference packer: this is the exact byte layout the firmware must emit, and
    the synthesis/test path uses it so the format has one executable definition.
    """
    n_frames, cpf, n_rx, n_samples = cube.shape
    if sample_fmt not in (
        SAMPLE_INT16_IQ,
        SAMPLE_RANGE_FFT_IQ16,
        SAMPLE_RANGE_FFT_IQ16_WINDOWED,
        SAMPLE_RANGE_FFT_IQ16_VARIABLE,
        SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
    ):
        raise ValueError(f"unsupported sample_fmt {sample_fmt}")
    temp_prefix = b""
    if temperature_report is not None:
        if version < 5:
            raise ValueError("temperature reports require dump version 5+")
        try:
            temp_values = tuple(int(temperature_report[key]) for key in TEMP_REPORT_KEYS)
        except KeyError as exc:  # pragma: no cover - defensive validation
            missing = exc.args[0]
            raise ValueError(f"temperature report missing {missing!r}") from exc
        temp_prefix = TEMP_REPORT.pack(*temp_values)
    elif version >= 5:
        raise ValueError("dump version 5+ requires a temperature report")
    if _has_shadow_extension(version) and sample_fmt not in _TIMED_SAMPLE_FORMATS:
        raise ValueError("shadow candidates require a timed variable-width sample format")
    if shadow_candidates is not None and not _has_shadow_extension(version):
        raise ValueError("shadow candidates require dump version 8 or 9")
    shadow_prefix = (
        _pack_shadow_candidates(shadow_candidates, n_frames)
        if _has_shadow_extension(version)
        else b""
    )
    frame_prefix = b""
    if sample_fmt in (
        SAMPLE_RANGE_FFT_IQ16_WINDOWED,
        SAMPLE_RANGE_FFT_IQ16_VARIABLE,
        SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
    ):
        minimum_version = (
            6
            if sample_fmt in _TIMED_SAMPLE_FORMATS
            else 5
            if sample_fmt == SAMPLE_RANGE_FFT_IQ16_VARIABLE
            else 4
        )
        if version < minimum_version:
            raise ValueError(f"sample format {sample_fmt} requires dump version {minimum_version}+")
        if range_bin_starts is None or len(range_bin_starts) != n_frames:
            raise ValueError("windowed range snapshots require one start bin per frame")
        if any(start < 0 or start > 255 for start in range_bin_starts):
            raise ValueError("frame range-bin starts must fit in uint8")
        if sample_fmt in _VARIABLE_SAMPLE_FORMATS:
            if range_bin_counts is None or len(range_bin_counts) != n_frames:
                raise ValueError("variable range snapshots require one bin count per frame")
            if any(count <= 0 or count > n_samples or count > 255 for count in range_bin_counts):
                raise ValueError("frame range-bin counts must fit in uint8 and cube width")
            if sample_fmt in _TIMED_SAMPLE_FORMATS:
                if frame_time_offsets_us is None or len(frame_time_offsets_us) != n_frames:
                    raise ValueError("timed range snapshots require one time offset per frame")
                if not frame_time_offsets_us or frame_time_offsets_us[0] != 0:
                    raise ValueError("the first timed frame offset must be zero")
                deltas_us = [
                    later - earlier
                    for earlier, later in zip(frame_time_offsets_us, frame_time_offsets_us[1:])
                ]
                if any(delta <= 0 or delta > 0xFFFF for delta in deltas_us):
                    raise ValueError("timed frame offsets must increase by 1-65535 microseconds")
                frame_prefix = b"".join(
                    TIMED_FRAME_DESCRIPTOR.pack(start, count, delta)
                    for start, count, delta in zip(
                        range_bin_starts,
                        range_bin_counts,
                        (0, *deltas_us),
                    )
                )
            else:
                frame_prefix = bytes(
                    value for pair in zip(range_bin_starts, range_bin_counts) for value in pair
                )
        else:
            frame_prefix = bytes(range_bin_starts)
    pad = range_bin_start if sample_fmt == SAMPLE_RANGE_FFT_IQ16 else 0
    hdr = HEADER.pack(
        MAGIC,
        version,
        n_frames,
        cpf,
        n_tx,
        n_rx,
        n_samples,
        sample_fmt,
        pad,
        trigger_frame,
        frame_period_us,
    )
    if sample_fmt in _VARIABLE_SAMPLE_FORMATS:
        flat = np.concatenate(
            [cube[frame, ..., :count].reshape(-1) for frame, count in enumerate(range_bin_counts)]
        )
    else:
        flat = cube.reshape(-1)
    if sample_fmt == SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED:
        scales: list[int] = []
        chunks: list[bytes] = []
        for frame, count in enumerate(range_bin_counts):
            frame_flat = cube[frame, ..., :count].reshape(-1)
            max_abs = max(
                float(np.max(np.abs(frame_flat.real), initial=0.0)),
                float(np.max(np.abs(frame_flat.imag), initial=0.0)),
            )
            scale = max(1, int(np.ceil(max_abs / 127.0)))
            scales.append(scale)
            iq8 = np.empty(frame_flat.size * 2, dtype=np.int8)
            iq8[0::2] = np.clip(np.round(frame_flat.imag / scale), -128, 127).astype(np.int8)
            iq8[1::2] = np.clip(np.round(frame_flat.real / scale), -128, 127).astype(np.int8)
            chunks.append(iq8.tobytes())
        scale_prefix = np.asarray(scales, dtype="<u2").tobytes()
        return hdr + temp_prefix + frame_prefix + scale_prefix + shadow_prefix + b"".join(chunks)

    iq = np.empty(flat.size * 2, dtype="<i2")
    iq[0::2] = np.clip(np.round(flat.imag), -32768, 32767).astype("<i2")  # Im first (TI ImRe)
    iq[1::2] = np.clip(np.round(flat.real), -32768, 32767).astype("<i2")
    return hdr + temp_prefix + frame_prefix + shadow_prefix + iq.tobytes()


def parse_header(raw: bytes) -> dict:
    """Unpack the fixed header and optional v5 temperature extension.

    Lets the runtime size the burst before the full payload has arrived.
    """
    if len(raw) < HEADER.size:
        raise ValueError(f"short header: {len(raw)} bytes < {HEADER.size} needed")
    (magic, ver, nf, cpf, ntx, nrx, ns, fmt, _pad, trig, period_us) = HEADER.unpack_from(raw, 0)
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic!r} (expected {MAGIC!r})")
    if ver > MAX_SUPPORTED_DUMP_VERSION:
        raise ValueError(
            f"unsupported dump version {ver}; max supported is {MAX_SUPPORTED_DUMP_VERSION}"
        )
    if fmt not in (
        SAMPLE_INT16_IQ,
        SAMPLE_RANGE_FFT_IQ16,
        SAMPLE_RANGE_FFT_IQ16_WINDOWED,
        SAMPLE_RANGE_FFT_IQ16_VARIABLE,
        SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
    ):
        raise ValueError(f"unsupported sample_fmt {fmt}")
    if _has_shadow_extension(ver) and fmt not in _TIMED_SAMPLE_FORMATS:
        raise ValueError("shadow dump requires a timed variable-width sample format")
    header_nbytes = HEADER.size
    temperature_report = None
    if ver >= 5:
        if len(raw) < HEADER.size + TEMP_REPORT.size:
            raise ValueError("short temperature report extension")
        temp = TEMP_REPORT.unpack_from(raw, HEADER.size)
        temperature_report = dict(zip(TEMP_REPORT_KEYS, temp, strict=True))
        header_nbytes += TEMP_REPORT.size
    range_metadata_nbytes = (
        TIMED_FRAME_DESCRIPTOR.size * nf + (2 * nf)
        if fmt == SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED
        else TIMED_FRAME_DESCRIPTOR.size * nf
        if fmt == SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED
        else 2 * nf
        if fmt == SAMPLE_RANGE_FFT_IQ16_VARIABLE
        else nf
        if fmt == SAMPLE_RANGE_FFT_IQ16_WINDOWED
        else 0
    )
    shadow_metadata_nbytes = SHADOW_CANDIDATE.size * nf if _has_shadow_extension(ver) else 0
    return dict(
        version=ver,
        n_frames=nf,
        chirps_per_frame=cpf,
        n_tx=ntx,
        n_rx=nrx,
        n_samples=ns,
        trigger_frame=trig,
        frame_period_us=period_us,
        sample_fmt=fmt,
        range_bin_start=_pad if fmt == SAMPLE_RANGE_FFT_IQ16 else 0,
        range_metadata_nbytes=range_metadata_nbytes,
        shadow_metadata_nbytes=shadow_metadata_nbytes,
        frame_metadata_nbytes=range_metadata_nbytes + shadow_metadata_nbytes,
        header_nbytes=header_nbytes,
        temperature_report=temperature_report,
    )


def _parse_frame_metadata(raw: bytes, meta: dict) -> None:
    """Populate per-frame range-window metadata once its table has arrived."""
    metadata_nbytes = meta.get("frame_metadata_nbytes", 0)
    start = meta.get("header_nbytes", HEADER.size)
    range_stop = start + meta.get("range_metadata_nbytes", metadata_nbytes)
    stop = start + metadata_nbytes
    if len(raw) < stop:
        raise ValueError("short per-frame range-window metadata")
    if meta["sample_fmt"] == SAMPLE_RANGE_FFT_IQ16_WINDOWED:
        meta["range_bin_starts"] = tuple(raw[start:range_stop])
    elif meta["sample_fmt"] == SAMPLE_RANGE_FFT_IQ16_VARIABLE:
        table = raw[start:range_stop]
        meta["range_bin_starts"] = tuple(table[0::2])
        meta["range_bin_counts"] = tuple(table[1::2])
        if any(count <= 0 or count > meta["n_samples"] for count in meta["range_bin_counts"]):
            raise ValueError("invalid per-frame range-bin count")
    elif meta["sample_fmt"] in _TIMED_SAMPLE_FORMATS:
        descriptor_stop = start + TIMED_FRAME_DESCRIPTOR.size * meta["n_frames"]
        if len(raw) < descriptor_stop:
            raise ValueError("short per-frame range-window metadata")
        descriptors = tuple(TIMED_FRAME_DESCRIPTOR.iter_unpack(raw[start:descriptor_stop]))
        meta["range_bin_starts"] = tuple(item[0] for item in descriptors)
        meta["range_bin_counts"] = tuple(item[1] for item in descriptors)
        deltas_us = tuple(item[2] for item in descriptors)
        if any(count <= 0 or count > meta["n_samples"] for count in meta["range_bin_counts"]):
            raise ValueError("invalid per-frame range-bin count")
        if not deltas_us or deltas_us[0] != 0 or any(delta == 0 for delta in deltas_us[1:]):
            raise ValueError("invalid per-frame time delta")
        elapsed = 0
        offsets = []
        for delta in deltas_us:
            elapsed += delta
            offsets.append(elapsed)
        meta["frame_time_offsets_us"] = tuple(offsets)
        if meta["sample_fmt"] == SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED:
            scale_start = descriptor_stop
            scale_stop = range_stop
            if len(raw) < scale_stop:
                raise ValueError("short per-frame IQ8 scale metadata")
            scales = np.frombuffer(raw, dtype="<u2", offset=scale_start, count=meta["n_frames"])
            if scales.size < meta["n_frames"] or np.any(scales == 0):
                raise ValueError("invalid per-frame IQ8 scale")
            meta["iq8_scales"] = tuple(int(scale) for scale in scales)
    if meta.get("shadow_metadata_nbytes", 0):
        candidates = []
        for values in SHADOW_CANDIDATE.iter_unpack(raw[range_stop:stop]):
            candidates.append(
                {
                    "valid": bool(values[0]),
                    "range_bin": values[1],
                    "tx_index": values[2],
                    "peak_loop": values[3],
                    "power": values[4],
                    "rx_iq": tuple((values[5 + 2 * rx], values[6 + 2 * rx]) for rx in range(4)),
                }
            )
        if len(candidates) != meta["n_frames"]:
            raise ValueError("invalid shadow candidate metadata")
        meta["shadow_candidates"] = tuple(candidates)


def payload_nbytes(meta: dict, raw: bytes | None = None) -> int:
    """Bytes of int16-I/Q ADC payload following the header."""
    if meta["sample_fmt"] in _VARIABLE_SAMPLE_FORMATS:
        if "range_bin_counts" not in meta:
            if raw is None:
                raise ValueError("variable-width payload requires frame metadata")
            _parse_frame_metadata(raw, meta)
        sample_count = sum(meta["range_bin_counts"])
    else:
        sample_count = meta["n_frames"] * meta["n_samples"]
    bytes_per_complex = 2 if meta["sample_fmt"] == SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED else 4
    iq_nbytes = sample_count * meta["chirps_per_frame"] * meta["n_rx"] * bytes_per_complex
    return meta.get("frame_metadata_nbytes", 0) + iq_nbytes


def parse_dump(raw: bytes):
    """Dump bytes -> (meta dict, complex cube [n_frames, cpf, n_rx, n_samples])."""
    meta = parse_header(raw)
    nf, cpf, nrx, ns = (meta["n_frames"], meta["chirps_per_frame"], meta["n_rx"], meta["n_samples"])
    payload_offset = HEADER.size + meta.get("frame_metadata_nbytes", 0)
    _parse_frame_metadata(raw, meta)
    expected_nbytes = meta["header_nbytes"] + payload_nbytes(meta, raw)
    if len(raw) < expected_nbytes:
        raise ValueError(f"short payload: {len(raw)} bytes < {expected_nbytes} needed")
    if meta["sample_fmt"] == SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED:
        cube = np.zeros((nf, cpf, nrx, ns), dtype=np.complex128)
        byte_offset = payload_offset
        for frame, count in enumerate(meta["range_bin_counts"]):
            n = cpf * nrx * count
            body = np.frombuffer(raw, dtype=np.int8, offset=byte_offset, count=2 * n)
            if body.size < 2 * n:
                raise ValueError(f"short frame {frame} payload: {body.size} i8 < {2 * n} needed")
            iq = body.astype(np.float64) * meta["iq8_scales"][frame]
            cube[frame, ..., :count] = (iq[1::2] + 1j * iq[0::2]).reshape(cpf, nrx, count)
            byte_offset += 2 * n * np.dtype(np.int8).itemsize
    elif meta["sample_fmt"] in _VARIABLE_SAMPLE_FORMATS:
        cube = np.zeros((nf, cpf, nrx, ns), dtype=np.complex128)
        word_offset = payload_offset
        for frame, count in enumerate(meta["range_bin_counts"]):
            n = cpf * nrx * count
            body = np.frombuffer(raw, dtype="<i2", offset=word_offset, count=2 * n)
            if body.size < 2 * n:
                raise ValueError(f"short frame {frame} payload: {body.size} i16 < {2 * n} needed")
            iq = body.astype(np.float64)
            cube[frame, ..., :count] = (iq[1::2] + 1j * iq[0::2]).reshape(cpf, nrx, count)
            word_offset += 2 * n * np.dtype("<i2").itemsize
    else:
        n = nf * cpf * nrx * ns
        body = np.frombuffer(raw, dtype="<i2", offset=payload_offset, count=2 * n)
        if body.size < 2 * n:
            raise ValueError(f"short payload: {body.size} i16 < {2 * n} needed")
        iq = body.astype(np.float64)
        cube = (iq[1::2] + 1j * iq[0::2]).reshape(nf, cpf, nrx, ns)  # ImRe: Q,I pairs
    return meta, cube


def compute_shadow_candidate(
    frame: np.ndarray,
    *,
    n_tx: int,
    range_bin_start: int,
    gate_start: int,
    gate_count: int,
    min_power: int = 0,
) -> dict:
    """Reference implementation of the firmware's per-frame shadow selector.

    ``frame`` is one decoded ``[chirp, rx, local_bin]`` IQ16 snapshot. The
    selector integrates shifted IQ power across loops for each TX/bin, then
    retains the four native Im/Re samples from the strongest loop. This is a
    parity oracle for static-reflector testing, not the final moving-club model.
    """
    if frame.ndim != 3 or frame.shape[1] != 4:
        raise ValueError("shadow selector requires [chirp, 4 RX, bin] data")
    if n_tx <= 0 or frame.shape[0] % n_tx:
        raise ValueError("chirp count must contain complete TDM loops")
    if gate_count <= 0:
        raise ValueError("gate_count must be positive")

    n_loops = frame.shape[0] // n_tx
    n_bins = frame.shape[2]
    gate_end = gate_start + gate_count
    scaled_re = np.trunc(frame.real / 16.0).astype(np.int64)
    scaled_im = np.trunc(frame.imag / 16.0).astype(np.int64)
    sample_power = scaled_re * scaled_re + scaled_im * scaled_im

    best_power = 0
    best_tx = 0
    best_local_bin = 0
    for tx in range(n_tx):
        for local_bin in range(n_bins):
            absolute_bin = range_bin_start + local_bin
            if absolute_bin < gate_start or absolute_bin >= gate_end:
                continue
            chirps = np.arange(tx, frame.shape[0], n_tx)
            power = int(sample_power[chirps, :, local_bin].sum())
            if power > best_power:
                best_power = power
                best_tx = tx
                best_local_bin = local_bin

    valid = best_power > 0 and best_power >= min_power
    peak_loop = 0
    rx_iq = ((0, 0), (0, 0), (0, 0), (0, 0))
    if valid:
        loop_powers = []
        for loop in range(n_loops):
            chirp = loop * n_tx + best_tx
            loop_powers.append(int(sample_power[chirp, :, best_local_bin].sum()))
        peak_loop = int(np.argmax(loop_powers))
        peak = frame[peak_loop * n_tx + best_tx, :, best_local_bin]
        rx_iq = tuple((int(round(value.imag)), int(round(value.real))) for value in peak)

    return {
        "valid": valid,
        "range_bin": range_bin_start + best_local_bin,
        "tx_index": best_tx,
        "peak_loop": peak_loop,
        "power": best_power,
        "rx_iq": rx_iq,
    }


def select_tdm_loops(raw: bytes, *, start: int, count: int) -> bytes:
    """Return the same capture with a contiguous subset of complete TDM loops.

    This is an offline ablation helper: frame timing, range windows, TX order,
    and trigger placement remain unchanged while every frame keeps the same
    loop indices. It lets one 12-loop hardware capture answer whether a
    10-loop storage plan would have preserved the measurement.
    """
    meta, cube = parse_dump(raw)
    n_tx = meta["n_tx"]
    chirps_per_frame = meta["chirps_per_frame"]
    if n_tx <= 0 or chirps_per_frame % n_tx != 0:
        raise ValueError(f"chirps_per_frame {chirps_per_frame} is not divisible by n_tx {n_tx}")
    n_loops = chirps_per_frame // n_tx
    if start < 0 or count <= 0 or start + count > n_loops:
        raise ValueError(f"loop selection start={start} count={count} is outside 0-{n_loops - 1}")

    selected = cube.reshape(
        meta["n_frames"],
        n_loops,
        n_tx,
        meta["n_rx"],
        meta["n_samples"],
    )[:, start : start + count]
    selected = selected.reshape(
        meta["n_frames"],
        count * n_tx,
        meta["n_rx"],
        meta["n_samples"],
    )
    return pack_dump(
        selected,
        n_tx=n_tx,
        trigger_frame=meta["trigger_frame"],
        version=meta["version"],
        frame_period_us=meta["frame_period_us"],
        sample_fmt=meta["sample_fmt"],
        range_bin_start=meta.get("range_bin_start", 0),
        range_bin_starts=meta.get("range_bin_starts"),
        range_bin_counts=meta.get("range_bin_counts"),
        frame_time_offsets_us=meta.get("frame_time_offsets_us"),
    )


def is_range_snapshot(meta: dict) -> bool:
    """True when payload already contains selected range-FFT bins."""
    return meta.get("sample_fmt", SAMPLE_INT16_IQ) in (
        SAMPLE_RANGE_FFT_IQ16,
        SAMPLE_RANGE_FFT_IQ16_WINDOWED,
        SAMPLE_RANGE_FFT_IQ16_VARIABLE,
        SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
    )


def project_tx_pair(raw: bytes, tx_indices: tuple[int, int] = (0, 1)) -> bytes:
    """Return a dump containing only the selected TX chirps from each TDM loop.

    The current vertical LCMF estimator consumes a 2TX virtual array. Experimental
    capture builds may store an extra TX for horizontal/aim work; this helper
    keeps the full raw dump on disk while letting the existing vertical pipeline
    operate on a deterministic TX pair.
    """
    meta, cube = parse_dump(raw)
    n_tx = meta["n_tx"]
    if n_tx == len(tx_indices) and tuple(tx_indices) == tuple(range(n_tx)):
        return raw
    if any(tx < 0 or tx >= n_tx for tx in tx_indices):
        raise ValueError(f"TX projection {tx_indices} invalid for {n_tx} TX dump")
    if meta["chirps_per_frame"] % n_tx:
        raise ValueError(
            f"chirps_per_frame {meta['chirps_per_frame']} is not divisible by n_tx {n_tx}"
        )
    n_frames, _cpf, n_rx, n_samples = cube.shape
    loops = meta["chirps_per_frame"] // n_tx
    tdm = cube.reshape(n_frames, loops, n_tx, n_rx, n_samples)
    projected = tdm[:, :, list(tx_indices), :, :].reshape(
        n_frames,
        loops * len(tx_indices),
        n_rx,
        n_samples,
    )
    return pack_dump(
        projected,
        n_tx=len(tx_indices),
        trigger_frame=meta["trigger_frame"],
        version=meta["version"],
        frame_period_us=meta.get("frame_period_us", 0),
        sample_fmt=meta.get("sample_fmt", SAMPLE_INT16_IQ),
        range_bin_start=meta.get("range_bin_start", 0),
        range_bin_starts=meta.get("range_bin_starts"),
        range_bin_counts=meta.get("range_bin_counts"),
        frame_time_offsets_us=meta.get("frame_time_offsets_us"),
    )


def range_fft(cube: np.ndarray, n_fft: int | None = None) -> np.ndarray:
    """FFT over the ADC-sample axis -> [n_frames, cpf, n_rx, n_range]."""
    n_fft = n_fft or cube.shape[-1]
    return np.fft.fft(cube, n=n_fft, axis=-1)


def range_data(meta: dict, cube: np.ndarray, n_fft: int | None = None) -> np.ndarray:
    """Return range-domain data for either raw ADC or range-snapshot dumps."""
    if is_range_snapshot(meta):
        return cube
    return range_fft(cube, n_fft=n_fft)


def ball_range_bin(
    rfft: np.ndarray, frame: int, *, lo_bin: int = 1, hi_bin: int | None = None
) -> int:
    """Strongest range bin in a gate (power summed over all chirps + RX)."""
    hi_bin = hi_bin or rfft.shape[-1] // 2
    power = np.sum(np.abs(rfft[frame, :, :, lo_bin:hi_bin]) ** 2, axis=(0, 1))
    return lo_bin + int(np.argmax(power))


def virtual_snapshot(
    rfft: np.ndarray, frame: int, range_bin: int, n_tx: int, n_rx: int, loop: int = 0
) -> np.ndarray:
    """n_tx*n_rx-element complex snapshot at (frame, range_bin) for one loop.

    Elements ordered [tx0.rx0.., tx1.rx0.., ...] = the rotated elevation ULA.
    """
    snap = np.empty(n_tx * n_rx, dtype=complex)
    for tx in range(n_tx):
        chirp = loop * n_tx + tx
        for rx in range(n_rx):
            snap[tx * n_rx + rx] = rfft[frame, chirp, rx, range_bin]
    return snap


def frame_elevation(
    rfft: np.ndarray,
    frame: int,
    *,
    n_tx: int,
    n_rx: int,
    noise_var: float = 1.0,
    lo_bin: int = 1,
    hi_bin: int | None = None,
):
    """Per-frame elevation (deg) at the frame's strongest range bin.

    Returns (elev_deg, range_bin). noise_var scales the MUSIC source-count
    threshold; pass an estimate of the per-element noise power.
    """
    rb = ball_range_bin(rfft, frame, lo_bin=lo_bin, hi_bin=hi_bin)
    snap = virtual_snapshot(rfft, frame, rb, n_tx, n_rx)
    theta = est_music_fbss(snap, noise_var)
    return float(np.degrees(theta)), rb


# --- synthesis (tests + executable spec of the firmware format) -------------
def synth_target(
    elev_deg: float,
    range_bin: int,
    *,
    n_frames: int = 1,
    n_tx: int = 2,
    n_rx: int = 4,
    n_samples: int = 128,
    amp: float = 1000.0,
    noise: float = 0.0,
    rng=None,
) -> np.ndarray:
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
        cube += (rng.standard_normal(cube.shape) + 1j * rng.standard_normal(cube.shape)) * noise
    return cube
