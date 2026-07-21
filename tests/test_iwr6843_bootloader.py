"""Tests for the IWR6843 ROM UART bootloader protocol."""

from __future__ import annotations

import struct

import pytest

import firmware.flash_iwr6843 as flash_cli
from openflight.iwr6843.bootloader import (
    ACK,
    FILE_TYPE_META_IMAGE1,
    OP_CLOSE,
    OP_ERASE,
    OP_GET_STATUS,
    OP_OPEN,
    OP_PING,
    OP_WRITE_FLASH,
    STATUS_SUCCESS,
    STORAGE_SFLASH,
    BootloaderError,
    IWR6843Bootloader,
    encode_command,
    read_response,
)


def _response(payload: bytes) -> bytes:
    length = len(payload) + 2
    return struct.pack(">HB", length, sum(payload) & 0xFF) + payload


class FakeSerial:
    """Small pyserial-compatible double with scripted bootloader replies."""

    def __init__(self, responses: list[bytes]):
        self._responses = bytearray().join(responses)
        self.writes: list[bytes] = []
        self.break_durations: list[float] = []
        self.break_states: list[bool] = []
        self._break_condition = False

    def read(self, size: int) -> bytes:
        data = bytes(self._responses[:size])
        del self._responses[:size]
        return data

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def send_break(self, duration: float = 0.25) -> None:
        self.break_durations.append(duration)

    @property
    def break_condition(self) -> bool:
        return self._break_condition

    @break_condition.setter
    def break_condition(self, active: bool) -> None:
        self._break_condition = active
        self.break_states.append(active)

    def reset_input_buffer(self) -> None:
        pass


def test_ping_packet_matches_ti_protocol_example():
    assert encode_command(OP_PING) == bytes.fromhex("aa00032020")


def test_open_packet_matches_ti_uniflash_trace():
    data = struct.pack(">IIII", 0x0004E4C4, STORAGE_SFLASH, FILE_TYPE_META_IMAGE1, 0)

    packet = encode_command(OP_OPEN, data)

    assert packet == bytes.fromhex("aa0013d3210004e4c4000000020000000400000000")


def test_read_response_validates_checksum():
    serial = FakeSerial([bytes.fromhex("0004cc00cc")])
    assert read_response(serial) == ACK

    corrupt = FakeSerial([bytes.fromhex("0004cd00cc")])
    with pytest.raises(BootloaderError, match="checksum"):
        read_response(corrupt)


def test_read_response_rejects_nack():
    serial = FakeSerial([bytes.fromhex("0004330033")])

    with pytest.raises(BootloaderError, match="NACK"):
        read_response(serial, require_ack=True)


def test_handshake_skips_zero_bytes_before_bootloader_ack():
    serial = FakeSerial([b"\x00\x00\x00" + _response(ACK), _response(ACK)])

    IWR6843Bootloader(serial).handshake()

    assert serial.break_states == [True, False]


def test_response_wait_survives_one_serial_timeout():
    """Flash erase can take longer than the serial port's read timeout."""

    class DelayedSerial:
        def __init__(self):
            self.reads = [b"", bytes.fromhex("0004"), bytes.fromhex("cc"), bytes.fromhex("00cc")]

        def read(self, _size):
            return self.reads.pop(0)

    assert read_response(DelayedSerial(), require_ack=True) == ACK


def test_flash_uses_break_and_240_byte_chunks():
    image = b"MSTR" + bytes(range(256)) * 2
    chunk_count = (len(image) + 239) // 240
    replies = [
        _response(ACK),  # UART break
        _response(ACK),  # ping
        _response(ACK),  # erase
        _response(ACK),  # open
        *[_response(ACK) for _ in range(chunk_count)],
        _response(ACK),  # close
        _response(bytes([STATUS_SUCCESS])),
    ]
    serial = FakeSerial(replies)
    bootloader = IWR6843Bootloader(serial)
    progress: list[tuple[int, int]] = []

    bootloader.flash(image, progress=lambda sent, total: progress.append((sent, total)))

    assert serial.break_durations == []
    assert serial.break_states == [True, False]
    opcodes = [packet[4] for packet in serial.writes]
    assert opcodes == [
        OP_PING,
        OP_ERASE,
        OP_OPEN,
        *([OP_WRITE_FLASH] * chunk_count),
        OP_CLOSE,
        OP_GET_STATUS,
    ]
    write_packets = [packet for packet in serial.writes if packet[4] == OP_WRITE_FLASH]
    assert all(len(packet) <= 245 for packet in write_packets)
    assert serial.writes[1] == bytes.fromhex("aa000f2a28000000020000000000000000")
    assert progress[-1] == (len(image), len(image))


def test_flash_rejects_non_meta_image_before_serial_activity():
    serial = FakeSerial([])
    bootloader = IWR6843Bootloader(serial)

    with pytest.raises(ValueError, match="MSTR"):
        bootloader.flash(b"not a TI meta image")

    assert serial.writes == []
    assert serial.break_durations == []


def test_flash_cli_opens_uart_before_asking_for_reset(tmp_path, monkeypatch):
    """Opening CP2105 after reset can glitch the LEVM control lines."""
    image = tmp_path / "firmware.bin"
    image.write_bytes(b"MSTR-test-image")
    events: list[str] = []

    class OpenedSerial(FakeSerial):
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            events.append("close")

    def open_serial(_port):
        events.append("open")
        return OpenedSerial([])

    def confirm(_action):
        events.append("confirm-reset")
        return False

    monkeypatch.setattr(flash_cli, "open_serial", open_serial)
    monkeypatch.setattr(flash_cli, "confirm_prepared", lambda: True)
    monkeypatch.setattr(flash_cli, "confirm_bootloader", confirm)
    monkeypatch.setattr(
        "sys.argv",
        ["flash_iwr6843.py", str(image), "--port", "/dev/ttyUSB0"],
    )

    assert flash_cli.main() == 2
    assert events == ["open", "confirm-reset", "close"]


def test_flash_cli_waits_for_reset_before_sending_break(tmp_path, monkeypatch):
    """The LEVM workflow resets before the UniFlash load operation."""
    image = tmp_path / "firmware.bin"
    image.write_bytes(b"MSTR-test-image")
    events: list[str] = []

    class OpenedSerial(FakeSerial):
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            events.append("close")

    class FakeBootloader:
        def __init__(self, _serial):
            events.append("bootloader")

        def handshake(self):
            events.append("break-pulse")

        def flash(self, _image, **_kwargs):
            events.append("flash")

    def open_serial(_port):
        events.append("open")
        return OpenedSerial([])

    def confirm(_action):
        events.append("confirm-reset")
        return True

    monkeypatch.setattr(flash_cli, "open_serial", open_serial)
    monkeypatch.setattr(flash_cli, "IWR6843Bootloader", FakeBootloader)
    monkeypatch.setattr(flash_cli, "confirm_prepared", lambda: True)
    monkeypatch.setattr(flash_cli, "confirm_bootloader", confirm)
    monkeypatch.setattr(
        "sys.argv",
        ["flash_iwr6843.py", str(image), "--port", "/dev/ttyUSB0"],
    )

    assert flash_cli.main() == 0
    assert events == [
        "open",
        "bootloader",
        "confirm-reset",
        "break-pulse",
        "flash",
        "close",
    ]
