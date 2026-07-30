"""Tests for the IWR6843 CLI and dump serial contract."""

from __future__ import annotations

import numpy as np
import pytest

from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dump import (
    SAMPLE_RANGE_FFT_IQ16_VARIABLE,
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    pack_dump,
)


def test_send_config_rejects_missing_cli_acknowledgement(tmp_path, monkeypatch):
    """A wedged board must not be reported as configured and armed."""
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: "")

    with pytest.raises(RuntimeError, match="did not acknowledge"):
        radar.send_config(str(config))


def test_read_dump_sizes_variable_width_payload_from_frame_metadata():
    cube = np.ones((2, 6, 4, 7), dtype=complex)
    raw = pack_dump(
        cube,
        n_tx=3,
        version=5,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16_VARIABLE,
        range_bin_starts=(20, 32),
        range_bin_counts=(4, 7),
    )

    class FakeSerial:
        def __init__(self, payload):
            self.payload = bytearray(b"l3dump\r\n" + payload)
            self.written = b""

        @property
        def in_waiting(self):
            return len(self.payload)

        def reset_input_buffer(self):
            return None

        def write(self, value):
            self.written += value

        def read(self, count):
            chunk = bytes(self.payload[:count])
            del self.payload[:count]
            return chunk

    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(raw)

    assert radar.read_dump(timeout_s=0.1) == raw
    assert radar.ser.written == b"l3dump\n"


def test_read_dump_sizes_timed_variable_payload_from_frame_metadata():
    cube = np.ones((3, 36, 4, 7), dtype=complex)
    raw = pack_dump(
        cube,
        n_tx=3,
        version=6,
        frame_period_us=2000,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        range_bin_starts=(20, 32, 47),
        range_bin_counts=(4, 7, 7),
        frame_time_offsets_us=(0, 2000, 6000),
    )

    class FakeSerial:
        def __init__(self, payload):
            self.payload = bytearray(b"l3dump\r\n" + payload)
            self.written = b""

        @property
        def in_waiting(self):
            return len(self.payload)

        def reset_input_buffer(self):
            return None

        def write(self, value):
            self.written += value

        def read(self, count):
            chunk = bytes(self.payload[:count])
            del self.payload[:count]
            return chunk

    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(raw)

    assert radar.read_dump(timeout_s=0.1) == raw
    assert radar.ser.written == b"l3dump\n"
