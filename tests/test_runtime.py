"""Tests for the Stage-1c Pi runtime core (hardware-free).

Covers the two testable pieces: read_dump() framing a burst by the header's
byte count, and process_dump() turning a dump into per-frame angles. The GPIO +
serial glue in run_loop() is Pi-only and smoke-tested on hardware.
"""
import numpy as np
import pytest

import iwr6843_l3dump as l3
import iwr6843_runtime as rt


def _chunked_reader(data: bytes, chunk: int = 7):
    """Fake serial.read: hands out `data` in <=chunk-byte pieces, then b''."""
    pos = [0]

    def read_fn(n):
        take = min(n, chunk, len(data) - pos[0])
        s = data[pos[0]:pos[0] + take]
        pos[0] += take
        return s

    return read_fn


class TestReadDump:
    def test_assembles_exact_burst(self):
        raw = l3.pack_dump(l3.synth_target(5.0, 22, n_frames=3), n_tx=2)
        got = rt.read_dump(_chunked_reader(raw, chunk=13), timeout_s=2.0)
        assert got == raw

    def test_handles_whole_burst_in_one_read(self):
        raw = l3.pack_dump(l3.synth_target(0.0, 10, n_frames=1), n_tx=2)
        got = rt.read_dump(_chunked_reader(raw, chunk=len(raw)), timeout_s=2.0)
        assert got == raw

    def test_timeout_when_silent(self):
        with pytest.raises(TimeoutError):
            rt.read_dump(lambda n: b"", timeout_s=0.05)

    def test_timeout_on_truncated_payload(self):
        raw = l3.pack_dump(l3.synth_target(0.0, 10, n_frames=2), n_tx=2)
        truncated = raw[: l3.HEADER.size + 100]   # header ok, payload short
        with pytest.raises(TimeoutError):
            rt.read_dump(_chunked_reader(truncated), timeout_s=0.05)


class TestProcessDump:
    @pytest.mark.parametrize("elev", [-9.0, 0.0, 18.0])
    def test_recovers_angle_every_frame(self, elev):
        raw = l3.pack_dump(l3.synth_target(elev, 28, n_frames=4), n_tx=2)
        meta, rows = rt.process_dump(raw, noise_var=1.0)
        assert meta["n_frames"] == 4 and len(rows) == 4
        for r in rows:
            assert r["range_bin"] == 28
            assert abs(r["elev_deg"] - elev) < 1.0

    def test_end_to_end_read_then_process(self):
        raw = l3.pack_dump(l3.synth_target(12.0, 35, n_frames=2), n_tx=2)
        got = rt.read_dump(_chunked_reader(raw, chunk=9), timeout_s=2.0)
        meta, rows = rt.process_dump(got)
        assert [r["range_bin"] for r in rows] == [35, 35]
        assert all(abs(r["elev_deg"] - 12.0) < 1.0 for r in rows)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
