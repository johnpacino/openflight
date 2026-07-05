"""Unit tests for the IWR6843 UART parser (Stage-1 prep, pre-hardware).

INTENT (for future sessions / Opus 4.8): these tests prove the parser is
self-consistent with the byte conventions documented in iwr6843_uart.py
(A1/A2/A3 assumptions). They CANNOT prove the conventions match the real
board — that's the first-real-frame check in docs/plans/stage1_levm_bringup.md.
If hardware reveals a convention difference, fix iwr6843_uart.py, and these
tests must still pass (they exercise the round-trip, not absolute layout).
"""

import numpy as np
import pytest

import iwr6843_uart as uart


def _synthetic_heatmap(n_bins=32, n_ant=8, seed=3):
    rng = np.random.default_rng(seed)
    hm = rng.normal(0, 100, (n_bins, n_ant)) + 1j * rng.normal(0, 100, (n_bins, n_ant))
    return np.round(hm).astype(np.complex64)


class TestFrameRoundTrip:
    def test_single_frame_round_trip(self):
        hm = _synthetic_heatmap()
        payload = uart.build_azimuth_heatmap_payload(hm)
        raw = uart.build_frame({uart.TLV_AZIMUTH_STATIC_HEATMAP: payload}, frame_number=7)

        frames = list(uart.iter_frames(raw))
        assert len(frames) == 1
        f = frames[0]
        assert f.ok
        assert f.header.frame_number == 7
        assert f.header.num_tlvs == 1
        recovered = uart.parse_azimuth_heatmap(f.tlvs[uart.TLV_AZIMUTH_STATIC_HEATMAP])
        np.testing.assert_allclose(recovered, hm, atol=0.51)

    def test_multiple_frames_and_garbage_prefix(self):
        hm = _synthetic_heatmap()
        payload = uart.build_azimuth_heatmap_payload(hm)
        raw = (b"\x00\xff garbage \x13" +
               uart.build_frame({4: payload}, frame_number=1) +
               b"\x99\x99" +  # mid-stream junk (e.g. reconnect glitch)
               uart.build_frame({4: payload}, frame_number=2))
        frames = list(uart.iter_frames(raw))
        assert [f.header.frame_number for f in frames] == [1, 2]

    def test_incomplete_tail_is_not_yielded(self):
        payload = uart.build_azimuth_heatmap_payload(_synthetic_heatmap())
        raw = uart.build_frame({4: payload})
        frames = list(uart.iter_frames(raw[:-10]))  # truncated packet
        assert frames == []

    def test_multi_tlv_frame(self):
        hm = _synthetic_heatmap(n_bins=16)
        points = np.array([[0.1, 3.0, 0.4, 31.5]], dtype=np.float32)
        rp = np.arange(16, dtype=np.uint16)
        raw = uart.build_frame({
            uart.TLV_DETECTED_POINTS: points.tobytes(),
            uart.TLV_RANGE_PROFILE: rp.tobytes(),
            uart.TLV_AZIMUTH_STATIC_HEATMAP: uart.build_azimuth_heatmap_payload(hm),
        })
        f = next(uart.iter_frames(raw))
        assert f.ok and f.header.num_tlvs == 3
        np.testing.assert_array_equal(
            uart.parse_detected_points(f.tlvs[1]), points)
        np.testing.assert_array_equal(uart.parse_range_profile(f.tlvs[2]), rp)
        assert uart.parse_azimuth_heatmap(f.tlvs[4]).shape == (16, 8)


class TestPhaseIntegrity:
    """The property that actually matters: cross-antenna PHASE survives the
    int16 round trip, because phase IS the launch angle."""

    def test_phase_ramp_survives_round_trip(self):
        theta = np.radians(12.0)
        amp = 2000.0  # comfortably inside int16
        snapshot = amp * np.exp(1j * np.pi * np.sin(theta) * np.arange(8))
        hm = np.tile(snapshot, (4, 1)).astype(np.complex64)
        payload = uart.build_azimuth_heatmap_payload(hm)
        rec = uart.parse_azimuth_heatmap(payload)
        dphi = np.angle(rec[0, 1:] * np.conj(rec[0, :-1]))
        np.testing.assert_allclose(dphi, np.pi * np.sin(theta), atol=1e-3)

    def test_bad_tlv_length_flags_not_crashes(self):
        payload = uart.build_azimuth_heatmap_payload(_synthetic_heatmap())
        raw = bytearray(uart.build_frame({4: payload}))
        # corrupt the TLV length field to overrun the packet (A1 violation)
        import struct
        struct.pack_into("<I", raw, uart.FRAME_HEADER_LEN + 4, len(payload) + 64)
        f = next(uart.iter_frames(bytes(raw)))
        assert not f.ok


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
