"""Tests for the Stage-1c L3 raw-ADC dump pipeline (hardware-free).

Validates the parse -> range-FFT -> virtual-snapshot -> FBSS-MUSIC integration
against synthetic point targets. The HW-specific bits (TX/RX ordering, phase
cal, TDM Doppler correction) are flagged VERIFY-ON-HW in iwr6843_l3dump.py; what
these lock down is that the format round-trips and the DSP chain recovers a
known injected elevation + range.
"""
import numpy as np
import pytest

import iwr6843_l3dump as l3


class TestDumpFormat:
    def test_pack_parse_roundtrip(self):
        cube = l3.synth_target(10.0, 20, n_frames=2, n_tx=2, n_rx=4)
        raw = l3.pack_dump(cube, n_tx=2, trigger_frame=1)
        meta, cube2 = l3.parse_dump(raw)
        assert meta["n_frames"] == 2
        assert meta["chirps_per_frame"] == 2
        assert meta["n_tx"] == 2 and meta["n_rx"] == 4
        assert meta["n_samples"] == 128
        assert meta["trigger_frame"] == 1
        # int16 quantization only
        assert np.allclose(cube2, cube, atol=1.0)

    def test_bad_magic_rejected(self):
        with pytest.raises(ValueError):
            l3.parse_dump(b"XXXX" + b"\x00" * 40)

    def test_short_payload_rejected(self):
        cube = l3.synth_target(0.0, 10, n_frames=1)
        raw = l3.pack_dump(cube, n_tx=2)
        with pytest.raises(ValueError):
            l3.parse_dump(raw[: l3.HEADER.size + 8])  # header + a few samples


class TestPipeline:
    def test_range_fft_peaks_at_injected_bin(self):
        cube = l3.synth_target(0.0, 40, n_frames=1)
        rfft = l3.range_fft(cube)
        assert l3.ball_range_bin(rfft, 0) == 40

    @pytest.mark.parametrize("elev", [-20.0, -8.0, 0.0, 12.0, 25.0])
    def test_elevation_recovered_clean(self, elev):
        cube = l3.synth_target(elev, 30, n_frames=1)
        rfft = l3.range_fft(cube)
        deg, rb = l3.frame_elevation(rfft, 0, n_tx=2, n_rx=4, noise_var=1.0)
        assert rb == 30
        assert abs(deg - elev) < 1.0

    def test_elevation_recovered_with_noise(self):
        # ~20 dB SNR: amp 1000 vs noise 100 -> MUSIC should still land the angle
        cube = l3.synth_target(15.0, 25, n_frames=1, amp=1000.0, noise=100.0,
                               rng=np.random.default_rng(3))
        rfft = l3.range_fft(cube)
        deg, rb = l3.frame_elevation(rfft, 0, n_tx=2, n_rx=4, noise_var=1e4)
        assert rb == 25
        assert abs(deg - 15.0) < 2.0

    def test_roundtrip_then_pipeline(self):
        # full path a firmware dump would take: bytes -> parse -> angle
        cube = l3.synth_target(-12.0, 33, n_frames=1)
        meta, parsed = l3.parse_dump(l3.pack_dump(cube, n_tx=2))
        rfft = l3.range_fft(parsed)
        deg, rb = l3.frame_elevation(rfft, 0, n_tx=meta["n_tx"],
                                     n_rx=meta["n_rx"], noise_var=1.0)
        assert rb == 33
        assert abs(deg - (-12.0)) < 1.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
