"""End-to-end test of the hardware-day CLI tools on a synthetic session.

INTENT (for future sessions / Opus 4.8): proves that stage1_analyze.py runs
correctly on a session directory laid out exactly as stage1_capture.py
creates it (raw_uart.bin + session_meta.json), using synthetic frames built
with the same physics as stage1_dryrun. If this passes, hardware day only
tests the serial layer and the A1-A3 byte conventions — everything from
file-on-disk to Gate-S1-A verdict is already known-good.

Requires the analysis extra (matplotlib via music_stage0_sim):
    uv run --extra analysis pytest tests/test_stage1_tools.py -v
"""

import json

import numpy as np
import pytest

pytest.importorskip("matplotlib")  # skip cleanly if analysis extra absent

import iwr6843_uart as uart
import stage1_analyze
from stage1_dryrun import RANGE_RES, synthesize_frame


def make_session(tmp_path, elev_deg=8.0, range_m=3.0, n_frames=6):
    rng = np.random.default_rng(11)
    raw = b"".join(
        uart.build_frame(
            {uart.TLV_AZIMUTH_STATIC_HEATMAP:
             uart.build_azimuth_heatmap_payload(
                 synthesize_frame(range_m, elev_deg, rng=rng)[0])},
            frame_number=i)
        for i in range(n_frames))
    (tmp_path / "raw_uart.bin").write_bytes(raw)

    # geometry that reproduces elev_deg exactly (mirrors synthesize_frame)
    h_radar = 0.25
    h_target = h_radar + range_m * np.sin(np.radians(elev_deg))
    meta = {
        "test": "static", "range_res_m": RANGE_RES,
        "radar_height_m": h_radar, "board_tilt_deg": 0.0,
        "reflector_positions": [{"range_m": range_m, "height_m": h_target}],
    }
    (tmp_path / "session_meta.json").write_text(json.dumps(meta))
    return tmp_path


class TestAnalyzeOnSyntheticSession:
    def test_gate_s1a_passes_on_clean_synthetic(self, tmp_path, capsys):
        sess = make_session(tmp_path, elev_deg=8.0)
        rc = stage1_analyze.main(["--session", str(sess)])
        out = capsys.readouterr().out
        assert rc == 0
        assert "Gate S1-A: PASS" in out
        assert "frames: 6" in out

    def test_reports_bin_and_range(self, tmp_path, capsys):
        sess = make_session(tmp_path, elev_deg=-5.0, range_m=2.5)
        stage1_analyze.main(["--session", str(sess)])
        out = capsys.readouterr().out
        expected_bin = int(round(2.5 / RANGE_RES))
        assert f" {expected_bin} " in out

    def test_placeholder_meta_prompts_edit(self, tmp_path, capsys):
        sess = make_session(tmp_path)
        meta = json.loads((sess / "session_meta.json").read_text())
        meta["radar_height_m"] = "EDIT_ME"
        (sess / "session_meta.json").write_text(json.dumps(meta))
        rc = stage1_analyze.main(["--session", str(sess)])
        assert rc == 0
        assert "ground truth unavailable" in capsys.readouterr().out

    def test_ball_cfg_capture_gives_clear_message(self, tmp_path, capsys):
        # point-cloud-only capture (ball cfg) has no heatmap TLV
        pts = np.array([[0.1, 3.0, 0.2, 9.5]], dtype=np.float32)
        raw = uart.build_frame({uart.TLV_DETECTED_POINTS: pts.tobytes()})
        (tmp_path / "raw_uart.bin").write_bytes(raw)
        (tmp_path / "session_meta.json").write_text("{}")
        rc = stage1_analyze.main(["--session", str(tmp_path)])
        assert rc == 1
        assert "ball cfg disables the heatmap" in capsys.readouterr().out


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
