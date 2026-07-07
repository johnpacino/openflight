"""Tests for the scripts/analysis/session_shot_report.py reporting tool.

The two_ray replay itself is covered by test_kld7_two_ray.py; here we test
the report's own logic: session parsing, club/tuning extraction, the
no-data replay short-circuits, and HTML rendering.
"""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

from openflight.launch_monitor import ClubType

_SCRIPT = Path(__file__).parent.parent / "scripts" / "analysis" / "session_shot_report.py"
_spec = importlib.util.spec_from_file_location("session_shot_report", _SCRIPT)
ssr = importlib.util.module_from_spec(_spec)
# Register before exec so the module's dataclasses can resolve their
# (string, via `from __future__ import annotations`) annotations.
sys.modules[_spec.name] = ssr
_spec.loader.exec_module(ssr)


class TestClubType:
    def test_known_clubs(self):
        assert ssr.club_type("7-iron") is ClubType.IRON_7
        assert ssr.club_type("driver") is ClubType.DRIVER
        assert ssr.club_type("pw") is ClubType.PW

    def test_unknown_or_missing(self):
        assert ssr.club_type("sand-wedge") is ClubType.UNKNOWN
        assert ssr.club_type("") is ClubType.UNKNOWN
        assert ssr.club_type(None) is ClubType.UNKNOWN


class TestTuningFromSession:
    def test_reads_logged_params_and_fills_defaults(self):
        start = {
            "config": {
                "kld7_experiments": {
                    "radc_tuning_params": {
                        "radc_speed_tolerance_mph": 8.0,
                        "radc_spectrum_source": "sum12",
                        # ops_anchored omitted -> should fall back to default
                    }
                }
            }
        }
        tuning = ssr.tuning_from_session(start)
        assert tuning["speed_tolerance_mph"] == 8.0
        assert tuning["spectrum_source"] == "sum12"
        assert (
            tuning["ops_anchored_peak_min_snr"] == ssr._TUNING_DEFAULTS["ops_anchored_peak_min_snr"]
        )

    def test_empty_session_start_is_all_defaults(self):
        assert ssr.tuning_from_session({}) == ssr._TUNING_DEFAULTS

    def test_returns_only_extract_launch_angle_kwargs(self):
        # No raw "radc_*" keys leak through — every key is a valid kwarg name.
        tuning = ssr.tuning_from_session({})
        assert set(tuning) == set(ssr._TUNING_DEFAULTS)


def _write_session(tmp_path: Path) -> Path:
    """A minimal but well-formed session: one 7-iron shot, no RADC buffer."""
    rows = [
        {
            "type": "session_start",
            "config": {
                "kld7_experiments": {"radc_tuning_params": {"radc_centroid_floor_frac": 0.6}}
            },
        },
        {
            "type": "shot_detected",
            "shot_number": 1,
            "club": "7-iron",
            "ball_speed_mph": 110.0,
            "club_speed_mph": 86.0,
            "smash_factor": 1.28,
            "launch_angle_vertical": 14.2,
            "launch_angle_vertical_confidence": 0.65,
            "launch_angle_vertical_source": "radar",
        },
        {
            "type": "rolling_buffer_capture",
            "shot_number": 1,
            "ball_speed_mph": 110.5,
        },
        {
            "type": "kld7_buffer",
            "orientation": "vertical",
            "shot_number": 1,
            "ball_angle": {
                "vertical_deg": 14.0,
                "confidence": 0.65,
                "radc_selection": {"angle_offset_deg": 1.5, "selected_t_ms": [25.0, 54.0]},
            },
            "frames": [],  # no radc_b64 -> offline replay short-circuits
        },
    ]
    path = tmp_path / "session_20260620_000000_test.jsonl"
    path.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    return path


class TestLoadSession:
    def test_parses_entries_and_detects_offset(self, tmp_path):
        sess = ssr.load_session(_write_session(tmp_path))
        assert set(sess.shots) == {1}
        assert sess.ball_speeds[1] == 110.5
        assert 1 in sess.vbufs
        assert sess.logged_offset_deg == 1.5
        assert sess.tuning["centroid_floor_frac"] == 0.6

    def test_tolerates_blank_and_malformed_lines(self, tmp_path):
        path = tmp_path / "s.jsonl"
        path.write_text(
            '\n{"type":"session_start"}\nnot json\n{"type":"shot_detected","shot_number":3}\n'
        )
        sess = ssr.load_session(path)
        assert set(sess.shots) == {3}


class TestReplayShortCircuits:
    def _geom(self):
        return ssr.Geometry(10.3, 1.5, 5.0, -4.0 / 12.0, 10.0)

    def test_no_buffer_refuses(self):
        out = ssr.replay_shot({"club": "7-iron"}, None, 110.0, self._geom(), ssr._TUNING_DEFAULTS)
        assert out["refusal"] == "no_buffer"
        assert out["tier"] is None

    def test_no_ball_speed_refuses(self):
        out = ssr.replay_shot(
            {"club": "7-iron"}, {"frames": []}, None, self._geom(), ssr._TUNING_DEFAULTS
        )
        assert out["refusal"] == "no_ball_speed"

    def test_empty_frames_refuses_without_crashing(self):
        # vbuf present but no usable RADC -> two_ray refuses, no exception
        out = ssr.replay_shot(
            {"club": "7-iron"}, {"frames": []}, 110.0, self._geom(), ssr._TUNING_DEFAULTS
        )
        assert out["tier"] is None
        assert out["refusal"] is not None


class TestRenderHtml:
    def test_end_to_end_render(self, tmp_path):
        sess = ssr.load_session(_write_session(tmp_path))
        geom = ssr.Geometry(10.3, 1.5, 5.0, -4.0 / 12.0, 10.0, angle_offset_auto=True)
        rows = ssr.collect_rows(sess, geom, sess.tuning)
        assert len(rows) == 1
        assert rows[0]["disp"] == 14.2 and rows[0]["source"] == "radar"
        out = ssr.render_html(rows, "session_test", geom)
        assert out.startswith("<!DOCTYPE html>")
        assert "session_test" in out
        assert "7-iron" in out  # club summary
        assert "1× 7-iron" in out
        assert "mount 10.3" in out and "offset 1.5" in out
        # the lone shot has no buffer frames -> offline refuses -> no tier pill
        assert out.count("<tr>") == 1

    def test_dealias_column_hidden_when_all_off(self, tmp_path):
        sess = ssr.load_session(_write_session(tmp_path))
        geom = ssr.Geometry(10.3, 1.5, 5.0, -4.0 / 12.0, 10.0)
        out = ssr.render_html(ssr.collect_rows(sess, geom, sess.tuning), "s", geom)
        assert "<th>de-alias</th>" not in out  # column hidden (footnote may still mention it)


class TestMain:
    def test_main_writes_report(self, tmp_path):
        session = _write_session(tmp_path)
        out = tmp_path / "report.html"
        rc = ssr.main([str(session), "-o", str(out)])
        assert rc == 0
        assert out.exists()
        assert out.read_text().startswith("<!DOCTYPE html>")

    def test_main_errors_on_missing_session(self, tmp_path):
        with pytest.raises(SystemExit):
            ssr.main([str(tmp_path / "nope.jsonl")])
