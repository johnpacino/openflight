import json
from types import SimpleNamespace

import pytest

from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.replay import (
    ReplayInput,
    inputs_from_session,
    replay_capture,
    replay_summary,
    write_records,
)


def test_inputs_from_session_extracts_debug_dump_references(tmp_path):
    dump = tmp_path / "shot.l3dump"
    dump.write_bytes(b"raw")
    session = tmp_path / "session.jsonl"
    session.write_text(
        "\n".join(
            [
                json.dumps({"type": "shot_detected", "shot_number": 1}),
                json.dumps(
                    {
                        "type": "iwr6843_capture",
                        "shot_number": 1,
                        "capture_path": "shot.l3dump",
                        "ball_speed_mph": 101.5,
                    }
                ),
                json.dumps(
                    {
                        "type": "iwr6843_capture",
                        "shot_number": 2,
                        "capture_path": None,
                        "ball_speed_mph": 102.5,
                    }
                ),
            ]
        ),
        encoding="utf-8",
    )

    inputs = inputs_from_session(session, club="9i")

    assert inputs == [
        ReplayInput(
            source=str(session),
            shot_number=1,
            capture_path=dump,
            ball_speed_mph=101.5,
            club="9i",
        )
    ]


def test_replay_capture_reports_missing_file(tmp_path):
    cal = Calibration.identity()
    cal.tee_range_m = 1.5

    record = replay_capture(
        ReplayInput(
            source="single",
            shot_number=None,
            capture_path=tmp_path / "missing.l3dump",
            ball_speed_mph=100.0,
        ),
        cal,
        net_range_m=4.6,
        tx_order="normal",
    )

    assert record.status == "missing_capture_file"
    assert not record.accepted
    assert "missing.l3dump" in record.error


def test_replay_capture_uses_estimator_result(monkeypatch, tmp_path):
    dump = tmp_path / "shot.l3dump"
    dump.write_bytes(b"raw")
    cal = Calibration.identity()
    cal.tee_range_m = 1.5

    def fake_estimator(raw, calibration, **kwargs):
        assert raw == b"raw"
        assert calibration is cal
        assert kwargs["ball_speed_mph"] == 101.0
        return SimpleNamespace(
            accepted=True,
            status="accepted",
            angle_deg=19.2,
            raw_angle_deg=19.2,
            component_std_deg=0.8,
            n_snapshots=20,
            n_frames=6,
            track_speed_mph=100.0,
            track_rms_bins=0.2,
            track_inliers=44,
        )

    monkeypatch.setattr(
        "openflight.iwr6843.replay.process_dump",
        lambda *_args, **_kwargs: SimpleNamespace(
            track=SimpleNamespace(bin_at=lambda _t: 33.0),
            geometry=SimpleNamespace(range_res_m=0.05),
        ),
    )

    record = replay_capture(
        ReplayInput(
            source="single",
            shot_number=4,
            capture_path=dump,
            ball_speed_mph=101.0,
            club="7i",
        ),
        cal,
        net_range_m=4.6,
        tx_order="normal",
        estimator=fake_estimator,
    )

    assert record.accepted
    assert record.launch_angle_deg == 19.2
    assert record.track_start_range_m == pytest.approx(1.65)


def test_replay_summary_and_csv_output(tmp_path):
    records = [
        SimpleNamespace(
            accepted=True,
            launch_angle_deg=20.0,
            component_std_deg=1.0,
            track_rms_bins=0.2,
            track_start_range_m=1.5,
            to_dict=lambda: {"shot_number": 1, "status": "accepted"},
        ),
        SimpleNamespace(
            accepted=False,
            launch_angle_deg=None,
            component_std_deg=None,
            track_rms_bins=None,
            track_start_range_m=None,
            to_dict=lambda: {"shot_number": 2, "status": "rejected"},
        ),
    ]

    summary = replay_summary(records)

    assert summary["shots"] == 2
    assert summary["accepted_shots"] == 1
    assert summary["coverage"] == 0.5
    assert summary["median_launch_angle_deg"] == 20.0

    out = tmp_path / "replay.jsonl"
    write_records(records, out, "jsonl")
    assert len(out.read_text(encoding="utf-8").splitlines()) == 2
