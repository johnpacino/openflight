"""Tests for the IWR6843 CLI and dump serial contract."""

from __future__ import annotations

import pytest

from openflight.iwr6843.driver import IWR6843Radar


def test_send_config_rejects_missing_cli_acknowledgement(tmp_path, monkeypatch):
    """A wedged board must not be reported as configured and armed."""
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: "")

    with pytest.raises(RuntimeError, match="did not acknowledge"):
        radar.send_config(str(config))
