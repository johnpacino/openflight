"""Tests for the K-LD7 motion onset diagnostic helpers."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts/analysis/kld7_motion_onset_test.py"
SPEC = importlib.util.spec_from_file_location("kld7_motion_onset_test", SCRIPT_PATH)
assert SPEC is not None
motion = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = motion
SPEC.loader.exec_module(motion)


def test_post_trigger_duration_matches_s16_split():
    assert motion.post_trigger_duration_ms(16) == pytest_approx(68.2666667)


def test_target_onset_prefers_first_any_target():
    trigger_ts = 1000.0
    frames = [
        {"timestamp": trigger_ts - 0.020, "pdat": [], "tdat": None},
        {"timestamp": trigger_ts + 0.010, "pdat": [{"distance": 1.0}], "tdat": None},
        {"timestamp": trigger_ts + 0.040, "pdat": [], "tdat": {"distance": 1.2}},
    ]

    summary = motion.summarize_target_onset(frames, trigger_timestamp=trigger_ts)

    assert summary["first_target_ms"] == pytest_approx(10.0)
    assert summary["first_pdat_ms"] == pytest_approx(10.0)
    assert summary["first_tdat_ms"] == pytest_approx(40.0)
    assert summary["frames_with_targets"] == 2


def test_radc_onset_uses_baseline_scaled_threshold():
    trigger_ts = 1000.0
    frames = [
        {
            "timestamp": trigger_ts - 0.100,
            "radc_metric": {"snr": 5.0, "peak_bin": 1, "peak_velocity_kmh": 0.1},
        },
        {
            "timestamp": trigger_ts - 0.050,
            "radc_metric": {"snr": 6.0, "peak_bin": 2, "peak_velocity_kmh": 0.2},
        },
        {
            "timestamp": trigger_ts + 0.010,
            "radc_metric": {"snr": 7.0, "peak_bin": 3, "peak_velocity_kmh": 0.3},
        },
        {
            "timestamp": trigger_ts + 0.040,
            "radc_metric": {"snr": 9.0, "peak_bin": 4, "peak_velocity_kmh": 0.4},
        },
    ]

    summary = motion.summarize_radc_onset(
        frames,
        trigger_timestamp=trigger_ts,
        min_snr=4.0,
        baseline_factor=1.5,
    )

    assert summary["baseline_median_snr"] == pytest_approx(5.5)
    assert summary["threshold_snr"] == pytest_approx(8.25)
    assert summary["first_motion"]["t_ms"] == pytest_approx(40.0)
    assert summary["strongest_motion"]["peak_bin"] == 4


def pytest_approx(value):
    import pytest

    return pytest.approx(value)
