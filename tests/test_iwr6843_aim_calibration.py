"""Tests for the static IWR6843 horizontal aim utility."""

import json
import math
import sys

import numpy as np
import pytest

from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.dump import SAMPLE_RANGE_FFT_IQ16, pack_dump
from scripts import calibrate_iwr6843_aim as aim_script
from scripts.calibrate_iwr6843_aim import (
    AimEstimate,
    _alignment_report,
    _corrected_bearing_deg,
    differential_reflector_readings,
    fuse_readings,
    solve_two_position_target_line,
    static_reflector_readings,
)


def _static_horizontal_dump(angle_deg: float) -> bytes:
    frames = 4
    loops = 6
    n_tx = 3
    n_rx = 4
    n_bins = 128
    cube = np.ones((frames, loops * n_tx, n_rx, n_bins), dtype=complex)
    phase = -2.0 * np.pi * np.sin(np.radians(angle_deg))
    target_bin = 40
    for frame in range(frames):
        for loop in range(loops):
            cube[frame, loop * n_tx + 0, :, target_bin] = 1000.0
            cube[frame, loop * n_tx + 1, :, target_bin] = 1000.0 * np.exp(1j * phase)
            cube[frame, loop * n_tx + 2, :, target_bin] = 1000.0
    return pack_dump(
        cube,
        n_tx=n_tx,
        version=3,
        frame_period_us=4000,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16,
    )


def _static_dump_with_reference_separation(separation_deg: float) -> bytes:
    """Build a target whose TX1/TX3 reference channels nearly cancel."""
    frames = 4
    loops = 6
    n_tx = 3
    n_rx = 4
    n_bins = 128
    cube = np.ones((frames, loops * n_tx, n_rx, n_bins), dtype=complex)
    half_separation = np.radians(separation_deg) / 2.0
    target_bin = 40
    for frame in range(frames):
        for loop in range(loops):
            cube[frame, loop * n_tx + 0, :, target_bin] = 1000.0 * np.exp(-1j * half_separation)
            cube[frame, loop * n_tx + 1, :, target_bin] = 1000.0
            cube[frame, loop * n_tx + 2, :, target_bin] = 1000.0 * np.exp(1j * half_separation)
    return pack_dump(
        cube,
        n_tx=n_tx,
        version=3,
        frame_period_us=4000,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16,
    )


def _static_dump_with_rx_phase_spread() -> bytes:
    """Build a temporally stable target whose RX channels disagree."""
    frames = 4
    loops = 6
    n_tx = 3
    n_rx = 4
    n_bins = 128
    cube = np.ones((frames, loops * n_tx, n_rx, n_bins), dtype=complex)
    target_bin = 40
    rx_phases = np.asarray([-0.7, -0.2, 0.2, 0.7])
    for frame in range(frames):
        for loop in range(loops):
            cube[frame, loop * n_tx + 0, :, target_bin] = 1000.0
            cube[frame, loop * n_tx + 1, :, target_bin] = 1000.0 * np.exp(1j * rx_phases)
            cube[frame, loop * n_tx + 2, :, target_bin] = 1000.0
    return pack_dump(
        cube,
        n_tx=n_tx,
        version=3,
        frame_period_us=4000,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16,
    )


def _static_scene_dump(
    *,
    holder: bool,
    sphere_angle_deg: float | None,
    complex_scale: complex = 1.0,
    clutter_phase_slope_delta: float = 0.0,
) -> bytes:
    """Build repeatable room clutter with optional holder and sphere returns."""
    frames = 4
    loops = 6
    n_tx = 3
    n_rx = 4
    n_bins = 64
    cube = np.empty((frames, loops * n_tx, n_rx, n_bins), dtype=complex)
    bins = np.arange(n_bins)
    for frame in range(frames):
        for loop in range(loops):
            for tx in range(n_tx):
                for rx in range(n_rx):
                    magnitude = 80.0 + 2.0 * bins + 5.0 * tx + rx
                    phase = (0.013 + clutter_phase_slope_delta) * bins + 0.11 * tx - 0.07 * rx
                    cube[frame, loop * n_tx + tx, rx] = magnitude * np.exp(1j * phase)

    if holder:
        holder_bin = 41
        holder_phase = -2.0 * np.pi * np.sin(np.radians(-4.0))
        for frame in range(frames):
            for loop in range(loops):
                cube[frame, loop * n_tx + 0, :, holder_bin] += 1200.0
                cube[frame, loop * n_tx + 1, :, holder_bin] += 1200.0 * np.exp(1j * holder_phase)
                cube[frame, loop * n_tx + 2, :, holder_bin] += 1200.0

    if sphere_angle_deg is not None:
        sphere_bin = 40
        sphere_phase = -2.0 * np.pi * np.sin(np.radians(sphere_angle_deg))
        for frame in range(frames):
            for loop in range(loops):
                cube[frame, loop * n_tx + 0, :, sphere_bin] += 700.0
                cube[frame, loop * n_tx + 1, :, sphere_bin] += 700.0 * np.exp(1j * sphere_phase)
                cube[frame, loop * n_tx + 2, :, sphere_bin] += 700.0

    return pack_dump(
        cube * complex_scale,
        n_tx=n_tx,
        version=3,
        frame_period_us=4000,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16,
    )


def test_static_reflector_recovers_horizontal_bearing():
    raw = _static_horizontal_dump(8.0)

    readings = static_reflector_readings(
        raw,
        Calibration.identity(8),
        expected_range_m=1.875,
        gate_half_m=0.15,
    )
    estimate = fuse_readings(readings)

    assert len(readings) == 4
    assert estimate.range_m == pytest.approx(1.875)
    assert estimate.raw_bearing_deg == pytest.approx(8.0, abs=0.01)
    assert estimate.bearing_stability_deg == pytest.approx(0.0, abs=1e-12)
    assert estimate.coherence == pytest.approx(1.0)
    assert estimate.median_rx_coherence == pytest.approx(1.0)


def test_static_reflector_rejects_destructive_tx_reference_cancellation():
    raw = _static_dump_with_reference_separation(165.0)

    with pytest.raises(ValueError, match="TX1/TX3 reference cancellation"):
        static_reflector_readings(
            raw,
            Calibration.identity(8),
            expected_range_m=1.875,
            gate_half_m=0.15,
        )


def test_static_reflector_rejects_stable_rx_phase_disagreement():
    raw = _static_dump_with_rx_phase_spread()

    with pytest.raises(
        ValueError,
        match=r"RX phase disagreement.*median rx=.*requires >=0\.980",
    ):
        static_reflector_readings(
            raw,
            Calibration.identity(8),
            expected_range_m=1.875,
            gate_half_m=0.15,
        )


def test_differential_reflector_separates_sphere_from_stronger_holder():
    holder_captures = [
        _static_scene_dump(holder=True, sphere_angle_deg=None),
        _static_scene_dump(
            holder=True,
            sphere_angle_deg=None,
            complex_scale=1.03 * np.exp(-0.02j),
        ),
    ]
    sphere_captures = [
        _static_scene_dump(
            holder=True,
            sphere_angle_deg=6.0,
            complex_scale=0.97 * np.exp(0.04j),
        ),
        _static_scene_dump(
            holder=True,
            sphere_angle_deg=6.0,
            complex_scale=1.01 * np.exp(0.02j),
        ),
    ]

    readings = differential_reflector_readings(
        holder_captures,
        sphere_captures,
        Calibration.identity(8),
        expected_range_m=1.90,
        gate_half_m=0.15,
    )
    estimate = fuse_readings(readings)

    assert len(readings) == 2
    assert {reading.absolute_bin for reading in readings} == {40}
    assert estimate.range_m == pytest.approx(1.875)
    assert estimate.raw_bearing_deg == pytest.approx(6.0, abs=0.05)
    assert estimate.median_rx_coherence == pytest.approx(1.0, abs=1e-3)
    assert estimate.median_reference_balance == pytest.approx(1.0, abs=1e-3)


def test_differential_reflector_rejects_changed_background():
    holder = _static_scene_dump(holder=True, sphere_angle_deg=None)
    changed = _static_scene_dump(
        holder=True,
        sphere_angle_deg=6.0,
        clutter_phase_slope_delta=0.4,
    )

    with pytest.raises(ValueError, match="background alignment"):
        differential_reflector_readings(
            [holder],
            [changed],
            Calibration.identity(8),
            expected_range_m=1.90,
            gate_half_m=0.15,
        )


def test_three_stage_cli_saves_isolated_sphere_reference(monkeypatch, tmp_path):
    captures = iter(
        [
            _static_scene_dump(holder=False, sphere_angle_deg=None),
            _static_scene_dump(holder=True, sphere_angle_deg=None),
            _static_scene_dump(holder=True, sphere_angle_deg=6.0),
        ]
    )

    class FakeRadar:
        port = "/dev/fake"

        def __init__(self, *, port):
            assert port is None

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def send_config(self, path):
            assert path == "fake.cfg"

        def read_dump(self):
            return next(captures)

    cal_path = tmp_path / "cal.json"
    cal_path.write_text(
        json.dumps(
            {
                "elem_phase_rad": [0.0] * 8,
                "elem_gain": [1.0] * 8,
                "tilt_deg": 0.0,
                "range_bias_const_m": 0.0,
            }
        ),
        encoding="utf-8",
    )
    reference_path = tmp_path / "sphere.json"
    outdir = tmp_path / "captures"
    monkeypatch.setattr(aim_script, "IWR6843Radar", FakeRadar)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "calibrate_iwr6843_aim.py",
            "--cfg",
            "fake.cfg",
            "--cal",
            str(cal_path),
            "--reflector-range-m",
            "1.90",
            "--gate-half-m",
            "0.15",
            "--captures",
            "1",
            "--interval-s",
            "0",
            "--sphere-three-stage",
            "--no-prompt",
            "--outdir",
            str(outdir),
            "--save-reference",
            str(reference_path),
        ],
    )

    assert aim_script.main() == 0

    saved = json.loads(reference_path.read_text(encoding="utf-8"))
    assert saved["method"] == "three_stage_complex_background_subtraction"
    assert saved["raw_bearing_deg"] == pytest.approx(6.0, abs=0.05)
    assert len(list(outdir.glob("*.l3dump"))) == 3


def test_saved_target_phase_removes_electrical_offset():
    target_phase = -0.42

    assert _corrected_bearing_deg(target_phase, target_phase) == pytest.approx(0.0)
    assert math.isfinite(_corrected_bearing_deg(target_phase + 0.1, target_phase))


def test_live_alignment_reports_cross_range_and_turn_direction():
    estimate = _aim_estimate(2.0, 5.0)

    report = _alignment_report(estimate, azimuth_offset_deg=None, reference=None)

    assert report.aim_error_deg == pytest.approx(5.0)
    assert report.lateral_error_cm == pytest.approx(200.0 * math.sin(math.radians(5.0)))
    assert report.direction == "RIGHT"


def test_live_alignment_applies_known_azimuth_offset():
    estimate = _aim_estimate(2.0, 3.0)

    report = _alignment_report(estimate, azimuth_offset_deg=-3.0, reference=None)

    assert report.aim_error_deg == pytest.approx(0.0)
    assert report.lateral_error_cm == pytest.approx(0.0)
    assert report.direction == "CENTERED"


def _aim_estimate(range_m: float, raw_bearing_deg: float) -> AimEstimate:
    return AimEstimate(
        range_m=range_m,
        phase_rad=-2.0 * math.pi * math.sin(math.radians(raw_bearing_deg)),
        raw_bearing_deg=raw_bearing_deg,
        bearing_stability_deg=0.1,
        coherence=0.95,
        median_rx_coherence=0.98,
        median_power_ratio=12.0,
        readings=12,
    )


def _polar_target(
    along_line_m: float,
    *,
    line_bearing_deg: float,
    lateral_offset_m: float,
) -> AimEstimate:
    line = math.radians(line_bearing_deg)
    along_x = math.cos(line)
    along_y = math.sin(line)
    normal_x = -along_y
    normal_y = along_x
    x = along_line_m * along_x + lateral_offset_m * normal_x
    y = along_line_m * along_y + lateral_offset_m * normal_y
    return _aim_estimate(math.hypot(x, y), math.degrees(math.atan2(y, x)))


def test_two_position_solve_recovers_target_line_and_lateral_offset():
    near = _polar_target(
        1.5,
        line_bearing_deg=-3.0,
        lateral_offset_m=0.12,
    )
    far = _polar_target(
        4.5,
        line_bearing_deg=-3.0,
        lateral_offset_m=0.12,
    )

    result = solve_two_position_target_line(near, far)

    assert result.raw_target_line_bearing_deg == pytest.approx(-3.0)
    assert result.azimuth_offset_deg == pytest.approx(3.0)
    assert result.lateral_offset_m == pytest.approx(0.12)
    assert result.target_separation_m == pytest.approx(3.0)


def test_two_position_solve_is_independent_of_capture_order():
    near = _polar_target(
        1.5,
        line_bearing_deg=2.0,
        lateral_offset_m=-0.08,
    )
    far = _polar_target(
        4.0,
        line_bearing_deg=2.0,
        lateral_offset_m=-0.08,
    )

    forward = solve_two_position_target_line(near, far)
    reversed_result = solve_two_position_target_line(far, near)

    assert reversed_result.raw_target_line_bearing_deg == pytest.approx(
        forward.raw_target_line_bearing_deg
    )
    assert reversed_result.azimuth_offset_deg == pytest.approx(forward.azimuth_offset_deg)
    assert reversed_result.lateral_offset_m == pytest.approx(forward.lateral_offset_m)


def test_two_position_solve_rejects_targets_without_useful_separation():
    near = _aim_estimate(2.0, 1.0)
    almost_same = _aim_estimate(2.05, 1.0)

    with pytest.raises(ValueError, match="at least 0.50 m"):
        solve_two_position_target_line(near, almost_same)
