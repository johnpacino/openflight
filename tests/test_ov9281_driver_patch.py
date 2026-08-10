"""Contract tests for the custom OV9281 high-speed driver patch."""

from pathlib import Path


def test_320x200_vertical_offset_preserves_native_sensor_window():
    patch = (Path(__file__).parents[1] / "drivers/ov9281/ov9282-high-speed.patch").read_text()

    assert "mode->width == 320 && mode->height == 200" in patch
    assert "y_start = 300 + 2 * offset" in patch
    assert "y_end = 815 + 2 * offset" in patch
    assert "return clamp(strip_y_offset, -150, 0)" in patch
