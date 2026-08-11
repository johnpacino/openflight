"""Contract tests for the custom OV9281 high-speed driver patch."""

from pathlib import Path


def test_320x200_vertical_offset_preserves_native_sensor_window():
    patch = (Path(__file__).parents[1] / "drivers/ov9281/ov9282-high-speed.patch").read_text()

    assert "mode->width == 320 && mode->height == 200" in patch
    assert "y_start = 300 + 2 * offset" in patch
    assert "y_end = 815 + 2 * offset" in patch
    assert "return clamp(strip_y_offset, -150, 0)" in patch


def test_vertical_offset_uses_kernel_safe_group_writable_permissions():
    patch = (Path(__file__).parents[1] / "drivers/ov9281/ov9282-high-speed.patch").read_text()

    assert "module_param(strip_y_offset, int, 0664)" in patch
    assert "module_param(strip_y_offset, int, 0666)" not in patch


def test_installer_uses_running_kernel_headers_without_full_kernel_build():
    installer = (
        Path(__file__).parents[1] / "scripts/setup/install_ov9281_high_speed_driver.sh"
    ).read_text()

    assert 'KERNEL_BUILD="/lib/modules/$KERNEL_RELEASE/build"' in installer
    assert 'git apply --recount "$PATCH_FILE"' in installer
    assert 'make -C "$SOURCE_DIR" -j"$JOBS"' not in installer
