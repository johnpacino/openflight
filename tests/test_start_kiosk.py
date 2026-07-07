"""Tests for the kiosk entry script flag wiring."""

import subprocess


def _dry_run(*args: str, check: bool = True):
    repo_root = __file__.rsplit("/tests/", 1)[0]
    return subprocess.run(
        ["bash", "scripts/start-kiosk.sh", *args, "--dry-run"],
        cwd=repo_root,
        check=check,
        capture_output=True,
        text=True,
    )


def test_kld7_requires_mount_tilt():
    """--kld7 without a mount tilt must fail loudly rather than assume a default."""
    result = _dry_run("--kld7", check=False)
    assert result.returncode != 0
    assert "mount tilt is unset" in (result.stdout + result.stderr)


def test_plain_kld7_enables_two_ray_defaults():
    """--kld7 (with the required tilt) forwards the cleaned-up flag set."""
    result = _dry_run("--kld7", "--kld7-mount-tilt", "10")
    command = result.stdout.strip()

    assert "--kld7 --kld7-port /dev/kld7_vertical" in command
    # Boresight offset defaults to the calibrated 1.5, not the old 8.
    assert "--kld7-angle-offset 1.5" in command
    assert "--kld7-mount-tilt 10" in command
    # The estimator is a fixed cascade now — no selection flag.
    assert "--kld7-vertical-estimator" not in command
    # Gating is on by default; raw mode is opt-in.
    assert "--kld7-vertical-raw" not in command
    # Cosine correction rides on --kld7 server-side, not a kiosk flag.
    assert "--ball-speed-cosine-correction" not in command


def test_kld7_angle_offset_override_wins():
    """An explicit boresight offset overrides the 1.5 default."""
    result = _dry_run("--kld7", "--kld7-mount-tilt", "10", "--kld7-angle-offset", "3.5")
    command = result.stdout.strip()

    assert "--kld7-angle-offset 3.5" in command
    assert "--kld7-angle-offset 1.5" not in command


def test_kld7_vertical_raw_flag_forwarded():
    """--kld7-vertical-raw reaches the server as the renamed raw flag."""
    result = _dry_run("--kld7", "--kld7-mount-tilt", "10", "--kld7-vertical-raw")
    assert "--kld7-vertical-raw" in result.stdout


def test_trackman_test_dry_run_enables_raw_capture():
    """The field preset captures raw replay data and forwards the clean flags."""
    result = _dry_run("--trackman-test", "--kld7-mount-tilt", "10")
    command = result.stdout.strip()

    assert command.startswith("openflight-server --web-port 8080")
    assert "--session-location trackman" in command
    assert "--kld7-raw-logging" in command
    assert "--experimental-kld7-radc-tuning" not in command
    assert "--kld7 --kld7-port /dev/kld7_vertical --kld7-angle-offset 1.5" in command
    assert "--kld7-mount-tilt 10" in command
    assert "--kld7-horizontal" in command
    assert "--kld7-horizontal-port /dev/kld7_horizontal" in command
    assert "--no-camera" in command
    assert "--trigger sound" in command
    # No legacy estimator selection survives.
    assert "--kld7-vertical-estimator" not in command


def test_trackman_test_allows_explicit_session_location():
    """A bay/location override should survive the TrackMan preset defaults."""
    result = _dry_run("--trackman-test", "--kld7-mount-tilt", "10", "--session-location", "bay-2")

    assert "--session-location bay-2" in result.stdout
    assert "--session-location trackman " not in result.stdout


def test_startup_applies_kld7_latency_setup_before_server_start():
    """Kiosk startup should attempt the FTDI latency setup for K-LD7 sessions."""
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[1]
    script = (repo_root / "scripts/start-kiosk.sh").read_text(encoding="utf-8")

    setup_idx = script.index("\nconfigure_kld7_latency\n")
    server_start_idx = script.index("$SERVER_CMD &")

    assert "scripts/setup/setup_kld7_latency.sh" in script
    assert 'sudo -n "$setup_script" --latency 1' in script
    assert setup_idx < server_start_idx


def test_radc_tuning_values_are_ignored_without_experimental_gate():
    """Loose tuning flags should not alter production extraction by accident."""
    result = _dry_run(
        "--experimental-kld7-speed-tolerance", "6", "--experimental-kld7-spectrum-source", "sum12"
    )

    assert "--experimental-kld7-speed-tolerance 6" not in result.stdout
    assert "--experimental-kld7-spectrum-source sum12" not in result.stdout
    assert "Ignoring experimental K-LD7 RADC tuning values" in result.stdout


def test_radc_tuning_values_are_forwarded_with_experimental_gate():
    """When explicitly enabled, replay-discovered RADC knobs reach the server."""
    result = _dry_run(
        "--kld7-raw-logging",
        "--experimental-kld7-radc-tuning",
        "--experimental-kld7-speed-tolerance",
        "6",
        "--experimental-kld7-spectrum-source",
        "sum12",
        "--experimental-kld7-horizontal-angle-limit",
        "30",
    )
    command = result.stdout.strip()

    assert "--kld7-raw-logging" in command
    assert "--experimental-kld7-radc-tuning" in command
    assert "--experimental-kld7-speed-tolerance 6" in command
    assert "--experimental-kld7-spectrum-source sum12" in command
    assert "--experimental-kld7-horizontal-angle-limit 30" in command
