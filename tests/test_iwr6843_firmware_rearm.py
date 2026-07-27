"""Regression checks for the HWA snapshot ring freeze/rearm sequence."""

from __future__ import annotations

import re
from pathlib import Path

FIRMWARE = Path(__file__).parents[1] / "firmware" / "iwr6843" / "l3_dump.c"
FIRMWARE_MAKEFILE = Path(__file__).parents[1] / "firmware" / "Makefile"
WINDOW53_12L18F_CONFIG = (
    Path(__file__).parents[1] / "config" / "iwr6843_l3dump_vTX2_window53_12l18f.cfg"
)


def _function_source(source: str, name: str, next_name: str) -> str:
    start = source.rindex(name)
    end = source.index(next_name, start)
    return source[start:end]


def test_hwa_dump_stops_at_boundary_before_streaming():
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    stop = dump.index("l3_stopCaptureAtBoundary")
    stream = dump.index("UART_writePolling")

    assert stop < stream


def test_hwa_chain_processes_and_rearms_one_frame_at_a_time():
    source = FIRMWARE.read_text(encoding="utf-8")
    common = _function_source(source, "static int32_t l3_configHwaCommon", "static void l3_drain")
    output = _function_source(
        source,
        "static int32_t l3_configHwaOutputEdma",
        "static int32_t l3_configHwaSignatureEdma",
    )

    assert "commonCfg.numLoops = CHIRPS_PER_FRAME / 2U" in common
    assert "param->cCount = (uint16_t)(CHIRPS_PER_FRAME / 2U)" in output


def test_completed_frame_advances_circular_ring_slot():
    source = FIRMWARE.read_text(encoding="utf-8")
    callback = _function_source(
        source,
        "static void l3_hwaOutputDoneCB",
        "static int32_t l3_hwaStartRing",
    )

    assert "gRingFrame++" in callback
    assert "gRingFrame % RING_FRAMES" in callback


def test_freeze_request_keeps_rearming_until_post_trigger_target():
    source = FIRMWARE.read_text(encoding="utf-8")
    queue = _function_source(
        source, "static void l3_hwaMaybeQueueRearm", "static void l3_hwaChainDoneCB"
    )
    freeze = _function_source(
        source,
        "static int32_t l3_freezeHwaAfterPostFrames",
        "static int32_t l3_armHwaChain",
    )

    assert "gHwaFreezeRequested" in queue
    assert "gRingFrame >= gHwaFreezeTargetFrame" in queue
    assert "Semaphore_post(gHwaFreezeSemaphore)" in queue
    assert "gHwaFreezeTargetFrame = gRingFrame + HWA_POST_TRIGGER_FRAMES" in freeze


def test_sensor_stop_uses_completed_frame_boundary_before_teardown():
    source = FIRMWARE.read_text(encoding="utf-8")
    stop_helper = _function_source(
        source,
        "static int32_t l3_stopCaptureAtBoundary",
        "static int32_t l3_armHwaChain",
    )
    sensor_stop = _function_source(
        source,
        "static int32_t l3_cli_sensorStop",
        "static void l3_initTask",
    )

    freeze = stop_helper.index("l3_freezeHwaAfterPostFrames")
    rf_stop = stop_helper.index("MMWave_stop")
    hwa_disable = stop_helper.index("HWA_enable(gHwaHandle, 0U)")
    inactive = stop_helper.index("gCaptureActive = 0U")

    assert freeze < rf_stop < hwa_disable < inactive
    assert "if (MMWave_stop(gMMWaveHandle, &errCode) < 0)" in stop_helper
    assert "if (!gCaptureActive)" in sensor_stop
    assert "return l3_stopCaptureAtBoundary()" in sensor_stop


def test_dump_header_rotates_from_oldest_completed_frame():
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    assert "gRingFrame % RING_FRAMES" in dump


def test_production_build_uses_balanced_geometry_and_single_release():
    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")
    target = _function_source(source, "build-native:", "clean:")

    assert "--define=N_TX=3" in target
    assert "--define=LOOPS=12" in target
    assert "--define=RING_FRAMES=25" in target
    assert "--define=HWA_POST_TRIGGER_FRAMES=12" in target
    assert "--define=SNAPSHOT_DYNAMIC_WINDOWS=1" in target
    assert "--define=SNAPSHOT_BIN_START=20" in target
    assert "--define=SNAPSHOT_MIDDLE_BIN_START=32" in target
    assert "--define=SNAPSHOT_LATE_BIN_START=47" in target
    assert "--define=SNAPSHOT_BINS=53" in target
    assert "RELEASE_NAME ?= l3_dump_vTX2_hwa_window53_12loops_25frames_4ms_v3.bin" in source
    assert '"$(RELEASE_DIR)/$(RELEASE_NAME)"' in target
    assert source.count("\nbuild-native:") == 1


def test_window53_12_loop_18_frame_config_matches_firmware_geometry():
    lines = {
        line.strip()
        for line in WINDOW53_12L18F_CONFIG.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("%")
    }

    assert "frameCfg 0 2 12 0 4 1 0" in lines
    assert "chirpCfg 0 0 0 0 0 0 0 1" in lines
    assert "chirpCfg 1 1 0 0 0 0 0 2" in lines
    assert "chirpCfg 2 2 0 0 0 0 0 4" in lines


def test_dynamic_window_start_is_recorded_per_ring_slot():
    source = FIRMWARE.read_text(encoding="utf-8")
    output = _function_source(
        source,
        "static int32_t l3_configHwaFrameOutput",
        "static void l3_drainHwaRearmSemaphore",
    )
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    assert "gFrameBinStart[ringSlot % RING_FRAMES]" in output
    assert "UART_writePolling(gDataUart, gFrameBinStart" in dump


def test_ring_fits_in_l3_ram():
    """The capture ring must fit in the 768 KiB of L3 the linker gives it.

    Overflowing shows up as a link failure, which only bites whoever runs the
    TI toolchain -- so it can sit unnoticed in a commit for a long time. This
    computes the budget from the build definition instead.

    `.l3scratch` (`g_rawFrame`) is allocated only under LIVE_SNAPSHOT_RING, and
    the production target selects HWA_CHAINED_SNAPSHOT_RING, so the whole
    region is available to `g_ring`.
    """
    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")
    target = _function_source(source, "build-native:", "clean:")

    def define(name: str) -> int:
        match = re.search(rf"--define={name}=(\d+)", target)
        assert match is not None, f"{name} is not set by the production build"
        return int(match.group(1))

    assert "--define=HWA_CHAINED_SNAPSHOT_RING=1" in target, (
        "this budget assumes .l3scratch is unused; LIVE_SNAPSHOT_RING would "
        "also allocate two raw frames there"
    )

    l3_ram_bytes = 768 * 1024
    bytes_per_frame = define("N_TX") * define("LOOPS") * 4 * define("SNAPSHOT_BINS") * 4
    ring_bytes = bytes_per_frame * define("RING_FRAMES")

    assert bytes_per_frame == 30_528, bytes_per_frame
    assert ring_bytes == 763_200, ring_bytes
    assert ring_bytes <= l3_ram_bytes, (
        f"ring is {ring_bytes} bytes, over the {l3_ram_bytes}-byte L3 region; "
        f"at {bytes_per_frame} bytes/frame the ceiling is "
        f"{l3_ram_bytes // bytes_per_frame} frames"
    )


def test_post_trigger_tail_leaves_room_for_approach_history():
    """Pre-trigger frames are what club path spends, so pin the split.

    The freeze is requested by a UART CLI command, so the trigger frame lands
    late by a variable 2-4 frames (measured 2026-07-25: impact at slot 1.7-4.1
    of an 18-frame ring against an assumed slot 6). The pre-trigger allocation
    has to absorb that and still leave more than CLUB_MIN_FRAMES behind.
    """
    from openflight.iwr6843.club import CLUB_MIN_FRAMES

    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")
    target = _function_source(source, "build-native:", "clean:")
    ring = int(re.search(r"--define=RING_FRAMES=(\d+)", target).group(1))
    post = int(re.search(r"--define=HWA_POST_TRIGGER_FRAMES=(\d+)", target).group(1))

    assert post < ring, "the firmware requires at least one pre-trigger frame"
    pre_trigger = ring - post
    worst_case_latency_frames = 4
    assert pre_trigger - worst_case_latency_frames > CLUB_MIN_FRAMES, (
        f"{pre_trigger} pre-trigger frames minus {worst_case_latency_frames} "
        f"of trigger latency leaves {pre_trigger - worst_case_latency_frames}, "
        f"which does not clear CLUB_MIN_FRAMES={CLUB_MIN_FRAMES}"
    )


def test_build_native_fails_fast_on_a_wrong_host():
    """`build-native` must check the host and toolchain before compiling.

    Without the prerequisite, running it on a non-Linux host (or with the SDK
    absent) dives straight into the TI makefile and dies with
    "No rule to make target .../mmwave_sdk.mak", which does not say that the
    host is wrong or that the SDK is missing.
    """
    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")

    assert re.search(r"^build-native:\s*check-tools\s*$", source, re.MULTILINE), (
        "build-native must depend on check-tools"
    )

    check = _function_source(source, "check-tools:", "build-native:")
    assert 'uname -s' in check and 'uname -m' in check, "the host must be verified"
    assert "MMWAVE_SDK_INSTALL_PATH" in check
    assert "R4F_CODEGEN_INSTALL_PATH" in check
    assert "XWR68XX_RADARSS_IMAGE_BIN" in check
    assert "exit 1" in check, "check-tools must fail, not just warn"


DOCKERFILE = Path(__file__).parents[1] / "firmware" / "Dockerfile"


def test_container_reuses_the_make_targets_instead_of_copying_them():
    """The Dockerfile must not restate the dep list or installer invocations.

    Two copies of "how the TI toolchain is installed" would drift the moment
    either path changed, and the container path is the one most people use, so
    the bare-metal path would rot silently.
    """
    dockerfile = DOCKERFILE.read_text(encoding="utf-8")

    assert "install-ti-deps-native" in dockerfile
    assert "install-ti-tools-native" in dockerfile
    # CMD is exec-form, so match the tokens rather than a shell string.
    assert re.search(r'CMD\s*\[.*"build-native".*\]', dockerfile), (
        "the container must run the same build-native recipe, not its own copy"
    )
    for leaked in ("libc6:i386", "--mode unattended", "SNAPSHOT_BINS", "RING_FRAMES"):
        assert leaked not in dockerfile, (
            f"{leaked!r} is defined in firmware/Makefile; the Dockerfile must "
            "call the make target rather than duplicate it"
        )


def test_container_verifies_the_environment_ti_installers_require():
    """x86_64, 4096-byte pages, and 32-bit execution are all load-bearing.

    Each has a real failure mode: Apple Silicon and the Pi are not x86_64, the
    Pi 5 uses 16 KiB pages, and Docker Desktop's Rosetta backend cannot run
    i386. Checking them in the image turns each into a build-time error.
    """
    dockerfile = DOCKERFILE.read_text(encoding="utf-8")

    assert 'test "$(uname -m)" = "x86_64"' in dockerfile
    assert 'test "$(getconf PAGE_SIZE)" = "4096"' in dockerfile
    assert "/lib/ld-linux.so.2" in dockerfile, "i386 execution must be verified"
    assert dockerfile.count("exit 1") >= 3, "each check must fail the build"


def test_container_does_not_bake_the_license_gated_installers_into_layers():
    """The installers are multi-gigabyte and license-gated.

    A plain COPY would persist them in an image layer even after deletion, so
    they are read through a build mount and removed inside the same RUN.
    """
    dockerfile = DOCKERFILE.read_text(encoding="utf-8")

    assert "--mount=type=bind" in dockerfile
    assert "COPY" not in dockerfile, "a COPY layer would retain the installers"
    assert "rm -rf /tmp/ti-installers" in dockerfile


def test_docker_targets_pin_the_amd64_platform():
    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")

    for target in ("docker-image:", "docker-build:", "docker-shell:"):
        assert f"\n{target}" in source, f"{target} is missing"
    # Three docker invocations, each explicitly amd64: the Dockerfile
    # deliberately does not pin the platform itself.
    assert source.count("--platform linux/amd64") == 3, source.count(
        "--platform linux/amd64"
    )


def test_fetch_installers_separates_automatic_from_login_gated():
    """Only the unauthenticated three may be scripted.

    The mmWave SDK and the ARM compiler need a TI login plus an export-control
    acceptance; a direct GET returns HTTP 200 with an HTML consent page.
    Listing either as fetchable would download that page and fail confusingly
    much later, inside `docker build`.
    """
    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")
    fetchable = _function_source(source, "FETCHABLE_INSTALLERS :=", "MANUAL_INSTALLERS :=")
    manual = _function_source(source, "MANUAL_INSTALLERS :=", "REQUIRED_INSTALLERS :=")

    for name in (
        "bios_6_73_01_01.run",
        "sysconfig-1.10.0_2163-setup.run",
        "xdctools_3_61_00_16_core_linux.zip",
    ):
        assert name in fetchable, f"{name} is downloadable without a login"

    for name in (
        "mmwave_sdk_03_06_02_00-LTS-Linux-x86-Install.bin",
        "ti_cgt_tms470_20.2.7.LTS_linux-x64_installer.bin",
    ):
        assert name in manual, f"{name} is login-gated"
        assert name not in fetchable, (
            f"{name} is login-gated; a scripted GET returns an HTML page"
        )

    # Entries are name|url pairs and MUST stay single-quoted: unquoted, the
    # shell reads the '|' in `for entry in $(FETCHABLE_INSTALLERS)` as a pipe.
    for block in (fetchable, manual):
        for line in block.splitlines():
            stripped = line.strip().rstrip("\\").strip()
            if "|" in stripped and not stripped.startswith("#"):
                assert stripped.startswith("'") and stripped.endswith("'"), (
                    f"unquoted entry would be parsed as a shell pipeline: {stripped}"
                )


def test_fetch_installers_validates_what_it_downloaded():
    """A 200 response is not proof of a binary; check the magic bytes."""
    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")
    target = _function_source(source, "fetch-installers:", "docker-image:")

    assert "7f454c46|504b0304" in target, "must accept only ELF or ZIP"
    assert "curl -fL" in target, "-f so an HTTP error is not written to the file"
    assert ".part" in target, "download to a temp name, rename only once valid"
    assert "exit 1" in target, "must fail rather than leave a partial set"
