"""Regression checks for the HWA snapshot ring freeze/rearm sequence."""

from __future__ import annotations

import re
from pathlib import Path

FIRMWARE = Path(__file__).parents[1] / "firmware" / "iwr6843" / "l3_dump.c"
FIRMWARE_MAKEFILE = Path(__file__).parents[1] / "firmware" / "Makefile"
RELEASE_DIR = Path(__file__).parents[1] / "firmware" / "releases"
CONFIGURABLE_CONFIG = Path(__file__).parents[1] / "config" / "iwr6843_l3dump_vTX2_configurable.cfg"
HYBRID_CONFIG = Path(__file__).parents[1] / "config" / "iwr6843_l3dump_vTX2_hybrid.cfg"
CLUB16_CONFIG = Path(__file__).parents[1] / "config" / "iwr6843_l3dump_club16.cfg"
CLUB14_CONFIG = Path(__file__).parents[1] / "config" / "iwr6843_l3dump_club14.cfg"


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


def test_hwa_dump_supports_cooperative_host_cancellation():
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    assert "L3_DUMP_CANCEL_BYTE" in source
    assert "L3_DUMP_CANCEL_ACK" in source
    assert "l3_dumpCancelRequested" in dump
    assert "UART_writePolling(gDataUart, L3_DUMP_CANCEL_ACK" in dump


def test_hwa_chain_processes_and_rearms_one_frame_at_a_time():
    source = FIRMWARE.read_text(encoding="utf-8")
    common = _function_source(source, "static int32_t l3_configHwaCommon", "static void l3_drain")
    output = _function_source(
        source,
        "static int32_t l3_configHwaOutputEdma",
        "static int32_t l3_configHwaSignatureEdma",
    )

    assert "gCapturePlan.chirpsPerFrame / 2U" in common
    assert "gCapturePlan.chirpsPerFrame / 2U" in output


def test_completed_frame_advances_circular_ring_slot():
    source = FIRMWARE.read_text(encoding="utf-8")
    callback = _function_source(
        source,
        "static void l3_hwaOutputDoneCB",
        "static int32_t l3_hwaStartRing",
    )

    assert "gRingFrame++" in callback
    assert "gPreFramesCaptured++" in callback
    assert "gPostFramesCaptured++" in callback


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
    assert "gPostFramesCaptured >= gCapturePlan.postFrames" in queue
    assert "Semaphore_post(gHwaFreezeSemaphore)" in queue
    assert "gPostFramesCaptured = 0U" in freeze


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


def test_dump_streams_pre_ring_then_post_tail_in_chronological_order():
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    assert "oldestPre = " in dump
    assert "(oldestPre + i) % gCapturePlan.preFrames" in dump
    assert "gCapturePlan.preFrames + i" in dump
    assert "L3_SAMPLE_RANGE_FFT_IQ16_VARIABLE" in source


def test_production_build_uses_configurable_capture_and_single_release():
    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")
    target = _function_source(source, "build-native:", "clean:")

    assert "--define=N_TX=3" in target
    assert "--define=CONFIGURABLE_CAPTURE=1" in target
    assert "--define=HYBRID_CADENCE_CAPTURE=1" in target
    for fixed_geometry in (
        "--define=LOOPS=",
        "--define=RING_FRAMES=",
        "--define=HWA_POST_TRIGGER_FRAMES=",
        "--define=SNAPSHOT_BINS=",
    ):
        assert fixed_geometry not in target
    assert "RELEASE_NAME ?= l3_dump_hybrid_cadence_20260801.bin" in source
    assert '"$(RELEASE_DIR)/$(RELEASE_NAME)"' in target
    assert source.count("\nbuild-native:") == 1


def test_release_filenames_use_feature_and_build_date_without_version_tokens():
    pattern = re.compile(r"^[a-z0-9_]+_\d{8}\.bin$")

    for release in RELEASE_DIR.glob("*.bin"):
        assert pattern.fullmatch(release.name), release.name
        assert not re.search(r"(?:^|_)v\d+(?:_|\.)", release.name), release.name


def test_configurable_capture_config_requests_32_frames_at_3ms():
    lines = {
        line.strip()
        for line in CONFIGURABLE_CONFIG.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("%")
    }

    assert "frameCfg 0 2 12 0 3 1 0" in lines
    assert "captureCfg 20 32 32 53 47 16" in lines
    assert "chirpCfg 0 0 0 0 0 0 0 1" in lines
    assert "chirpCfg 1 1 0 0 0 0 0 2" in lines
    assert "chirpCfg 2 2 0 0 0 0 0 4" in lines


def test_hybrid_config_acquires_12_loops_at_2ms_and_retains_alternate_post_frames():
    lines = {
        line.strip()
        for line in HYBRID_CONFIG.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("%")
    }

    assert "frameCfg 0 2 12 0 2 1 0" in lines
    # Omit the optional stride so the config also works with the original
    # hybrid image, which hardcodes the same every-other-frame post cadence.
    assert "captureCfg 20 32 32 53 47 16" in lines
    assert "captureCfg 20 32 32 53 47 16 2" not in lines


def test_hybrid_timing_budget_and_retained_movie_length():
    commands = {
        line.split()[0]: line.split()
        for line in HYBRID_CONFIG.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("%")
    }
    profile = commands["profileCfg"]
    frame = commands["frameCfg"]
    capture = commands["captureCfg"]

    loops = int(frame[3])
    frame_period_us = int(frame[5]) * 1000
    chirp_us = float(profile[3]) + float(profile[5])
    rf_occupancy_us = 3 * loops * chirp_us
    post_frames = int(capture[6])
    source = FIRMWARE.read_text(encoding="utf-8")
    assert "#define L3_DEFAULT_POST_STRIDE 2U" in source
    post_stride = int(capture[7]) if len(capture) > 7 else 2
    acquired_post_frames = 1 + (post_frames - 1) * post_stride

    assert rf_occupancy_us == 1620
    assert frame_period_us - rf_occupancy_us == 380
    assert acquired_post_frames == 31
    assert 16 * frame_period_us + acquired_post_frames * frame_period_us == 94_000


def test_club16_config_uses_dense_impact_and_wide_late_flight_phases():
    commands = {
        line.split()[0]: line.split()
        for line in CLUB16_CONFIG.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("%")
    }

    profile = commands["profileCfg"]
    frame = commands["frameCfg"]
    phased = commands["phaseCaptureCfg"]
    loops = int(frame[3])
    frame_period_us = round(float(frame[5]) * 1000)
    chirp_us = float(profile[3]) + float(profile[5])
    rf_occupancy_us = 3 * loops * chirp_us

    assert loops == 16
    assert frame_period_us == 2_500
    assert rf_occupancy_us == 2_160
    assert frame_period_us - rf_occupancy_us == 340
    assert phased == [
        "phaseCaptureCfg",
        "23", "18", "8",       # pre: 3.4-6.2 ft, 8 frames
        "23", "18", "6",       # impact: same window, 6 dense frames
        "32", "53", "47",      # middle/late ball windows
        "14", "2",              # 14 retained ball frames at 5 ms
    ]

    bytes_per_bin = 3 * loops * 4 * 4
    used = (8 * 18 + 6 * 18 + 14 * 53) * bytes_per_bin
    assert used == 763_392
    assert used <= 768 * 1024


def test_club14_config_uses_validated_2_25ms_cadence_with_l3_headroom():
    commands = {
        line.split()[0]: line.split()
        for line in CLUB14_CONFIG.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("%")
    }

    profile = commands["profileCfg"]
    frame = commands["frameCfg"]
    phased = commands["phaseCaptureCfg"]
    loops = int(frame[3])
    frame_period_us = round(float(frame[5]) * 1000)
    chirp_us = float(profile[3]) + float(profile[5])
    rf_occupancy_us = 3 * loops * chirp_us

    assert loops == 14
    assert frame_period_us == 2_250
    assert rf_occupancy_us == 1_890
    assert frame_period_us - rf_occupancy_us == 360
    assert phased == [
        "phaseCaptureCfg",
        "23", "18", "8",       # pre: 3.4-6.2 ft, 8 frames
        "23", "18", "6",       # impact: same window, 6 dense frames
        "32", "53", "47",      # middle/late ball windows
        "14", "2",              # 14 retained ball frames at 4.5 ms
    ]

    bytes_per_bin = 3 * loops * 4 * 4
    used = (8 * 18 + 6 * 18 + 14 * 53) * bytes_per_bin
    assert used == 667_968
    assert used <= 768 * 1024


def test_phased_capture_keeps_dense_impact_before_decimating_ball_frames():
    source = FIRMWARE.read_text(encoding="utf-8")
    planner = _function_source(
        source,
        "static int32_t l3_finalizeCapturePlan",
        "/* captureCfg",
    )
    output = _function_source(
        source,
        "static int32_t l3_configHwaFrameOutput",
        "static void l3_drainHwaRearmSemaphore",
    )

    assert "gCapturePlan.impactFrames" in planner
    assert "gCapturePlan.requestedPreFrames" in planner
    assert "gCapturePlan.ballFrames" in planner
    assert "gFrameBytes[slot]" in planner
    assert "gPostFramesCaptured < gCapturePlan.impactFrames" in output
    assert "gCapturePlan.postStride" in output


def test_hybrid_firmware_counts_observed_and_retained_post_frames_separately():
    source = FIRMWARE.read_text(encoding="utf-8")
    callback = _function_source(
        source,
        "static void l3_hwaOutputDoneCB",
        "static int32_t l3_hwaStartRing",
    )
    output = _function_source(
        source,
        "static int32_t l3_configHwaFrameOutput",
        "static void l3_drainHwaRearmSemaphore",
    )
    descriptor = _function_source(
        source,
        "static void l3_writeFrameDescriptor",
        "#ifndef HWA_CHAINED_SNAPSHOT_RING",
    )

    assert "gPostFramesObserved++" in callback
    assert "gActiveFrameShouldKeep" in callback
    assert "gCapturePlan.postStride" in output
    assert "gFrameDeltaUs[slot]" in descriptor
    assert "L3_SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED" in source


def test_variable_window_start_and_count_are_recorded_per_frame():
    source = FIRMWARE.read_text(encoding="utf-8")
    output = _function_source(
        source,
        "static int32_t l3_configHwaFrameOutput",
        "static void l3_drainHwaRearmSemaphore",
    )
    descriptor = _function_source(
        source,
        "static void l3_writeFrameDescriptor",
        "#ifndef HWA_CHAINED_SNAPSHOT_RING",
    )

    assert "gFrameOffset[ringSlot]" in output
    assert "gCapturePlan.preBins" in output
    assert "gCapturePlan.postBins" in output
    assert "descriptor[0] = gFrameBinStart[slot]" in descriptor
    assert "descriptor[1] = gFrameBinCount[slot]" in descriptor


def test_configurable_ring_uses_exact_l3_arena_and_plan_fits():
    """The capture ring must fit in the 768 KiB of L3 the linker gives it.

    Overflowing shows up as a link failure, which only bites whoever runs the
    TI toolchain -- so it can sit unnoticed in a commit for a long time. This
    computes the budget from the build definition instead.

    `.l3scratch` (`g_rawFrame`) is allocated only under LIVE_SNAPSHOT_RING, and
    the production target selects HWA_CHAINED_SNAPSHOT_RING, so the whole
    region is available to `g_ring`.
    """
    firmware = FIRMWARE.read_text(encoding="utf-8")
    l3_ram_bytes = 768 * 1024
    assert "#define L3_CAPTURE_BYTES       (6U * 128U * 1024U)" in firmware
    assert "static uint8_t g_ring[L3_CAPTURE_BYTES]" in firmware

    pre_frame_bytes = 3 * 12 * 4 * 32 * 4
    post_frame_bytes = 3 * 12 * 4 * 53 * 4
    pre_frames = (l3_ram_bytes - 16 * post_frame_bytes) // pre_frame_bytes
    used = pre_frames * pre_frame_bytes + 16 * post_frame_bytes

    assert pre_frames == 16
    assert used == 783_360
    assert used <= l3_ram_bytes


def test_same_capture_config_safely_replans_for_16_loops():
    l3_ram_bytes = 768 * 1024
    bytes_per_bin = 3 * 16 * 4 * 4
    post_bytes = 16 * 53 * bytes_per_bin
    pre_frame_bytes = 32 * bytes_per_bin

    pre_frames = (l3_ram_bytes - post_bytes) // pre_frame_bytes
    used = pre_frames * pre_frame_bytes + post_bytes

    assert pre_frames == 5
    assert used == 774_144
    assert used <= l3_ram_bytes


def test_capture_config_rejects_post_count_that_cannot_leave_a_pre_frame():
    firmware = FIRMWARE.read_text(encoding="utf-8")
    capture_cfg = _function_source(
        firmware,
        "static int32_t l3_cli_captureCfg",
        "#endif",
    )
    planner = _function_source(
        firmware,
        "static int32_t l3_finalizeCapturePlan",
        "/* captureCfg",
    )

    assert "values[5] >= L3_MAX_CAPTURE_FRAMES" in capture_cfg
    assert "gCapturePlan.postFrames >= L3_MAX_CAPTURE_FRAMES" in planner


def test_post_trigger_tail_leaves_room_for_approach_history():
    """Pre-trigger frames are what club path spends, so pin the split.

    The freeze is requested by a UART CLI command, so the trigger frame lands
    late by a variable 2-4 frames (measured 2026-07-25: impact at slot 1.7-4.1
    of an 18-frame ring against an assumed slot 6). The pre-trigger allocation
    has to absorb that and still leave more than CLUB_MIN_FRAMES behind.
    """
    from openflight.iwr6843.club import CLUB_MIN_FRAMES

    lines = CONFIGURABLE_CONFIG.read_text(encoding="utf-8")
    capture = re.search(r"^captureCfg \d+ (\d+) \d+ (\d+) \d+ (\d+)$", lines, re.MULTILINE)
    frame = re.search(r"^frameCfg \d+ \d+ (\d+) ", lines, re.MULTILINE)
    assert capture is not None and frame is not None
    pre_bins, post_bins, post = map(int, capture.groups())
    loops = int(frame.group(1))
    bytes_per_bin = 3 * loops * 4 * 4
    pre_trigger = (768 * 1024 - post * post_bins * bytes_per_bin) // (pre_bins * bytes_per_bin)

    assert post == 16
    assert pre_trigger == 16
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
    assert "uname -s" in check and "uname -m" in check, "the host must be verified"
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

    assert "DOCKER_IMAGE ?= openflight-iwr-sdk:latest" in source
    for target in ("docker-image:", "docker-build:", "docker-shell:"):
        assert f"\n{target}" in source, f"{target} is missing"
    # Three docker invocations, each explicitly amd64: the Dockerfile
    # deliberately does not pin the platform itself.
    assert source.count("--platform linux/amd64") == 3, source.count("--platform linux/amd64")


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
        assert name not in fetchable, f"{name} is login-gated; a scripted GET returns an HTML page"

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
