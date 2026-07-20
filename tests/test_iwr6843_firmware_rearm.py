"""Regression checks for the HWA snapshot ring freeze/rearm sequence."""

from pathlib import Path


FIRMWARE = Path(__file__).parents[1] / "firmware" / "l3_dump" / "l3_dump.c"


def _function_source(source: str, name: str, next_name: str) -> str:
    start = source.rindex(name)
    end = source.index(next_name, start)
    return source[start:end]


def test_hwa_dump_waits_for_rearm_worker_before_stopping_rf():
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    disable = dump.index("gCaptureActive = 0U")
    wait = dump.index("l3_waitForHwaRearmIdle")
    stop = dump.index("MMWave_stop")

    assert disable < wait < stop


def test_full_hwa_rearm_clears_stale_state_before_reconfiguration():
    source = FIRMWARE.read_text(encoding="utf-8")
    arm = _function_source(source, "static int32_t l3_armHwaChain", "static void l3_hwaRearmTask")

    reset = arm.index("HWA_reset")
    first_param = arm.index("HWA_configParamSet")

    assert reset < first_param
    assert "l3_drainHwaRearmSemaphore" in arm
