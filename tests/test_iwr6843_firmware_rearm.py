"""Regression checks for the HWA snapshot ring freeze/rearm sequence."""

from pathlib import Path


FIRMWARE = Path(__file__).parents[1] / "firmware" / "l3_dump" / "l3_dump.c"


def _function_source(source: str, name: str, next_name: str) -> str:
    start = source.rindex(name)
    end = source.index(next_name, start)
    return source[start:end]


def test_hwa_dump_freezes_only_at_a_completed_ring_boundary():
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    freeze = dump.index("l3_freezeHwaAtRingBoundary")
    stop = dump.index("MMWave_stop")

    assert freeze < stop


def test_dump_reuses_completed_chain_instead_of_full_hwa_reconfiguration():
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    restart = dump.index("l3_restartCompletedHwaRing")
    non_hwa_branch = dump.index("#else", restart)
    full_arm = dump.index("l3_armCapture", restart)

    assert restart < non_hwa_branch < full_arm


def test_freeze_request_suppresses_automatic_ring_rearm():
    source = FIRMWARE.read_text(encoding="utf-8")
    queue = _function_source(source, "static void l3_hwaMaybeQueueRearm", "static void l3_hwaChainDoneCB")

    assert "gHwaFreezeRequested" in queue
    assert "Semaphore_post(gHwaFreezeSemaphore)" in queue
