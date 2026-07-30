"""Offline 12-versus-10 loop club-path ablation helpers."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts" / "iwr6843"))

import club_path_loop_ablation as ablation  # noqa: E402


def test_loop_variants_cover_first_center_and_last_ten():
    assert ablation.loop_variants(12, 10) == [
        ("full", 0, 12),
        ("first", 0, 10),
        ("center", 1, 10),
        ("last", 2, 10),
    ]


def test_loop_variants_do_not_duplicate_full_capture():
    assert ablation.loop_variants(10, 10) == [("full", 0, 10)]


@pytest.mark.parametrize("keep_loops", (0, 13))
def test_loop_variants_reject_invalid_count(keep_loops):
    with pytest.raises(ValueError, match="keep-loops"):
        ablation.loop_variants(12, keep_loops)
