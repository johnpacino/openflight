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


def test_parser_exposes_independent_attack_and_path_windows():
    parser = ablation._parser()
    args = parser.parse_args(
        [
            "--session",
            "session.jsonl",
            "--tee-m",
            "1.524",
            "--attack-pre-frames",
            "3",
            "--path-pre-frames",
            "4",
            "--path-post-frames",
            "1",
            "--path-post-speed-scale",
            "0.9",
        ]
    )

    assert args.attack_pre_frames == 3
    assert args.attack_post_frames == 0
    assert args.path_pre_frames == 4
    assert args.path_post_frames == 1
    assert args.path_post_speed_scale == pytest.approx(0.9)


@pytest.mark.parametrize("keep_loops", (0, 13))
def test_loop_variants_reject_invalid_count(keep_loops):
    with pytest.raises(ValueError, match="keep-loops"):
        ablation.loop_variants(12, keep_loops)
