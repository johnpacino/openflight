#!/usr/bin/env python3
"""Render the truth-backed figures used by the July IWR6843 field report."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
RESULTS = HERE / "lcmf_v1_block_a_shots.csv"
CACHE = HERE / "cache_tm0714.npz"
OUTPUT = REPO / "docs" / "assets"
CORRECTION_DEG = 2.8387689101987568

INK = "#17332d"
MUTED = "#6d7c77"
LINE = "#d9dedb"
ORANGE = "#d96c2f"
TEAL = "#0f766e"
PALE = "#edf3f0"


def _rows() -> list[dict[str, str]]:
    with RESULTS.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _balanced_indices(cache: np.lib.npyio.NpzFile, shot: int) -> np.ndarray:
    indices = np.nonzero((cache["shot"] == shot) & (cache["snr"] >= 8.0) & (cache["r"] <= 4.7))[0]
    selected: list[int] = []
    for frame in np.unique(cache["frame"][indices]):
        frame_indices = indices[cache["frame"][indices] == frame]
        order = np.argsort(cache["snr"][frame_indices])[::-1]
        selected.extend(frame_indices[order[:4]])
    return np.asarray(selected, dtype=int)


def _representative(rows: list[dict[str, str]], club: str) -> dict[str, str]:
    accepted = [row for row in rows if row["club"] == club and row["status"] == "accepted"]
    accepted.sort(
        key=lambda row: abs(float(row["lcmf_v1_angle_deg"]) - float(row["trackman_launch_deg"]))
    )
    return accepted[len(accepted) // 2]


def render_frame_selection(rows: list[dict[str, str]], cache: np.lib.npyio.NpzFile) -> None:
    examples = [_representative(rows, club) for club in ("Driver", "7Iron", "9Iron")]
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.5), sharey=True)
    fig.patch.set_facecolor("white")
    for axis, row in zip(axes, examples, strict=True):
        shot = int(row["shot"])
        all_indices = np.nonzero(cache["shot"] == shot)[0]
        selected = _balanced_indices(cache, shot)
        chronological = selected[np.argsort(cache["t"][selected])]
        late = chronological[len(chronological) // 2 :]

        axis.set_facecolor("white")
        axis.grid(axis="both", color=LINE, linewidth=0.7, alpha=0.75)
        axis.scatter(
            cache["t"][all_indices] * 1000,
            cache["r"][all_indices],
            s=9,
            color=MUTED,
            alpha=0.20,
            linewidths=0,
            label="tracked snapshots",
            zorder=1,
        )
        axis.scatter(
            cache["t"][selected] * 1000,
            cache["r"][selected],
            s=34,
            facecolors="white",
            edgecolors=ORANGE,
            linewidths=1.3,
            label="strongest four / frame",
            zorder=3,
        )
        axis.scatter(
            cache["t"][late] * 1000,
            cache["r"][late],
            s=24,
            color=TEAL,
            linewidths=0,
            label="late half",
            zorder=4,
        )

        for frame in np.unique(cache["frame"][selected]):
            frame_indices = selected[cache["frame"][selected] == frame]
            axis.text(
                float(np.mean(cache["t"][frame_indices]) * 1000),
                float(np.max(cache["r"][frame_indices]) + 0.08),
                f"F{int(frame)}",
                ha="center",
                va="bottom",
                fontsize=7,
                color=MUTED,
            )

        club_name = row["club"].replace("Iron", "-iron").lower().replace("driver", "Driver")
        axis.set_title(
            f"{club_name}\nTrackMan {float(row['trackman_launch_deg']):.1f}°  ·  "
            f"LCMF {float(row['lcmf_v1_angle_deg']):.1f}°",
            color=INK,
            fontsize=11,
            fontweight="bold",
            pad=12,
        )
        axis.set_xlabel("milliseconds into captured flight", color=MUTED)
        axis.tick_params(colors=MUTED, labelsize=8)
        for spine in axis.spines.values():
            spine.set_color(LINE)
    axes[0].set_ylabel("range from radar (m)", color=MUTED)
    handles, labels = axes[-1].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="lower center",
        ncol=3,
        frameon=False,
        bbox_to_anchor=(0.5, -0.01),
        fontsize=9,
    )
    fig.suptitle(
        "Real TrackMan shots: which radar snapshots LCMF-v1 keeps",
        color=INK,
        fontsize=15,
        fontweight="bold",
        y=1.03,
    )
    fig.tight_layout(rect=(0, 0.08, 1, 0.97))
    fig.savefig(OUTPUT / "iwr6843-lcmf-frame-selection.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def render_accuracy(rows: list[dict[str, str]]) -> None:
    accepted = [row for row in rows if row["status"] == "accepted"]
    club_order = ["Driver", "5Iron", "7Iron", "9Iron", "SandWedge"]
    colors = {
        "Driver": "#17332d",
        "5Iron": "#287271",
        "7Iron": "#68a691",
        "9Iron": "#d96c2f",
        "SandWedge": "#e6a15f",
    }
    truth = np.asarray([float(row["trackman_launch_deg"]) for row in accepted])
    corrected = np.asarray([float(row["lcmf_v1_angle_deg"]) for row in accepted])
    raw = corrected - CORRECTION_DEG
    corrected_error = corrected - truth
    raw_error = raw - truth

    fig, (scatter, residual) = plt.subplots(
        1,
        2,
        figsize=(12.8, 5.1),
        gridspec_kw={"width_ratios": [1.08, 0.92]},
    )
    fig.patch.set_facecolor("white")
    for axis in (scatter, residual):
        axis.set_facecolor("white")
        axis.grid(color=LINE, linewidth=0.7, alpha=0.8)
        axis.tick_params(colors=MUTED, labelsize=8)
        for spine in axis.spines.values():
            spine.set_color(LINE)

    limit_min = min(truth.min(), corrected.min()) - 2
    limit_max = max(truth.max(), corrected.max()) + 2
    scatter.fill_between(
        [limit_min, limit_max],
        [limit_min - 1, limit_max - 1],
        [limit_min + 1, limit_max + 1],
        color=TEAL,
        alpha=0.08,
        label="within 1°",
    )
    scatter.plot([limit_min, limit_max], [limit_min, limit_max], color=INK, linewidth=1.2)
    for club in club_order:
        club_rows = [row for row in accepted if row["club"] == club]
        scatter.scatter(
            [float(row["trackman_launch_deg"]) for row in club_rows],
            [float(row["lcmf_v1_angle_deg"]) for row in club_rows],
            s=32,
            color=colors[club],
            label=club.replace("Iron", "-iron").replace("SandWedge", "sand wedge"),
            alpha=0.9,
            edgecolors="white",
            linewidths=0.5,
        )
    scatter.set_xlim(limit_min, limit_max)
    scatter.set_ylim(limit_min, limit_max)
    scatter.set_aspect("equal", adjustable="box")
    scatter.set_xlabel("TrackMan launch angle (°)", color=MUTED)
    scatter.set_ylabel("LCMF-v1 launch angle (°)", color=MUTED)
    scatter.set_title("53 measured shots", color=INK, fontsize=12, fontweight="bold")
    scatter.legend(frameon=False, fontsize=7, ncol=2, loc="upper left")

    x_raw = np.zeros(len(raw_error))
    x_corrected = np.ones(len(corrected_error))
    for before, after in zip(raw_error, corrected_error, strict=True):
        residual.plot([0, 1], [before, after], color=MUTED, alpha=0.17, linewidth=0.8)
    residual.scatter(x_raw, raw_error, color=MUTED, alpha=0.55, s=17, zorder=3)
    residual.scatter(x_corrected, corrected_error, color=ORANGE, alpha=0.75, s=17, zorder=3)
    residual.axhline(0, color=INK, linewidth=1.1)
    residual.scatter([0], [raw_error.mean()], color=INK, marker="D", s=55, zorder=5)
    residual.scatter([1], [corrected_error.mean()], color=INK, marker="D", s=55, zorder=5)
    residual.text(
        0,
        raw_error.mean() - 0.8,
        f"mean {raw_error.mean():.3f}°",
        ha="center",
        va="top",
        fontsize=9,
        color=INK,
    )
    residual.text(
        1,
        corrected_error.mean() + 0.8,
        f"mean {corrected_error.mean():.3f}°",
        ha="center",
        va="bottom",
        fontsize=9,
        color=INK,
    )
    residual.set_xlim(-0.35, 1.35)
    residual.set_xticks([0, 1], ["raw LCMF", "+2.8388° correction"])
    residual.set_ylabel("estimate minus TrackMan (°)", color=MUTED)
    residual.set_title(
        "One constant removes the shared bias", color=INK, fontsize=12, fontweight="bold"
    )

    errors = np.abs(corrected_error)
    fig.suptitle(
        f"Block A development result · MAE {errors.mean():.2f}° · "
        f"p50 {np.percentile(errors, 50):.2f}° · 53/60 reads",
        color=INK,
        fontsize=15,
        fontweight="bold",
        y=1.02,
    )
    fig.tight_layout()
    fig.savefig(OUTPUT / "iwr6843-lcmf-trackman-accuracy.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    rows = _rows()
    cache = np.load(CACHE)
    render_frame_selection(rows, cache)
    render_accuracy(rows)
    print(f"wrote report figures to {OUTPUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
