"""Confirm the TDM-Doppler-residual hypothesis with truth velocity.

Recompute each snapshot's TX-block phase correction using the TRUE
instantaneous radial velocity (TM trajectory) instead of the track-average
speed, then re-measure the residual curves. If the block-antisymmetric
structure collapses, the mechanism is confirmed.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import tm0714
from openflight.iwr6843.doa import TDM_TAU_S
from openflight.iwr6843.music import LAM, steer

HERE = Path(__file__).parent
C = np.load(HERE / "cache_tm0714.npz")
TABLE = {r["num"]: r for r in
         json.loads((HERE / "cache_tm0714.json").read_text())}
CAL = tm0714.load_cal()
SNR_MIN, EXPL_MIN = 8.0, 0.40
BINS = np.arange(-26.0, 12.1, 2.0)
CENTERS = 0.5 * (BINS[:-1] + BINS[1:])


def corrected_vectors() -> np.ndarray:
    """Stored vectors with the truth-velocity TDM correction applied.

    Stored layout is post-flip: elements 0-3 = TX2 block (already rotated by
    the track-average phase), 4-7 = TX0. The delta rotation uses
    (v_true(x) - v_track_avg).
    """
    shots = {s.num: s for s in tm0714.load_shots()}
    vec = C["vec"].copy()
    for num, rec in TABLE.items():
        if not rec.get("guided_ms"):
            continue
        m = C["shot"] == num
        if not m.any():
            continue
        s = shots[num]
        vr = np.array([s.vr_at(x) for x in C["x"][m]])
        delta = (rec["tdm_sign"] * 4 * np.pi
                 * (vr - rec["guided_ms"]) * TDM_TAU_S / LAM)
        vec[np.nonzero(m)[0], :4] *= np.exp(-1j * delta)[:, None]
    return vec


def residual_curves(vec_raw: np.ndarray):
    """Two-source truth fit -> per-snapshot block-split phase + ramp."""
    vec = vec_raw * CAL.elem_correction[None, :]
    n = len(vec)
    split = np.full(n, np.nan)
    ramp = np.full(n, np.nan)
    expl = np.full(n, np.nan)
    keep = C["snr"] >= SNR_MIN
    m_idx = np.arange(8)
    for i in np.nonzero(keep)[0]:
        s = np.column_stack([steer(C["th_d"][i], 8),
                             steer(C["th_i"][i], 8)])
        coef, *_ = np.linalg.lstsq(s, vec[i], rcond=None)
        pred = s @ coef
        p = float(np.vdot(vec[i], vec[i]).real)
        e = 1.0 - float(np.vdot(vec[i] - pred, vec[i] - pred).real) / p
        expl[i] = e
        if e < EXPL_MIN:
            continue
        good = np.abs(pred) > 0.15 * np.abs(pred).max()
        c = np.where(good, vec[i] / np.where(good, pred, 1.0), 0.0)
        w = np.where(good, np.abs(pred) ** 2, 0.0)
        za = np.sum(w[:4] * c[:4])
        zb = np.sum(w[4:] * c[4:])
        if abs(za) > 0 and abs(zb) > 0:
            split[i] = np.angle(za * np.conj(zb))
        ph = np.angle(np.where(good, c, 1.0))
        ok = good & (w > 0)
        if ok.sum() >= 5:
            mw = m_idx[ok] - np.average(m_idx[ok], weights=w[ok])
            ramp[i] = float(np.sum(w[ok] * mw * ph[ok])
                            / np.sum(w[ok] * mw ** 2))
    return split, ramp, expl


def binned(vals: np.ndarray, mask: np.ndarray) -> np.ndarray:
    th = np.degrees(C["th_d"])
    out = []
    for lo, hi, cen in zip(BINS[:-1], BINS[1:], CENTERS):
        m = mask & (th >= lo) & (th < hi) & np.isfinite(vals)
        if m.sum() >= 25:
            out.append((cen, float(np.median(vals[m]))))
    return np.array(out) if out else np.empty((0, 2))


def main() -> None:
    blocks = np.array([TABLE[int(s)]["block"] for s in C["shot"]])
    results = {}
    for tag, vec in (("track-avg (current)", C["vec"]),
                     ("truth-velocity TDM", corrected_vectors())):
        split, ramp, expl = residual_curves(vec)
        used = np.isfinite(expl) & (expl >= EXPL_MIN)
        results[tag] = (split, ramp, expl, used)
        print(f"{tag}: median explained "
              f"A={np.nanmedian(expl[(C['snr'] >= SNR_MIN) & (blocks == 'A')]):.3f} "
              f"C={np.nanmedian(expl[(C['snr'] >= SNR_MIN) & (blocks == 'C')]):.3f}")

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    for ax, (tag, (split, ramp, expl, used)) in zip(axes, results.items()):
        for blk, col in (("A", "tab:blue"), ("C", "tab:green")):
            cs = binned(np.degrees(split), used & (blocks == blk))
            if len(cs):
                ax.plot(cs[:, 0], cs[:, 1], "-o", ms=3, color=col,
                        label=f"blk {blk} split")
            cr = binned(np.degrees(ramp) * 60, used & (blocks == blk))
            if len(cr):
                ax.plot(cr[:, 0], cr[:, 1], "--s", ms=3, color=col,
                        alpha=0.6, label=f"blk {blk} ramp x60")
        ax.axhline(0, color="k", lw=0.5)
        ax.set_title(tag)
        ax.set_xlabel("true direct angle (deg)")
        ax.set_ylim(-45, 45)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    axes[0].set_ylabel("TX-block split phase (deg)")
    fig.suptitle("TDM Doppler residual: TX-block phase split, before/after "
                 "truth-velocity correction")
    fig.tight_layout()
    fig.savefig(HERE / "tdm_split_test.png", dpi=110)
    print("wrote tdm_split_test.png")

    for tag, (split, ramp, expl, used) in results.items():
        for blk in "AC":
            cs = binned(np.degrees(split), used & (blocks == blk))
            if len(cs):
                print(f"  {tag:22s} blk {blk}: split rms "
                      f"{np.sqrt((cs[:, 1] ** 2).mean()):6.2f} deg  "
                      f"range [{cs[:, 1].min():+.1f}, {cs[:, 1].max():+.1f}]")


if __name__ == "__main__":
    main()
