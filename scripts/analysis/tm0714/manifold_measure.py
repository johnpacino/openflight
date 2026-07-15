"""THE calibration diagnosis: per-element residual vs TRUE arrival angle.

For every cached snapshot the TM-truth trajectory fixes both arrival angles
(direct + floor image). Solving only the two complex amplitudes leaves a
per-element residual ratio; if the array manifold (element patterns + radome)
deviates from the ideal ULA model, that residual is a SMOOTH, BLOCK-CONSISTENT
function of angle. If it's just noise, calibration isn't the problem.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import tm0714
from openflight.iwr6843.music import steer

HERE = Path(__file__).parent
C = np.load(HERE / "cache_tm0714.npz")
TABLE = {r["num"]: r for r in
         json.loads((HERE / "cache_tm0714.json").read_text())}
CAL = tm0714.load_cal()

SNR_MIN = 8.0
EXPL_MIN = 0.40


def main() -> None:
    vec = C["vec"] * CAL.elem_correction[None, :]
    shot = C["shot"]
    blocks = np.array([TABLE[int(s)]["block"] for s in shot])
    keep = C["snr"] >= SNR_MIN
    print(f"snapshots: {len(vec)}, snr>={SNR_MIN}: {keep.sum()}")

    n_el = 8
    m_idx = np.arange(n_el)
    res_phase = np.full((len(vec), n_el), np.nan)
    res_gain = np.full((len(vec), n_el), np.nan)
    weight = np.zeros((len(vec), n_el))
    expl = np.full(len(vec), np.nan)
    ramp = np.full(len(vec), np.nan)       # common linear-phase ramp rad/elem

    idx = np.nonzero(keep)[0]
    for i in idx:
        s = np.column_stack([steer(C["th_d"][i], n_el),
                             steer(C["th_i"][i], n_el)])
        coef, *_ = np.linalg.lstsq(s, vec[i], rcond=None)
        pred = s @ coef
        p = float(np.vdot(vec[i], vec[i]).real)
        e = 1.0 - float(np.vdot(vec[i] - pred, vec[i] - pred).real) / p
        expl[i] = e
        if e < EXPL_MIN:
            continue
        good = np.abs(pred) > 0.15 * np.abs(pred).max()
        c = np.where(good, vec[i] / np.where(good, pred, 1.0), np.nan)
        res_phase[i] = np.angle(c)
        res_gain[i] = np.abs(c)
        weight[i] = np.where(good, np.abs(pred) ** 2, 0.0)
        # common ramp: weighted LS of phase vs element index (angle-error tell)
        ph = np.angle(c)
        w = weight[i]
        ok = np.isfinite(ph) & (w > 0)
        if ok.sum() >= 5:
            mw = m_idx[ok] - np.average(m_idx[ok], weights=w[ok])
            ramp[i] = float(np.sum(w[ok] * mw * ph[ok])
                            / np.sum(w[ok] * mw ** 2))

    used = np.isfinite(expl) & (expl >= EXPL_MIN)
    print(f"explained>={EXPL_MIN}: {used.sum()} "
          f"({100*used.sum()/max(keep.sum(),1):.0f}% of gated)")
    for blk in "ABC":
        m = keep & (blocks == blk)
        if m.sum():
            print(f"  block {blk}: median explained "
                  f"{np.nanmedian(expl[m]):.3f}  (n={m.sum()})")

    th_deg = np.degrees(C["th_d"])
    bins = np.arange(-26.0, 12.1, 2.0)
    centers = 0.5 * (bins[:-1] + bins[1:])

    fig, axes = plt.subplots(3, 3, figsize=(15, 10), sharex=True)
    colors = {"A": "tab:blue", "B": "tab:orange", "C": "tab:green"}
    curves = {}
    for el in range(n_el):
        ax = axes.flat[el]
        for blk, col in colors.items():
            sel = used & (blocks == blk)
            ph, w, th = res_phase[sel, el], weight[sel, el], th_deg[sel]
            fin = np.isfinite(ph) & (w > 0)
            ph, w, th = ph[fin], w[fin], th[fin]
            cur = []
            for lo, hi, cen in zip(bins[:-1], bins[1:], centers):
                m = (th >= lo) & (th < hi)
                if w[m].sum() > 0 and m.sum() >= 25:
                    z = np.sum(w[m] * np.exp(1j * ph[m])) / w[m].sum()
                    cur.append((cen, np.degrees(np.angle(z))))
            if cur:
                cs = np.array(cur)
                ax.plot(cs[:, 0], cs[:, 1], "-o", ms=3, color=col,
                        label=f"blk {blk}")
                curves[(el, blk)] = cs.tolist()
        ax.axhline(0, color="k", lw=0.5)
        ax.set_title(f"element {el}", fontsize=9)
        ax.set_ylim(-60, 60)
        ax.grid(alpha=0.3)
    # 9th panel: common ramp -> implied angle error vs angle
    ax = axes.flat[8]
    for blk, col in colors.items():
        sel = used & (blocks == blk) & np.isfinite(ramp)
        th, rp, w = th_deg[sel], ramp[sel], np.nanmax(weight[sel], axis=1)
        cur = []
        for lo, hi, cen in zip(bins[:-1], bins[1:], centers):
            m = (th >= lo) & (th < hi)
            if m.sum() >= 25:
                r_med = float(np.median(rp[m]))
                dth = r_med / (np.pi * np.cos(np.radians(cen)))
                cur.append((cen, np.degrees(dth)))
        if cur:
            cs = np.array(cur)
            ax.plot(cs[:, 0], cs[:, 1], "-o", ms=3, color=col,
                    label=f"blk {blk}")
            curves[("ramp", blk)] = cs.tolist()
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title("implied ANGLE error (deg) from common ramp", fontsize=9)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("true direct angle vs boresight (deg)")
    for ax in axes[:, 0]:
        ax.set_ylabel("residual phase (deg)")
    fig.suptitle("Per-element residual phase vs TRUE angle "
                 "(TM-truth two-source fit; existing cal applied)")
    fig.tight_layout()
    fig.savefig(HERE / "manifold_curves.png", dpi=110)
    Path(HERE / "manifold_curves.json").write_text(
        json.dumps({f"{k[0]}_{k[1]}": v for k, v in curves.items()}))
    print("wrote manifold_curves.png")

    # headline numbers: rms of binned curves, block agreement where overlap
    for el in range(n_el):
        vals = {blk: dict((round(c, 1), v) for c, v in
                          curves.get((el, blk), []))
                for blk in "ABC"}
        rms = {blk: (np.sqrt(np.mean(np.array(list(v.values())) ** 2))
                     if v else np.nan) for blk, v in vals.items()}
        print(f"el{el}: curve rms A={rms['A']:.1f} B={rms['B']:.1f} "
              f"C={rms['C']:.1f} deg")


if __name__ == "__main__":
    main()
