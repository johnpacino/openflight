"""Causal test: does windowing the range FFT collapse the phase corruption?

Same truth-referenced residual measurement on both caches; compare explained
fraction, TX-block split curve, and implied angle-error ramp, late-track.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import tm0714
from openflight.iwr6843.music import steer

HERE = Path(__file__).parent
SNR_MIN, EXPL_MIN = 8.0, 0.40
BINS = np.arange(-26.0, 12.1, 2.0)


def measure(cache_name: str) -> None:
    c = np.load(HERE / f"{cache_name}.npz")
    table = {r["num"]: r for r in
             json.loads((HERE / f"{cache_name}.json").read_text())}
    cal = tm0714.load_cal()
    vec = c["vec"] * cal.elem_correction[None, :]
    blocks = np.array([table[int(s)]["block"] for s in c["shot"]])
    n = len(vec)
    split = np.full(n, np.nan)
    ramp = np.full(n, np.nan)
    expl = np.full(n, np.nan)
    keep = c["snr"] >= SNR_MIN
    m_idx = np.arange(8)
    for i in np.nonzero(keep)[0]:
        b = np.column_stack([steer(c["th_d"][i], 8),
                             steer(c["th_i"][i], 8)])
        coef, *_ = np.linalg.lstsq(b, vec[i], rcond=None)
        pred = b @ coef
        p = float(np.vdot(vec[i], vec[i]).real)
        expl[i] = 1.0 - float(np.vdot(vec[i] - pred,
                                      vec[i] - pred).real) / p
        if expl[i] < EXPL_MIN:
            continue
        good = np.abs(pred) > 0.15 * np.abs(pred).max()
        cr = np.where(good, vec[i] / np.where(good, pred, 1.0), 0.0)
        w = np.where(good, np.abs(pred) ** 2, 0.0)
        za, zb = np.sum(w[:4] * cr[:4]), np.sum(w[4:] * cr[4:])
        if abs(za) > 0 and abs(zb) > 0:
            split[i] = np.angle(za * np.conj(zb))
        ph = np.angle(np.where(good, cr, 1.0))
        ok = good & (w > 0)
        if ok.sum() >= 5:
            mw = m_idx[ok] - np.average(m_idx[ok], weights=w[ok])
            ramp[i] = float(np.sum(w[ok] * mw * ph[ok])
                            / np.sum(w[ok] * mw ** 2))

    th = np.degrees(c["th_d"])
    used = np.isfinite(expl) & (expl >= EXPL_MIN)
    print(f"\n=== {cache_name} ===")
    for blk in "AC":
        g = keep & (blocks == blk)
        u = used & (blocks == blk)
        late = u & (th > -4.0)
        early = u & (th <= -4.0)
        print(f" block {blk}: gated {g.sum():5d}  expl med "
              f"{np.nanmedian(expl[g]):.3f}  pass {100*u.sum()/max(g.sum(),1):3.0f}%")
        for tag, m in (("early", early), ("late ", late)):
            if m.sum() < 30:
                continue
            s_med = np.degrees(np.nanmedian(split[m]))
            s_sd = np.degrees(np.nanstd(split[m]))
            r_deg = np.degrees(np.nanmedian(ramp[m])) * 60
            print(f"   {tag}: n={m.sum():5d}  split med {s_med:+6.1f} "
                  f"sd {s_sd:5.1f} deg   ramp-implied angle err "
                  f"{r_deg/60/np.pi*180/np.cos(np.radians(5)):+5.2f} deg")
    # binned split curve block A for shape comparison
    cur = []
    for lo, hi in zip(BINS[:-1], BINS[1:]):
        m = used & (blocks == "A") & (th >= lo) & (th < hi)
        if m.sum() >= 25:
            cur.append((0.5 * (lo + hi),
                        float(np.degrees(np.median(split[m])))))
    print("  blk A split curve:",
          "  ".join(f"{c0:+.0f}:{v:+.0f}" for c0, v in cur))


if __name__ == "__main__":
    measure("cache_tm0714")
    measure("cache_tm0714_blackman")
