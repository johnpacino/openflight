"""Is the residual corruption carried by the floor-image path?

Test 1: bin residual split by IMAGE angle -- if A and C align there (they do
not align in direct-angle coordinates), the corruption is image-borne.
Test 2: split magnitude vs image power fraction quartiles.
Test 3: explained fraction vs image fraction (does the two-source model
degrade when the image is strong?).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import tm0714
from openflight.iwr6843.music import steer

HERE = Path(__file__).parent
C = np.load(HERE / "cache_tm0714.npz")
TABLE = {r["num"]: r for r in
         json.loads((HERE / "cache_tm0714.json").read_text())}
CAL = tm0714.load_cal()
SNR_MIN, EXPL_MIN = 8.0, 0.40


def main() -> None:
    vec_all = C["vec"] * CAL.elem_correction[None, :]
    n = len(vec_all)
    split = np.full(n, np.nan)
    imfrac = np.full(n, np.nan)
    expl = np.full(n, np.nan)
    keep = C["snr"] >= SNR_MIN
    for i in np.nonzero(keep)[0]:
        b = np.column_stack([steer(C["th_d"][i], 8),
                             steer(C["th_i"][i], 8)])
        coef, *_ = np.linalg.lstsq(b, vec_all[i], rcond=None)
        pred = b @ coef
        p = float(np.vdot(vec_all[i], vec_all[i]).real)
        expl[i] = 1.0 - float(np.vdot(vec_all[i] - pred,
                                      vec_all[i] - pred).real) / p
        pa, pb = abs(coef[0]) ** 2, abs(coef[1]) ** 2
        imfrac[i] = pb / (pa + pb + 1e-12)
        if expl[i] < EXPL_MIN:
            continue
        good = np.abs(pred) > 0.15 * np.abs(pred).max()
        c = np.where(good, vec_all[i] / np.where(good, pred, 1.0), 0.0)
        w = np.where(good, np.abs(pred) ** 2, 0.0)
        za, zb = np.sum(w[:4] * c[:4]), np.sum(w[4:] * c[4:])
        if abs(za) > 0 and abs(zb) > 0:
            split[i] = np.angle(za * np.conj(zb))

    blocks = np.array([TABLE[int(s)]["block"] for s in C["shot"]])
    fin = np.isfinite(split)
    th_i_deg = np.degrees(C["th_i"])
    th_d_deg = np.degrees(C["th_d"])

    print("TEST 1 -- split binned by IMAGE angle (deg):")
    ibins = np.arange(-34, -8, 3.0)
    hdr = "  ".join(f"{0.5*(a+b):6.0f}" for a, b in zip(ibins[:-1],
                                                        ibins[1:]))
    print(f"  bin centers:      {hdr}")
    for blk in "AC":
        cur = []
        for lo, hi in zip(ibins[:-1], ibins[1:]):
            m = fin & (blocks == blk) & (th_i_deg >= lo) & (th_i_deg < hi)
            cur.append(f"{np.degrees(np.median(split[m])):6.1f}"
                       if m.sum() >= 25 else "     .")
        print(f"  block {blk} split:   {'  '.join(cur)}")
    print("\n  same, binned by DIRECT angle (reference for non-alignment):")
    dbins = np.arange(-22, 8, 3.0)
    hdr = "  ".join(f"{0.5*(a+b):6.0f}" for a, b in zip(dbins[:-1],
                                                        dbins[1:]))
    print(f"  bin centers:      {hdr}")
    for blk in "AC":
        cur = []
        for lo, hi in zip(dbins[:-1], dbins[1:]):
            m = fin & (blocks == blk) & (th_d_deg >= lo) & (th_d_deg < hi)
            cur.append(f"{np.degrees(np.median(split[m])):6.1f}"
                       if m.sum() >= 25 else "     .")
        print(f"  block {blk} split:   {'  '.join(cur)}")

    print("\nTEST 2 -- |split| and TEST 3 -- explained, by image-fraction "
          "quartile (block A):")
    m = fin & (blocks == "A")
    qs = np.nanquantile(imfrac[m], [0, 0.25, 0.5, 0.75, 1.0])
    for lo, hi in zip(qs[:-1], qs[1:]):
        q = m & (imfrac >= lo) & (imfrac <= hi)
        print(f"  imfrac {lo:.2f}-{hi:.2f}: n={q.sum():5d}  "
              f"|split| med {np.degrees(np.nanmedian(np.abs(split[q]))):5.1f} "
              f"deg   split sd {np.degrees(np.nanstd(split[q])):5.1f} deg   "
              f"expl med {np.nanmedian(expl[q]):.3f}")
    # and the gated-but-failed population: where does low explained live?
    print("\n  explained vs image fraction (ALL gated, block A):")
    ga = keep & (blocks == "A")
    for lo, hi in ((0, .1), (.1, .25), (.25, .45), (.45, .7), (.7, 1.01)):
        q = ga & (imfrac >= lo) & (imfrac < hi)
        if q.sum() > 30:
            print(f"  imfrac {lo:.2f}-{hi:.2f}: n={q.sum():5d}  "
                  f"expl med {np.nanmedian(expl[q]):.3f}  "
                  f"expl<0.4: {100*np.mean(expl[q] < 0.4):4.0f}%")


if __name__ == "__main__":
    main()
