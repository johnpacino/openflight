"""What drives the TX-block split phase? Regress against candidate causes.

Candidates per snapshot (truth-derived): sin(azimuth), residual radial
velocity (track-avg minus instantaneous), horizontal distance x, ball height
z. If sin(az) wins with a clean slope, the TX blocks are offset in azimuth
and the split is a geometric phase the reflector cal (az=0) could not see.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

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


def main() -> None:
    shots = {s.num: s for s in tm0714.load_shots()}
    vec_all = C["vec"] * CAL.elem_correction[None, :]
    n = len(vec_all)
    split = np.full(n, np.nan)
    az = np.full(n, np.nan)
    dvr = np.full(n, np.nan)
    keep = C["snr"] >= SNR_MIN
    for num, rec in TABLE.items():
        if not rec.get("guided_ms"):
            continue
        s = shots[num]
        m = (C["shot"] == num) & keep
        for i in np.nonzero(m)[0]:
            x = C["x"][i]
            az[i] = math.atan2(s.y_at(x), x)
            dvr[i] = s.vr_at(x) - rec["guided_ms"]
            b = np.column_stack([steer(C["th_d"][i], 8),
                                 steer(C["th_i"][i], 8)])
            coef, *_ = np.linalg.lstsq(b, vec_all[i], rcond=None)
            pred = b @ coef
            p = float(np.vdot(vec_all[i], vec_all[i]).real)
            e = 1.0 - float(np.vdot(vec_all[i] - pred,
                                    vec_all[i] - pred).real) / p
            if e < EXPL_MIN:
                continue
            good = np.abs(pred) > 0.15 * np.abs(pred).max()
            c = np.where(good, vec_all[i] / np.where(good, pred, 1.0), 0.0)
            w = np.where(good, np.abs(pred) ** 2, 0.0)
            za = np.sum(w[:4] * c[:4])
            zb = np.sum(w[4:] * c[4:])
            if abs(za) > 0 and abs(zb) > 0:
                split[i] = np.angle(za * np.conj(zb))

    blocks = np.array([TABLE[int(s)]["block"] for s in C["shot"]])
    fin = np.isfinite(split)
    print(f"snapshots with split measured: {fin.sum()}")

    # candidate features
    feats = {
        "pi*sin(az)": np.pi * np.sin(az),
        "tdm dvr (rad)": 4 * np.pi * dvr * TDM_TAU_S / LAM,
        "x (m)": C["x"],
        "z (m)": C["z"],
        "th_d (rad)": C["th_d"],
    }
    for blk in ("A", "C", "ALL"):
        m = fin & ((blocks == blk) if blk != "ALL" else True)
        if m.sum() < 100:
            continue
        y = split[m]
        print(f"\nblock {blk} (n={m.sum()}), split sd "
              f"{np.degrees(y.std()):.1f} deg:")
        for name, f in feats.items():
            xv = f[m]
            ok = np.isfinite(xv)
            r = np.corrcoef(xv[ok], y[ok])[0, 1]
            slope = np.polyfit(xv[ok], y[ok], 1)[0]
            print(f"  {name:14s} r={r:+5.2f}  slope={slope:+7.3f} "
                  f"(rad/rad or rad/m)")
        # joint: sin(az) + dvr
        good2 = np.isfinite(feats["pi*sin(az)"][m])
        design = np.column_stack([feats["pi*sin(az)"][m][good2],
                                  feats["tdm dvr (rad)"][m][good2],
                                  np.ones(good2.sum())])
        beta, *_ = np.linalg.lstsq(design, y[good2], rcond=None)
        resid = y[good2] - design @ beta
        print(f"  joint az+dvr: coef_az={beta[0]:+.3f} coef_dvr={beta[1]:+.3f}"
              f" const={np.degrees(beta[2]):+.1f}deg"
              f"  resid sd {np.degrees(resid.std()):.1f} deg"
              f" (was {np.degrees(y[good2].std()):.1f})")

    # per-shot: mean split late-track vs TM LaunchDirection
    print("\nper-shot late-track split vs LaunchDirection:")
    pairs = []
    for num, rec in sorted(TABLE.items()):
        if not rec.get("guided_ms"):
            continue
        m = (C["shot"] == num) & fin & (C["x"] > 3.2)
        if m.sum() < 10:
            continue
        pairs.append((shots[num].tm_dir_deg,
                      float(np.degrees(np.median(split[m]))), rec["block"]))
    arr = np.array([(d, s) for d, s, _b in pairs])
    if len(arr) > 10:
        r = np.corrcoef(arr[:, 0], arr[:, 1])[0, 1]
        print(f"  n={len(arr)} shots, corr(direction, split) = {r:+.2f}")
        for blk in "ABC":
            sub = np.array([(d, s) for d, s, b in pairs if b == blk])
            if len(sub) > 5:
                r = np.corrcoef(sub[:, 0], sub[:, 1])[0, 1]
                print(f"    block {blk}: n={len(sub)} r={r:+.2f} "
                      f"slope={np.polyfit(sub[:, 0], sub[:, 1], 1)[0]:+.2f} "
                      f"deg-split per deg-direction")


if __name__ == "__main__":
    main()
