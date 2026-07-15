"""Does a 3rd coherent arrival explain the mid-mix residual?

On mid-mix block-A snapshots (image fraction 0.15-0.7): compare 2-source
explained vs 2-source + one free-angle 3rd source. Histogram the best 3rd
angle -- a physical extra path spikes somewhere consistent (e.g. the mat-top
image); incoherent junk gives a flat histogram and modest gains.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

import tm0714
from openflight.iwr6843.music import steer

HERE = Path(__file__).parent
C = np.load(HERE / "cache_tm0714.npz")
TABLE = {r["num"]: r for r in
         json.loads((HERE / "cache_tm0714.json").read_text())}
CAL = tm0714.load_cal()
GRID3 = np.radians(np.arange(-38.0, 38.01, 1.0))
STEER3 = np.exp(1j * np.pi * np.sin(GRID3)[None, :] * np.arange(8)[:, None])


def main() -> None:
    rng = np.random.default_rng(7)
    vec_all = C["vec"] * CAL.elem_correction[None, :]
    blocks = np.array([TABLE[int(s)]["block"] for s in C["shot"]])
    shots = {s.num: s for s in tm0714.load_shots()}

    # first pass: imfrac for selection
    keep = (C["snr"] >= 8.0) & (blocks == "A")
    idx_all = np.nonzero(keep)[0]
    sel = []
    for i in idx_all:
        b = np.column_stack([steer(C["th_d"][i], 8),
                             steer(C["th_i"][i], 8)])
        coef, *_ = np.linalg.lstsq(b, vec_all[i], rcond=None)
        pa, pb = abs(coef[0]) ** 2, abs(coef[1]) ** 2
        f = pb / (pa + pb + 1e-12)
        if 0.15 <= f <= 0.70:
            sel.append(i)
    rng.shuffle(sel)
    sel = sel[:500]
    print(f"mid-mix sample: {len(sel)} snapshots")

    gains, th3s, dmat = [], [], []
    e2s, e3s = [], []
    for i in sel:
        x = vec_all[i]
        p = float(np.vdot(x, x).real)
        b2 = np.column_stack([steer(C["th_d"][i], 8),
                              steer(C["th_i"][i], 8)])
        coef, *_ = np.linalg.lstsq(b2, x, rcond=None)
        r2 = x - b2 @ coef
        e2 = 1.0 - float(np.vdot(r2, r2).real) / p
        best_e3, best_th = e2, np.nan
        for k, th3 in enumerate(GRID3):
            if (abs(th3 - C["th_d"][i]) < np.radians(4)
                    or abs(th3 - C["th_i"][i]) < np.radians(4)):
                continue
            b3 = np.column_stack([b2, STEER3[:, k]])
            coef3, *_ = np.linalg.lstsq(b3, x, rcond=None)
            r3 = x - b3 @ coef3
            e3 = 1.0 - float(np.vdot(r3, r3).real) / p
            if e3 > best_e3:
                best_e3, best_th = e3, th3
        e2s.append(e2)
        e3s.append(best_e3)
        gains.append(best_e3 - e2)
        th3s.append(math.degrees(best_th) if np.isfinite(best_th) else np.nan)
        # mat-top image prediction for this snapshot
        s = shots[int(C["shot"][i])]
        zm = 0.035
        zim = 2 * zm - C["z"][i]
        th_mat = math.degrees(
            math.atan2(zim - s.rh, C["x"][i]) - math.radians(s.tilt_deg))
        dmat.append(th3s[-1] - th_mat if np.isfinite(best_th) else np.nan)

    gains = np.array(gains)
    th3s = np.array(th3s)
    print(f"explained 2-src median {np.median(e2s):.3f} -> 3-src "
          f"{np.median(e3s):.3f} (gain med {np.median(gains):.3f})")
    print(f"gain > 0.15: {100*np.mean(gains > 0.15):.0f}%   "
          f"gain > 0.30: {100*np.mean(gains > 0.30):.0f}%")
    # note: a 3rd free complex source on 8 elements ALWAYS gains something;
    # random-noise reference below calibrates that
    noise_gain = []
    for _ in range(300):
        x = (rng.standard_normal(8) + 1j * rng.standard_normal(8))
        p = float(np.vdot(x, x).real)
        b2 = np.column_stack([steer(0.1, 8), steer(-0.3, 8)])
        coef, *_ = np.linalg.lstsq(b2, x, rcond=None)
        r2 = x - b2 @ coef
        e2 = 1.0 - float(np.vdot(r2, r2).real) / p
        best = e2
        for k in range(0, len(GRID3), 2):
            b3 = np.column_stack([b2, STEER3[:, k]])
            coef3, *_ = np.linalg.lstsq(b3, x, rcond=None)
            r3 = x - b3 @ coef3
            best = max(best, 1.0 - float(np.vdot(r3, r3).real) / p)
        noise_gain.append(best - e2)
    print(f"pure-noise reference gain: med {np.median(noise_gain):.3f}")

    hist, edges = np.histogram(th3s[np.isfinite(th3s)],
                               bins=np.arange(-40, 41, 4.0))
    print("\nbest 3rd-source angle histogram (4-deg bins):")
    for h, lo in zip(hist, edges[:-1]):
        bar = "#" * int(40 * h / max(hist.max(), 1))
        print(f"  {lo:+5.0f}..{lo+4:+5.0f}: {h:4d} {bar}")
    dmat = np.array(dmat)
    fin = np.isfinite(dmat)
    print(f"\n3rd angle minus MAT-image prediction: med "
          f"{np.median(dmat[fin]):+.1f} deg, within 4 deg: "
          f"{100*np.mean(np.abs(dmat[fin]) < 4):.0f}%")


if __name__ == "__main__":
    main()
