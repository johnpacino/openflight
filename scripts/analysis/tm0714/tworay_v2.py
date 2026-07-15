"""Two-ray v2: ONE trajectory fit jointly over all snapshots of a shot.

v1 solves each snapshot's ball height independently, then line-fits the
heights. v2 parameterizes the trajectory (launch angle through the known
tee point, gravity sag from the measured speed) and scans LA: for each
candidate, every snapshot's direct+image angles are fixed by geometry and
only the two complex amplitudes are solved (closed form). The winning LA
maximizes total explained energy. Snapshots whose individual height
posterior is ambiguous still contribute.

Scored with the same pools + split-half holdout as the shipped v1 policy.
"""
from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import numpy as np

import tm0714
from openflight.iwr6843.music import est_bartlett
from openflight.iwr6843.shot import FAST_BALL_MS, near_mti_notch

HERE = Path(__file__).parent
C = np.load(HERE / "cache_tm0714.npz")
TABLE = {r["num"]: r for r in
         json.loads((HERE / "cache_tm0714.json").read_text())}
CAL = tm0714.load_cal()
G = 9.81
M8 = np.arange(8)
LA_GRID = np.radians(np.arange(-5.0, 45.01, 0.2))
TH_GATE = np.radians(-2.5)


def track_ok(rec) -> bool:
    return (rec.get("guided_ms") is not None and rec["guided_rms"] < 0.50
            and rec["guided_n"] >= 30 and rec["n_snaps"] >= 56)


def v1_quality(vec: np.ndarray, x: float, rh: float, tilt: float
               ) -> tuple[float, float]:
    """(best explained, image fraction) from the v1 per-snapshot scan."""
    grid = np.arange(-0.02, 1.30, 0.02)
    th_d = np.arctan2(grid - rh, x) - tilt
    th_i = np.arctan2(-(grid + rh), x) - tilt
    sd = np.exp(1j * np.pi * np.sin(th_d)[:, None] * M8)
    si = np.exp(1j * np.pi * np.sin(th_i)[:, None] * M8)
    a12 = np.einsum("gm,gm->g", sd.conj(), si)
    b1, b2 = sd.conj() @ vec, si.conj() @ vec
    det = 64.0 - np.abs(a12) ** 2
    det = np.where(np.abs(det) < 1e-9, 1e-9, det)
    ca = (8 * b1 - a12 * b2) / det
    cb = (8 * b2 - np.conj(a12) * b1) / det
    pp = (np.abs(ca) ** 2 * 8 + np.abs(cb) ** 2 * 8
          + 2 * np.real(np.conj(ca) * cb * a12))
    p = float(np.vdot(vec, vec).real) + 1e-12
    k = int(np.argmax(pp.real))
    pa, pb = abs(ca[k]) ** 2, abs(cb[k]) ** 2
    return float(pp.real[k] / p), float(pb / (pa + pb + 1e-12))


def joint_la(vecs: np.ndarray, xs: np.ndarray, w: np.ndarray, rec: dict,
             z0_off: float = 0.0) -> tuple[float, float]:
    """Scan LA for the trajectory that best explains all snapshots.

    Returns (la_deg, sharpness). Energy objective with snapshot weights w.
    """
    rh, tilt = rec["rh"], np.radians(rec["tilt"])
    x_tee, z0 = rec["x_tee"], rec["z0"] + z0_off
    v = rec["guided_ms"] / max(rec.get("cos_factor", 0.96), 0.9)
    d = xs[None, :] - x_tee                                  # (1, S)
    la = LA_GRID[:, None]                                    # (L, 1)
    z = z0 + np.tan(la) * d - G * d ** 2 / (2 * v * v * np.cos(la) ** 2)
    xh = xs[None, :]
    th_d = np.arctan2(z - rh, xh) - tilt                     # (L, S)
    th_i = np.arctan2(-(z + rh), xh) - tilt
    sd = np.exp(1j * np.pi * np.sin(th_d)[..., None] * M8)   # (L, S, 8)
    si = np.exp(1j * np.pi * np.sin(th_i)[..., None] * M8)
    a12 = np.einsum("lsm,lsm->ls", sd.conj(), si)
    b1 = np.einsum("lsm,sm->ls", sd.conj(), vecs)
    b2 = np.einsum("lsm,sm->ls", si.conj(), vecs)
    det = 64.0 - np.abs(a12) ** 2
    det = np.where(np.abs(det) < 1e-9, 1e-9, det)
    ca = (8 * b1 - a12 * b2) / det
    cb = (8 * b2 - np.conj(a12) * b1) / det
    pred = (np.abs(ca) ** 2 * 8 + np.abs(cb) ** 2 * 8
            + 2 * np.real(np.conj(ca) * cb * a12)).real      # (L, S)
    score = (pred * w[None, :]).sum(axis=1)
    k = int(np.argmax(score))
    if 0 < k < len(LA_GRID) - 1:                             # parabolic refine
        y0, y1, y2 = score[k - 1], score[k], score[k + 1]
        den = y0 - 2 * y1 + y2
        off = 0.5 * (y0 - y2) / den if den < 0 else 0.0
        la_best = LA_GRID[k] + np.clip(off, -1, 1) * np.radians(0.2)
    else:
        la_best = LA_GRID[k]
    sharp = float((score[k] - np.median(score)) / (abs(score[k]) + 1e-12))
    return float(np.degrees(la_best)), sharp


def v2_for_shot(num: int, variant: str) -> float | None:
    rec = TABLE[num]
    m = C["shot"] == num
    idx = np.nonzero(m)[0]
    vec = C["vec"][idx] * CAL.elem_correction[None, :]
    snr, rng = C["snr"][idx], C["r"][idx]
    keep = snr >= 8.0
    vec, rng, snr = vec[keep], rng[keep], snr[keep]
    if len(vec) < 6:
        return None
    rh, tilt = rec["rh"], np.radians(rec["tilt"])
    far = (near_mti_notch(rec["guided_ms"])
           or rec["guided_ms"] >= FAST_BALL_MS)
    if far:
        pool = rng <= 3.8
    else:
        bart = np.array([est_bartlett(v) for v in vec])
        pool = bart <= TH_GATE
    if pool.sum() < 6:
        pool = np.ones(len(vec), dtype=bool)
    vecs, xs = vec[pool], rng[pool]
    energy = (np.abs(vecs) ** 2).mean(axis=1)
    w = np.ones(len(vecs))
    if variant == "b":               # quality-weighted (v1 expl + dominance)
        q = np.array([v1_quality(v, float(x), rh, tilt)
                      for v, x in zip(vecs, xs)])
        w = np.maximum(q[:, 0] - 0.5, 0.05)
        if far:
            w *= 0.25 + 0.75 * 2 * np.abs(q[:, 1] - 0.5)
    if variant == "c":               # equal per-snapshot voice
        w = 1.0 / (energy * 8 + 1e-12)
    if variant in ("e", "f"):
        # bounded votes: per-snapshot explained FRACTION (normalize each
        # snapshot's energy out), SNR-capped weight so noise can't dominate
        q = np.array([v1_quality(v, float(x), rh, tilt)
                      for v, x in zip(vecs, xs)])
        w = np.minimum(snr[pool], 25.0) / (energy * 8 + 1e-12)
        if far:
            w *= 0.25 + 0.75 * 2 * np.abs(q[:, 1] - 0.5)
        if variant == "f":           # exactly v1's snapshot set
            good = q[:, 0] >= 0.70
            if good.sum() >= 6:
                vecs, xs, w = vecs[good], xs[good], w[good]
    if variant == "d":               # float the tee height +/- 3 cm
        best = (None, -np.inf)
        for off in (-0.03, -0.015, 0.0, 0.015, 0.03):
            la, sh = joint_la(vecs, xs, w, rec, z0_off=off)
            if sh > best[1]:
                best = (la, sh)
        return best[0]
    la, _sh = joint_la(vecs, xs, w, rec)
    return la


def main() -> None:
    v1_ref = {"5Iron": 3.44, "7Iron": 3.26, "9Iron": 2.59,
              "Driver": 1.17, "SandWedge": 1.27}
    for variant in ("e", "f"):
        by_club = defaultdict(list)
        for num, rec in sorted(TABLE.items()):
            if rec["block"] != "A" or not track_ok(rec):
                continue
            la = v2_for_shot(num, variant)
            if la is not None:
                by_club[rec["club"].split()[0]].append((la, rec["tm_la"]))
        pooled = []
        line = []
        for club, lst in sorted(by_club.items()):
            fold = []
            for p in (0, 1):
                bias = np.mean([a - b for a, b in lst[p::2]])
                fold += [abs(a - b - bias) for a, b in lst[1 - p::2]]
            pooled += fold
            line.append(f"{club[:2]}:{np.mean(fold):5.2f}"
                        f"(v1 {v1_ref[club]:4.2f})")
        print(f"v2{variant}: pooled {np.mean(pooled):5.2f} "
              f"(v1 2.58) | " + "  ".join(line))


if __name__ == "__main__":
    main()
