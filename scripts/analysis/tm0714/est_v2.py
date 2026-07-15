"""Estimator matrix v2: everything learned today, scored on holdout.

Variants (per shot):
  music_free       unconstrained line on MUSIC points
  music_tee        line through the CORRECTED tee anchor (z0 - rh)
  music_tee_xlim   same, points x <= XLIM only (late-track corruption cut)
  tworay_w         explained-weighted two-ray line fit
  tworay_w_xlim    same, x <= XLIM
  tworay_dom_xlim  + dominance weighting (mid-mix snapshots downweighted)
  golden_k2        strict coherent K=2 two-ray (the golden tier)

Scoring: per club (block A), two-fold split-half bias calibration --
bias fit on one half, MAE scored on the other, folds averaged. TDM local-vr
fix togglable (cache stores per-snapshot vr; 'off' re-rotates to track-avg).
"""
from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import numpy as np

import tm0714
from openflight.iwr6843.doa import TDM_TAU_S
from openflight.iwr6843.music import (LAM, est_bartlett, est_music_fbss_high,
                                      steer)

HERE = Path(__file__).parent
C = np.load(HERE / "cache_tm0714.npz")
TABLE = {r["num"]: r for r in
         json.loads((HERE / "cache_tm0714.json").read_text())}
CAL = tm0714.load_cal()
XLIM = 3.8
TH_GATE = np.radians(-2.5)    # keep snapshots below the corrupted/uncal zone
GRID = np.arange(-0.02, 1.30, 0.01)
AGREE = np.radians(8.0)


def track_ok(rec: dict) -> bool:
    """No-measurement rule: reject thin/ragged tracks (junk captures)."""
    return (rec.get("guided_ms") is not None
            and rec["guided_rms"] < 0.50
            and rec["guided_n"] >= 30
            and rec["n_snaps"] >= 56)


def two_ray_grid_solve(vec: np.ndarray, x_m: float, rh: float,
                       tilt_rad: float) -> tuple[float, float, float]:
    """Vectorized height scan -> (best_h, explained, image_fraction)."""
    th_d = np.arctan2(GRID - rh, x_m) - tilt_rad
    th_i = np.arctan2(-(GRID + rh), x_m) - tilt_rad
    m = np.arange(8)
    sd = np.exp(1j * np.pi * np.sin(th_d)[:, None] * m[None, :])
    si = np.exp(1j * np.pi * np.sin(th_i)[:, None] * m[None, :])
    # normal equations per hypothesis (2x2, solved in closed form)
    a11 = 8.0
    a12 = np.einsum("gm,gm->g", sd.conj(), si)
    a22 = 8.0
    b1 = sd.conj() @ vec
    b2 = si.conj() @ vec
    det = a11 * a22 - np.abs(a12) ** 2
    det = np.where(np.abs(det) < 1e-9, 1e-9, det)
    ca = (a22 * b1 - a12 * b2) / det
    cb = (a11 * b2 - np.conj(a12) * b1) / det
    pred_power = (np.abs(ca) ** 2 * a11 + np.abs(cb) ** 2 * a22
                  + 2 * np.real(np.conj(ca) * cb * a12))
    power = float(np.vdot(vec, vec).real) + 1e-12
    expl = pred_power.real / power
    k = int(np.argmax(expl))
    pa, pb = abs(ca[k]) ** 2, abs(cb[k]) ** 2
    return float(GRID[k]), float(expl[k]), float(pb / (pa + pb + 1e-12))


def line_la(xs, hs, ws=None) -> float | None:
    xs, hs = np.asarray(xs), np.asarray(hs)
    if len(xs) < 6:
        return None
    w = np.ones_like(xs) if ws is None else np.asarray(ws)
    xm = np.average(xs, weights=w)
    hm = np.average(hs, weights=w)
    den = np.sum(w * (xs - xm) ** 2)
    if den <= 0:
        return None
    slope = np.sum(w * (xs - xm) * (hs - hm)) / den
    return float(np.degrees(np.arctan(slope)))


def tee_la(xs, hs, x0, h0) -> float | None:
    xs, hs = np.asarray(xs), np.asarray(hs)
    if len(xs) < 6:
        return None
    dx = xs - x0
    den = np.sum(dx * dx)
    if den <= 0:
        return None
    slope = float(np.sum(dx * (hs - h0)) / den)
    return float(np.degrees(np.arctan(slope)))


def estimates_for(num: int, tdm_local: bool) -> dict[str, float | None]:
    rec = TABLE[num]
    m = C["shot"] == num
    if not m.any() or not rec.get("guided_ms"):
        return {}
    idx = np.nonzero(m)[0]
    vec = C["vec"][idx].copy()
    if not tdm_local:
        delta = (rec["tdm_sign"] * 4 * np.pi
                 * (C["vr"][idx] - rec["guided_ms"]) * TDM_TAU_S / LAM)
        vec[:, :4] *= np.exp(+1j * delta)[:, None]     # undo local-vr fix
    vec = vec * CAL.elem_correction[None, :]
    snr = C["snr"][idx]
    rng = C["r"][idx]
    tilt = np.radians(rec["tilt"])
    rh = rec["rh"]
    out: dict[str, float | None] = {}

    # ---- MUSIC points ----
    xs, hs, thm = [], [], []
    for v, s, r in zip(vec, snr, rng):
        if s < 8.0:
            continue
        noise = float((np.abs(v) ** 2).mean() / s)
        th = est_music_fbss_high(v, noise)
        if abs(th - est_bartlett(v)) > AGREE:
            continue
        w = th + tilt
        xs.append(r * np.cos(w))
        hs.append(r * np.sin(w))
        thm.append(th)
    xs, hs, thm = np.asarray(xs), np.asarray(hs), np.asarray(thm)
    out["music_free"] = line_la(xs, hs)
    x0, h0 = rec["x_tee"], rec["z0"] - rh
    out["music_tee"] = tee_la(xs, hs, x0, h0)
    ml = xs <= XLIM
    if ml.sum() >= 6:
        out["music_tee_xlim"] = tee_la(xs[ml], hs[ml], x0, h0)
    else:
        out["music_tee_xlim"] = out["music_tee"]
    # angle-domain gate: keep points inside the calibrated/clean region
    tg = thm <= TH_GATE
    out["music_tee_th"] = (tee_la(xs[tg], hs[tg], x0, h0)
                           if tg.sum() >= 6 else out["music_tee"])

    # ---- two-ray on K=1 snapshots ----
    txs, ths, tws, tdom, tbart = [], [], [], [], []
    for v, s, r in zip(vec, snr, rng):
        if s < 8.0:
            continue
        h, e, imf = two_ray_grid_solve(v, float(r), rh, tilt)
        if e >= 0.70 and np.isfinite(h):
            txs.append(float(r))
            ths.append(h)
            tws.append(e - 0.70 + 1e-3)
            tdom.append(0.25 + 0.75 * 2 * abs(imf - 0.5))
            tbart.append(est_bartlett(v))
    txs = np.asarray(txs)
    ths_a = np.asarray(ths)
    tws = np.asarray(tws)
    tdom = np.asarray(tdom)
    tbart = np.asarray(tbart)
    out["tworay_n"] = len(txs)

    def tr_la(mask, ws):
        if mask.sum() < 6:
            return None
        return line_la(txs[mask], ths_a[mask], ws[mask])

    allm = np.ones(len(txs), dtype=bool)
    xm = txs <= XLIM if len(txs) else allm
    if len(txs) and xm.sum() < 6:
        xm = allm
    out["tworay_w"] = tr_la(allm, tws) if len(txs) else None
    out["tworay_w_xlim"] = tr_la(xm, tws) if len(txs) else None
    out["tworay_dom_xlim"] = tr_la(xm, tws * tdom) if len(txs) else None
    if len(txs):
        tg2 = tbart <= TH_GATE
        out["tworay_w_th"] = (tr_la(tg2, tws) if tg2.sum() >= 6
                              else out["tworay_w"])
    else:
        out["tworay_w_th"] = None

    # tee-ANCHORED two-ray: heights are above-floor, so the anchor is the
    # physically known ball height z0 at the tee -- restores lever arm when
    # the angle gate cuts the top of the flight
    def tr_tee(mask, ws):
        if mask.sum() < 4:
            return None
        dx = txs[mask] - rec["x_tee"]
        den = np.sum(ws[mask] * dx * dx)
        if den <= 0:
            return None
        slope = float(np.sum(ws[mask] * dx * (ths_a[mask] - rec["z0"]))
                      / den)
        return float(np.degrees(np.arctan(slope)))

    if len(txs):
        out["tworay_tee"] = tr_tee(allm, tws)
        out["tworay_tee_th"] = (tr_tee(tg2, tws) if tg2.sum() >= 4
                                else out["tworay_tee"])
    else:
        out["tworay_tee"] = out["tworay_tee_th"] = None

    # ---- golden strict K=2 coherent ----
    order = np.argsort(C["t"][idx])
    by_frame: dict[tuple[int, int], list[int]] = defaultdict(list)
    for j in order:
        by_frame[(int(C["frame"][idx[j]]), 0)].append(j)
    gxs, ghs, gws = [], [], []
    noise_ref = float(np.median((np.abs(vec) ** 2).mean(axis=1) / snr))
    for _fr, js in by_frame.items():
        js_sorted = sorted(js, key=lambda j: C["loop"][idx[j]])
        for a, b in zip(js_sorted[:-1:2], js_sorted[1::2]):
            if C["loop"][idx[b]] != C["loop"][idx[a]] + 1:
                continue
            acc = vec[a] + vec[b] * np.exp(-1j * C["lph"][idx[a]])
            s2 = float((np.abs(acc) ** 2).mean() / (noise_ref * 2))
            if s2 < 8.0 * 2:                       # strict full-gain gate
                continue
            r_mid = 0.5 * (rng[a] + rng[b])
            h, e, _imf = two_ray_grid_solve(acc, float(r_mid), rh, tilt)
            if e >= 0.80 and np.isfinite(h):
                gxs.append(float(r_mid))
                ghs.append(h)
                gws.append(e - 0.80 + 1e-3)
    out["golden_n"] = len(gxs)
    out["golden_k2"] = (line_la(gxs, ghs, gws) if len(gxs) >= 6 else None)
    return out


ESTS = ("music_tee", "music_tee_th", "tworay_w", "tworay_dom_xlim",
        "tworay_tee", "tworay_tee_th", "golden_k2")


def score_block_a(res: dict[int, dict], tag: str) -> None:
    print(f"\n{'='*74}\nSCORE block A ({tag}) -- split-half bias-cal MAE "
          f"per club\n{'='*74}")
    by_club: dict[str, list[int]] = defaultdict(list)
    n_rej = 0
    for num, rec in sorted(TABLE.items()):
        if rec["block"] != "A":
            continue
        if not track_ok(rec):
            n_rej += 1
            continue
        by_club[rec["club"].split()[0]].append(num)
    print(f"  track-quality gate rejected {n_rej} shots (no-measurement)")
    pooled = {e: [] for e in ESTS}
    cov = {e: [0, 0] for e in ESTS}
    for club, nums in sorted(by_club.items()):
        line = [f"{club:10s} n={len(nums):2d}"]
        for e in ESTS:
            pairs = [(res[n][e], TABLE[n]["tm_la"]) for n in nums
                     if res.get(n, {}).get(e) is not None]
            cov[e][0] += len(pairs)
            cov[e][1] += len(nums)
            if len(pairs) < 6:
                line.append(f"{e}: n<6")
                continue
            fold = []
            for par in (0, 1):
                cal_p = pairs[par::2]
                sco_p = pairs[1 - par::2]
                bias = np.mean([m - t for m, t in cal_p])
                fold += [abs(m - t - bias) for m, t in sco_p]
            mae = float(np.mean(fold))
            pooled[e] += fold
            line.append(f"{e}:{mae:5.2f}")
        print("  " + "  ".join(line))
    print("  " + "-" * 70)
    parts = []
    for e in ESTS:
        mae = np.mean(pooled[e]) if pooled[e] else float("nan")
        cpct = 100 * cov[e][0] / max(cov[e][1], 1)
        parts.append(f"{e}: {mae:5.2f} ({cpct:3.0f}%)")
    print("  POOLED holdout MAE (coverage):\n    " + "\n    ".join(parts))


def main() -> None:
    for tdm_local in (True, False):
        res = {num: estimates_for(num, tdm_local)
               for num, rec in sorted(TABLE.items()) if rec.get("guided_ms")}
        tag = "TDM local-vr ON" if tdm_local else "TDM track-avg (old)"
        score_block_a(res, tag)
        if tdm_local:
            Path(HERE / "est_v2_results.json").write_text(json.dumps(
                {str(k): v for k, v in res.items()}, indent=1))
            n_g = [v.get("golden_n", 0) for v in res.values()
                   if TABLE[int([k for k, vv in res.items()
                                 if vv is v][0])]["block"] == "A"] \
                if False else [v.get("golden_n", 0) for v in res.values()]
            print(f"\n  golden K2 point counts: med "
                  f"{int(np.median(n_g))}, >=6 on "
                  f"{sum(1 for g in n_g if g >= 6)}/{len(n_g)} shots")


if __name__ == "__main__":
    main()
