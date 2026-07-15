"""Finding #1 (tee-anchor height bug) + Finding #2 (cosine speed) rescore.

Replicates the production angle chain from the snapshot cache: apply element
cal, gate snr>=8, MUSIC-high with Bartlett agreement <=8 deg, then line fits.
Compares fit_tee with the shipped anchor (h0=+0.04 above radar plane) vs the
physically correct anchor (ball height above floor minus radar height).
"""
from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import numpy as np

import tm0714
from openflight.iwr6843.music import est_bartlett, est_music_fbss_high

HERE = Path(__file__).parent
CACHE = np.load(HERE / "cache_tm0714.npz")
TABLE = {r["num"]: r for r in
         json.loads((HERE / "cache_tm0714.json").read_text())}
CAL = tm0714.load_cal()
AGREE = np.radians(8.0)


def music_points(num: int) -> tuple[np.ndarray, np.ndarray]:
    """Production-equivalent (x, h) angle points for one shot (world frame)."""
    m = CACHE["shot"] == num
    rec = TABLE[num]
    tilt = np.radians(rec["tilt"])
    vec = CACHE["vec"][m] * CAL.elem_correction[None, :]
    snr = CACHE["snr"][m]
    rng = CACHE["r"][m]
    xs, hs = [], []
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
    return np.asarray(xs), np.asarray(hs)


def tee_fit(xs: np.ndarray, hs: np.ndarray, x0: float, h0: float
            ) -> float | None:
    if len(xs) < 8:
        return None
    dx = xs - x0
    slope = float(np.sum(dx * (hs - h0)) / np.sum(dx * dx))
    return float(np.degrees(np.arctan(slope)))


def free_fit(xs: np.ndarray, hs: np.ndarray) -> float | None:
    if len(xs) < 8:
        return None
    slope = np.polyfit(xs, hs, 1)[0]
    return float(np.degrees(np.arctan(slope)))


def stats(pairs: list[tuple[float, float]]) -> str:
    if not pairs:
        return "n=0"
    err = np.array([m - t for m, t in pairs])
    return (f"n={len(err):2d} bias={err.mean():+6.2f} "
            f"mae={np.abs(err).mean():5.2f} "
            f"mae-cal={np.abs(err - err.mean()).mean():5.2f} "
            f"sd={err.std():5.2f}")


def main() -> None:
    print("=" * 78)
    print("FINDING 1 -- tee anchor: shipped h0=+0.04 (radar plane) vs "
          "physical h0=z0-rh")
    print("=" * 78)
    rows = defaultdict(lambda: defaultdict(list))
    per_shot = {}
    for num, rec in sorted(TABLE.items()):
        if not rec.get("guided_ms"):
            continue
        xs, hs = music_points(num)
        old = tee_fit(xs, hs, rec["tee_slant"], 0.04)
        new = tee_fit(xs, hs, rec["x_tee"], rec["z0"] - rec["rh"])
        fre = free_fit(xs, hs)
        per_shot[num] = dict(old=old, new=new, free=fre, n=len(xs))
        club = rec["club"].split()[0].replace("(relabeled)", "")
        key = (rec["block"], club)
        for name, val in (("old", old), ("new", new), ("free", fre)):
            if val is not None:
                rows[key][name].append((val, rec["tm_la"]))
    for (blk, club), d in sorted(rows.items()):
        print(f"\n block {blk}  {club}")
        for name in ("old", "new", "free"):
            print(f"   tee-{name if name != 'free' else '    free'}"
                  f"  {stats(d[name])}")
    # pooled block A
    print("\n pooled BLOCK A (production mount):")
    for name in ("old", "new", "free"):
        allp = [p for (blk, _c), d in rows.items() if blk == "A"
                for p in d[name]]
        print(f"   {name:5s} {stats(allp)}")
    Path(HERE / "tee_rescore.json").write_text(json.dumps(per_shot, indent=1))

    print()
    print("=" * 78)
    print("FINDING 2 -- cosine projection on ball speed (guided radar track "
          "vs TM)")
    print("=" * 78)
    per_club = defaultdict(list)
    for num, rec in sorted(TABLE.items()):
        if not rec.get("guided_ms") or rec["block"] != "A":
            continue
        club = rec["club"].split()[0].replace("(relabeled)", "")
        raw = rec["guided_ms"] * tm0714.MPH
        fixed = rec["guided_ms"] / rec["cos_factor"] * tm0714.MPH
        tm = rec["tm_ball_ms"] * tm0714.MPH
        per_club[club].append((raw - tm, fixed - tm, rec["cos_factor"]))
    print(f"{'club':10s} {'n':>3s} {'raw bias':>9s} {'raw sd':>7s} "
          f"{'cos bias':>9s} {'cos sd':>7s} {'cos_factor':>11s}")
    allr, allf = [], []
    for club, lst in sorted(per_club.items()):
        r = np.array([a for a, _b, _c in lst])
        f = np.array([b for _a, b, _c in lst])
        c = np.array([c for _a, _b, c in lst])
        allr += list(r)
        allf += list(f)
        print(f"{club:10s} {len(lst):3d} {r.mean():+9.2f} {r.std():7.2f} "
              f"{f.mean():+9.2f} {f.std():7.2f} "
              f"{c.mean():11.4f}")
    allr, allf = np.array(allr), np.array(allf)
    print(f"{'ALL':10s} {len(allr):3d} {allr.mean():+9.2f} {allr.std():7.2f} "
          f"{allf.mean():+9.2f} {allf.std():7.2f}")
    print(f"\n  |err| raw -> cos-corrected: "
          f"{np.abs(allr).mean():.2f} -> {np.abs(allf).mean():.2f} mph "
          f"(worst {np.abs(allf).max():.1f})")


if __name__ == "__main__":
    main()
