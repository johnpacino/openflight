"""One pass over all 101 truth dumps -> snapshot cache (npz) + shot table.

Per shot: guided track (TM speed window; value stays radar-measured), stock
production track for comparison, per-loop raw snapshots with TM-predicted
direct/image angles. Everything downstream reads the cache.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

import tm0714

WINDOW = sys.argv[1] if len(sys.argv) > 1 else None
OUT = Path(__file__).parent / ("cache_tm0714" + (f"_{WINDOW}" if WINDOW
                                                 else ""))


def main() -> None:
    shots = tm0714.load_shots()
    cal = tm0714.load_cal()
    print(f"{len(shots)} truth shots; extracting (window={WINDOW})...")
    vecs, cols = [], {k: [] for k in
                      ("shot", "t", "frame", "loop", "r", "snr",
                       "th_d", "th_i", "x", "z", "vr", "lph")}
    table = []
    t_start = time.time()
    for s in shots:
        try:
            mti, geo = tm0714.load_mti(s.file, window=WINDOW)
        except (OSError, ValueError) as err:
            print(f"  {s.num}: LOAD FAIL {err}")
            continue
        trk = tm0714.find_guided(mti, geo, s.tm_ball_ms)
        stock = None
        try:
            from openflight.iwr6843.tracking import find_ball
            stock = find_ball(mti, geo, max_range_m=4.95)
        except Exception:                                    # noqa: BLE001
            pass
        rec = dict(num=s.num, club=s.club, block=s.block,
                   tm_la=s.tm_la, tm_ball_ms=s.tm_ball_ms,
                   tm_dir=s.tm_dir_deg, tm_club_mph=s.tm_club_mph,
                   tm_spin=s.tm_spin, tm_attack=s.tm_attack,
                   of_ball_mph=s.of_ball_mph, live=s.live,
                   tee_slant=s.tee_slant, rh=s.rh, tilt=s.tilt_deg,
                   z0=s.z0, x_tee=s.x_tee,
                   stock_ms=(stock.speed_ms if stock else None),
                   stock_lowconf=(stock.low_confidence if stock else None))
        if trk is None:
            rec.update(guided_ms=None, n_snaps=0)
            table.append(rec)
            print(f"  {s.num} {s.club:18s}: NO GUIDED TRACK")
            continue
        snaps = tm0714.raw_snapshots(mti, trk, geo, cal.range_bias_m)
        for sn in snaps:
            x = s.x_from_r(sn["r"])
            th_d, th_i = s.angles_at(x)
            vecs.append(sn["vec"])
            for k, v in (("shot", s.num), ("t", sn["t"]),
                         ("frame", sn["frame"]), ("loop", sn["loop"]),
                         ("r", sn["r"]), ("snr", sn["snr"]),
                         ("th_d", th_d), ("th_i", th_i),
                         ("x", x), ("z", s.z_at(x)),
                         ("vr", sn["v_r"]), ("lph", sn["loop_phase"])):
                cols[k].append(v)
        # predicted average radial slope over the track span (cosine check)
        r0 = trk.range_at(trk.t_first, geo.range_res_m) - cal.range_bias_m
        r1 = trk.range_at(trk.t_last, geo.range_res_m) - cal.range_bias_m
        x0, x1 = s.x_from_r(r0), s.x_from_r(r1)
        la = np.radians(s.tm_la)
        dt = (x1 - x0) / (s.tm_ball_ms * np.cos(la)
                          * np.cos(np.radians(s.tm_dir_deg)))
        cos_factor = (s.r_at(x1) - s.r_at(x0)) / (s.tm_ball_ms * dt)
        rec.update(guided_ms=trk.speed_ms, guided_rms=trk.rms_bins,
                   guided_n=trk.n_inliers, t_first=trk.t_first,
                   t_last=trk.t_last, loop_phase=snaps[0]["loop_phase"],
                   tdm_sign=snaps[0]["sign"], n_snaps=len(snaps),
                   cos_factor=float(cos_factor), x_first=x0, x_last=x1)
        table.append(rec)
    np.savez_compressed(
        OUT.with_suffix(".npz"),
        vec=np.array(vecs),
        **{k: np.array(v) for k, v in cols.items()})
    Path(OUT.with_suffix(".json")).write_text(json.dumps(table, indent=1))
    n_ok = sum(1 for r in table if r.get("guided_ms"))
    print(f"done in {time.time()-t_start:.0f}s: {n_ok}/{len(table)} tracked, "
          f"{len(vecs)} snapshots cached -> {OUT}.npz")


if __name__ == "__main__":
    main()
