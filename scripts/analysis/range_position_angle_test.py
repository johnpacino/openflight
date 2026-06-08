#!/usr/bin/env python
"""Position-based launch-angle test (range + bearing per frame).

Theory: the KLD7 FSK two-frequency phase gives RANGE per frame, so each ball
frame is a full (x, y) position -- not just a bearing. Fit a line through the
positions and the slope IS the launch angle: no assumed distance, no speed.

The frame selection is a CONFIG block you can loosen. New lever: a DISTANCE gate
(range must be in-bounds and increasing as the ball recedes) -- enabled by the
range measurement -- to admit more good frames without admitting clutter.

Run:  PYTHONPATH=src python scripts/analysis/range_position_angle_test.py
"""
import base64
import json
import math

import numpy as np
from openflight.kld7.radc import (
    parse_radc_payload, to_complex_iq, compute_fft_complex, per_bin_angle_deg,
    expected_ball_bin_from_speed, ball_bin_range_from_speed, circular_bin_distance,
    _find_peak_near_expected_bin, _centroid_angle_for_peak,
)

# ============================== CONFIG (tune me) ==============================
SESSION = "/Users/john.pacino/openflight_sessions/session_20260530_105209_range.jsonl"
# TrackMan ground-truth Launch Angle per shot (None to skip comparison):
TRACKMAN = {1: 8.9, 2: 18.1, 3: 20.3, 4: 19.7, 5: 19.4, 6: 20.4, 7: 20.9, 8: 12.4,
            9: 20.4, 10: 18.2, 11: 19.3, 12: 20.8, 13: 17.4, 14: 21.2, 15: 16.0, 16: 7.8,
            17: 20.6, 18: 18.6, 19: 21.7, 20: 19.5, 21: 6.4, 22: 14.5, 23: 17.4, 24: 18.0,
            25: 15.0, 26: 14.4, 27: 17.4, 28: 19.5, 29: 19.6, 30: 18.7, 31: 17.9, 32: 21.9}
IGNORE = {16, 21}

MOUNT_DEG = 10.0        # radar up-tilt (elevation = bearing + offset + mount)
OFFSET_DEG = 2.5        # boresight calibration offset
R_UNAMB_FT = 5.0 * 3.28084   # range=5m setting -> 16.4 ft unambiguous

# --- frame selection knobs (loosen to admit more frames) ---
WINDOW_MS = (5.0, 110.0)     # impact-relative window to search
BIN_ERR_MAX = 25             # |peak bin - OPS expected| ; raise to loosen
SNR_MIN = 0.0                # 0 = ignore SNR
# distance gate (the new lever the range enables):
RANGE_MIN_FT = 4.0           # reject too-near / wrapped
RANGE_MAX_FT = 16.0          # reject near the 16.4 ft wrap
REQUIRE_RANGE_INCREASING = True   # ball must be receding (drops wrapped/clutter)
MIN_FRAMES = 2               # need >= this many surviving frames to fit
# ============================================================================

FFT, MAXKMH = 2048, 100.0


def frame_metrics(radc, ball):
    ch = parse_radc_payload(radc)
    f1a = compute_fft_complex(to_complex_iq(ch["f1a_i"], ch["f1a_q"]), fft_size=FFT)
    f2a = compute_fft_complex(to_complex_iq(ch["f2a_i"], ch["f2a_q"]), fft_size=FFT)
    f1b = compute_fft_complex(to_complex_iq(ch["f1b_i"], ch["f1b_q"]), fft_size=FFT)
    spec = np.abs(f1a)
    exp = expected_ball_bin_from_speed(ball, FFT, MAXKMH)
    bands = ball_bin_range_from_speed(ball, 10.0, FFT, MAXKMH)
    pb, pv, pband = _find_peak_near_expected_bin(spec, bands, exp, BIN_ERR_MAX, FFT)
    if pb is None:
        return None
    ang, _w = _centroid_angle_for_peak(per_bin_angle_deg(f1a, f2a), spec, pb, pv, pband, 0.5)
    dphi = float(np.angle(f1b[pb] * np.conj(f1a[pb])))
    rng_ft = (dphi % (2 * math.pi)) / (2 * math.pi) * R_UNAMB_FT
    pos = spec[spec > 0]
    med = float(np.median(pos)) if pos.size else 0.0
    return {"bin_err": circular_bin_distance(pb, exp, FFT), "ang": float(ang),
            "rng_ft": rng_ft, "snr": (pv / med if med > 0 else 0.0)}


def select_frames(buf, ball):
    base = buf["shot_timestamp"]
    cand = []
    for fr in buf.get("frames", []):
        b64, ts = fr.get("radc_b64"), fr.get("timestamp")
        if not b64 or ts is None:
            continue
        try:
            radc = base64.b64decode(b64)
        except Exception:
            continue
        if len(radc) != 3072:
            continue
        t = (float(ts) - base) * 1000.0
        if not (WINDOW_MS[0] <= t <= WINDOW_MS[1]):
            continue
        m = frame_metrics(radc, ball)
        if m is None:
            continue
        if m["bin_err"] > BIN_ERR_MAX or m["snr"] < SNR_MIN:
            continue
        if not (RANGE_MIN_FT <= m["rng_ft"] <= RANGE_MAX_FT):
            continue
        m["t"] = t
        cand.append(m)
    cand.sort(key=lambda f: f["t"])
    if REQUIRE_RANGE_INCREASING:
        kept = []
        for f in cand:
            if not kept or f["rng_ft"] > kept[-1]["rng_ft"]:
                kept.append(f)
        cand = kept
    return cand


def launch_angle_from_positions(frames):
    """Fit a line through the (x,y) ball positions; slope -> launch angle (deg)."""
    xs, ys = [], []
    for f in frames:
        phi = math.radians(f["ang"] + OFFSET_DEG + MOUNT_DEG)  # elevation from horizontal
        xs.append(f["rng_ft"] * math.cos(phi))   # downrange
        ys.append(f["rng_ft"] * math.sin(phi))   # height
    xs, ys = np.array(xs), np.array(ys)
    if np.ptp(xs) < 1e-6:
        return None
    slope = np.polyfit(xs, ys, 1)[0]
    return math.degrees(math.atan(slope))


def main():
    shots, kld = {}, {}
    for line in open(SESSION):
        try:
            d = json.loads(line)
        except Exception:
            continue
        if d.get("type") == "shot_detected":
            shots[d["shot_number"]] = d
        elif d.get("type") == "kld7_buffer" and d.get("orientation") == "vertical":
            kld[d["shot_number"]] = d

    print(f"selection: window={WINDOW_MS} bin_err<={BIN_ERR_MAX} snr>={SNR_MIN} "
          f"range=[{RANGE_MIN_FT},{RANGE_MAX_FT}]ft inc={REQUIRE_RANGE_INCREASING} minN={MIN_FRAMES}")
    print(f"{'sh':>2} {'ball':>5} {'nF':>2} | {'frames t(rng_ft@ang)':>40} | {'ALPHA':>6} {'TM':>5} {'err':>6}")
    print("-" * 92)
    errs = []
    for n in sorted(shots):
        buf = kld.get(n)
        if not buf:
            continue
        ball = shots[n].get("ball_speed_mph") or 0.0
        frames = select_frames(buf, ball)
        tm = TRACKMAN.get(n) if TRACKMAN else None
        tag = " IGN" if n in IGNORE else ""
        if len(frames) < MIN_FRAMES:
            print(f"{n:>2} {ball:>5.0f} {len(frames):>2} | {'(too few frames)':>40} |"
                  f" {'--':>6} {(tm or 0):>5.1f}{tag}")
            continue
        alpha = launch_angle_from_positions(frames)
        cells = " ".join(f"{f['t']:.0f}({f['rng_ft']:.1f}@{f['ang']:+.0f})" for f in frames)
        err = (alpha - tm) if (tm is not None and alpha is not None) else None
        if tm is not None and err is not None and n not in IGNORE:
            errs.append(err)
        es = f"{err:+5.1f}" if err is not None else "  -"
        print(f"{n:>2} {ball:>5.0f} {len(frames):>2} | {cells:>40} | {alpha:>5.1f}° "
              f"{(tm or 0):>5.1f} {es}{tag}")

    if errs:
        mae = sum(abs(e) for e in errs) / len(errs)
        print(f"\nvs TrackMan (n={len(errs)}, ignoring {sorted(IGNORE)}): "
              f"bias={sum(errs)/len(errs):+.1f}°  MAE={mae:.1f}°  "
              f"std={np.std(errs):.1f}°  solved={len(errs)}/{len(TRACKMAN)-len(IGNORE)}")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="position-based launch-angle test (loosenable)")
    p.add_argument("--session", default=SESSION)
    p.add_argument("--bin-err", type=int, default=BIN_ERR_MAX)
    p.add_argument("--snr-min", type=float, default=SNR_MIN)
    p.add_argument("--win-hi", type=float, default=WINDOW_MS[1])
    p.add_argument("--min-frames", type=int, default=MIN_FRAMES)
    p.add_argument("--range-inc", choices=["on", "off"],
                   default="on" if REQUIRE_RANGE_INCREASING else "off")
    p.add_argument("--range-max", type=float, default=RANGE_MAX_FT)
    a = p.parse_args()
    SESSION = a.session
    BIN_ERR_MAX = a.bin_err
    SNR_MIN = a.snr_min
    WINDOW_MS = (WINDOW_MS[0], a.win_hi)
    MIN_FRAMES = a.min_frames
    REQUIRE_RANGE_INCREASING = a.range_inc == "on"
    RANGE_MAX_FT = a.range_max
    main()
