#!/usr/bin/env python3
"""Stage-1 offline analysis: captured raw_uart.bin -> elevation angle table.

INTENT (for future sessions / Opus 4.8)
---------------------------------------
Entry point #2 on hardware day (run after stage1_capture.py). Reads the
raw byte stream + session_meta.json, parses every azimuth-heatmap frame,
estimates the arrival angle of the strongest range bin with FBSS-MUSIC,
and compares against the geometry ground truth in the metadata.

This is the Gate S1-A tool. Expected outcome on a real corner reflector:
median estimated elevation within +/-0.5 deg of the tape-measured truth
(after board tilt is subtracted), stable across frames. Large constant
offset -> check board_tilt_deg in meta (the K-LD7 +2 deg boresight lesson).
Estimates scattered/garbage -> re-run stage1_capture.py --verify and check
the A1/A3 verdicts before suspecting the estimator.

Differences from stage1_dryrun.analyze_frames (the reference impl):
  - noise power is ESTIMATED from the data (median bin power across the
    heatmap, robust to one strong target) instead of passed in, because
    real captures don't come with a known noise floor.
  - ground-truth comparison from session_meta.json reflector_positions.

Fully tested pre-hardware: tests/test_stage1_tools.py generates a synthetic
session on disk (same physics as stage1_dryrun) and runs main() on it.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import iwr6843_uart as uart
from music_stage0_sim import est_music_fbss


def estimate_noise_var(hm: np.ndarray) -> float:
    """Per-element complex noise power via median bin power (robust: the
    reflector occupies ~1 bin of 128+, so the median bin is noise)."""
    per_bin_elem_power = np.mean(np.abs(hm) ** 2, axis=1)
    return float(np.median(per_bin_elem_power))


def analyze_capture(raw: bytes, range_res: float):
    """Yield (frame_number, bin, range_m, elev_deg, peak_db, snr_db)."""
    for frame in uart.iter_frames(raw):
        payload = frame.tlvs.get(uart.TLV_AZIMUTH_STATIC_HEATMAP)
        if payload is None:
            continue
        hm = uart.parse_azimuth_heatmap(payload)
        power = np.sum(np.abs(hm) ** 2, axis=1)
        k = int(np.argmax(power))
        nv = estimate_noise_var(hm)
        theta = est_music_fbss(hm[k].astype(complex), nv)
        snr_db = 10 * np.log10(power[k] / (nv * hm.shape[1]) + 1e-12)
        yield (frame.header.frame_number, k, k * range_res,
               float(np.degrees(theta)),
               float(10 * np.log10(power[k] + 1e-12)), float(snr_db))


def expected_elevation(meta: dict) -> float | None:
    """Tape-measure truth: elevation of the reflector as seen by the radar,
    in the radar's frame (board tilt subtracted)."""
    try:
        pos = meta["reflector_positions"][0]
        h_r = float(meta["radar_height_m"])
        r = float(pos["range_m"])
        h_t = float(pos["height_m"])
        tilt = float(meta.get("board_tilt_deg", 0.0))
        return float(np.degrees(np.arcsin((h_t - h_r) / r)) - tilt)
    except (KeyError, TypeError, ValueError, IndexError):
        return None  # meta still has EDIT_ME placeholders


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--session", required=True,
                    help="session dir containing raw_uart.bin + session_meta.json")
    args = ap.parse_args(argv)

    sess = Path(args.session).expanduser()
    raw = (sess / "raw_uart.bin").read_bytes()
    meta = json.loads((sess / "session_meta.json").read_text())
    range_res = float(meta.get("range_res_m", 0.0468))

    rows = list(analyze_capture(raw, range_res))
    if not rows:
        print("no azimuth-heatmap frames found — was the static cfg used? "
              "(ball cfg disables the heatmap TLV)")
        return 1

    print(f"{'frame':>6} {'bin':>5} {'range_m':>8} {'elev_deg':>9} "
          f"{'peak_dB':>8} {'snr_dB':>7}")
    for r in rows:
        print(f"{r[0]:6d} {r[1]:5d} {r[2]:8.2f} {r[3]:9.2f} {r[4]:8.1f} {r[5]:7.1f}")

    elevs = np.array([r[3] for r in rows])
    med = float(np.median(elevs))
    print(f"\nframes: {len(rows)}  median elev: {med:+.2f} deg  "
          f"spread (IQR): {np.percentile(elevs, 75) - np.percentile(elevs, 25):.2f} deg")

    truth = expected_elevation(meta)
    if truth is None:
        print("ground truth unavailable (EDIT_ME placeholders in "
              "session_meta.json) — fill geometry and rerun")
    else:
        err = med - truth
        verdict = "PASS" if abs(err) <= 0.5 else "FAIL"
        print(f"truth (from geometry, tilt-corrected): {truth:+.2f} deg  "
              f"error: {err:+.2f} deg  ->  Gate S1-A: {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
