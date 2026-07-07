#!/usr/bin/env python3
"""Stage-1 software dry run: synthetic LEVM bytes -> parser -> FBSS-MUSIC.

INTENT (for future sessions / Opus 4.8)
---------------------------------------
This is the end-to-end rehearsal of the Stage-1a static corner-reflector
test, run entirely in software before the board arrives. It:

  1. Synthesizes physically-correct azimuth-heatmap TLV frames for a corner
     reflector at known (range, elevation) — including the coherent floor-
     bounce image and int16 quantization — using the SAME two-ray physics
     as music_stage0_sim.py.
  2. Packs them into demo-format UART bytes (iwr6843_uart.build_frame).
  3. Parses them back (iwr6843_uart.iter_frames / parse_azimuth_heatmap).
  4. Picks the reflector's range bin, feeds the 8-antenna snapshot into
     est_music_fbss (unchanged from the Stage-0 sim), recovers elevation.
  5. PASS if recovered angle is within tolerance of truth across the sweep.

Expected outcome: PASS at <=0.3 deg MAE clean, <=1.0 deg with the floor
image present at Gamma=0.7. When the real board arrives, replace
`synthesize_session()` with SerialReader.frames() and the analysis half
(`analyze_frames`) runs UNCHANGED on real data — that function is the
actual Stage-1a analysis tool, not throwaway scaffolding.

Run:  uv run --extra analysis python stage1_dryrun.py
(needs the analysis extra only because music_stage0_sim imports matplotlib)
"""

import numpy as np

import iwr6843_uart as uart
from music_stage0_sim import GAMMA, LAM, est_music_fbss, gate_w, steer

RANGE_RES = 0.0488        # m/bin for the STATIC cfg (slope 60 MHz/us, HW-verified
                          # 2026-07-07; see iwr6843_levm_static.cfg derived block)
N_BINS = 128
N_ANT = 8
AMP = 3000.0              # reflector amplitude in int16 units (strong target)
NOISE = 30.0              # per-sample noise -> ~40 dB SNR, generous for a
                          # corner reflector at 3 m (ball SNR is Gate S1-D)
H_RADAR = 0.25


def synthesize_frame(range_m, elev_deg, gamma=GAMMA, rng=None):
    """One azimuth-heatmap frame: reflector + coherent floor image + noise.

    Mirrors music_stage0_sim.snapshot() for a STATIC target: the image
    arrives from the mirrored elevation with the bounce path phase, weighted
    by how much of it lands in the direct bin (range gating happens in the
    radar's FFT; we model it with the same Hann-mainlobe weight).
    """
    rng = rng or np.random.default_rng(0)
    theta = np.radians(elev_deg)
    h_target = H_RADAR + range_m * np.sin(theta)
    d = range_m * np.cos(theta)
    r_image = np.hypot(d, h_target + H_RADAR)
    d_r = r_image - range_m
    k = 2 * np.pi / LAM
    w_id = gate_w(d_r / 2)
    w_ii = gate_w(d_r)
    g1 = gamma * np.exp(-1j * k * d_r)
    theta_i = -np.arctan2(h_target + H_RADAR, d)

    heatmap = (rng.standard_normal((N_BINS, N_ANT)) +
               1j * rng.standard_normal((N_BINS, N_ANT))) * NOISE
    snap = AMP * ((1 + g1 * w_id) * steer(theta, N_ANT) +
                  (g1 * w_id + g1 * g1 * w_ii) * steer(theta_i, N_ANT))
    k_bin = int(round(range_m / RANGE_RES))
    heatmap[k_bin] += snap
    return heatmap.astype(np.complex64), k_bin


def analyze_frames(frames, noise_var):
    """The real Stage-1a analysis: for each parsed frame, find the strongest
    range bin in the heatmap and estimate its arrival angle with FBSS-MUSIC.

    Works identically on synthetic frames (here) and SerialReader frames
    (hardware day). Returns list of (frame_number, bin, angle_deg, power_db).
    """
    results = []
    for f in frames:
        payload = f.tlvs.get(uart.TLV_AZIMUTH_STATIC_HEATMAP)
        if payload is None:
            continue
        hm = uart.parse_azimuth_heatmap(payload)
        power = np.sum(np.abs(hm) ** 2, axis=1)
        k = int(np.argmax(power))
        theta = est_music_fbss(hm[k].astype(complex), noise_var)
        results.append((f.header.frame_number, k,
                        float(np.degrees(theta)),
                        float(10 * np.log10(power[k]))))
    return results


def main():
    rng = np.random.default_rng(42)
    print("=" * 74)
    print("STAGE-1 DRY RUN  |  synthetic reflector -> UART bytes -> parser -> MUSIC")
    print("=" * 74)

    sweep = [(-10, 3.0), (-5, 3.0), (0, 3.0), (5, 3.0), (10, 3.0),
             (15, 2.5), (20, 2.5)]
    noise_var = 2 * NOISE ** 2  # per-element complex noise power in int16 units

    for label, gamma in [("clean (no floor image)", 0.0),
                         ("with floor image, Gamma=0.7", 0.7)]:
        errs = []
        print(f"\n--- {label} ---")
        print("  true_el   range   est_el    err    bin")
        for elev, rng_m in sweep:
            # 3 frames per position, exactly like a real capture burst
            raw = b"".join(
                uart.build_frame(
                    {uart.TLV_AZIMUTH_STATIC_HEATMAP:
                     uart.build_azimuth_heatmap_payload(
                         synthesize_frame(rng_m, elev, gamma, rng)[0])},
                    frame_number=i)
                for i in range(3))
            res = analyze_frames(uart.iter_frames(raw), noise_var)
            est = float(np.median([r[2] for r in res]))
            expected_bin = int(round(rng_m / RANGE_RES))
            assert all(r[1] == expected_bin for r in res), \
                f"peak found in wrong range bin: {res} != {expected_bin}"
            err = est - elev
            errs.append(err)
            print(f"  {elev:7.1f} {rng_m:7.2f} {est:8.2f} {err:+7.2f} "
                  f"{res[0][1]:6d}")
        mae = float(np.mean(np.abs(errs)))
        gate = 0.3 if gamma == 0.0 else 1.0
        verdict = "PASS" if mae <= gate else "FAIL"
        print(f"  MAE {mae:.2f} deg  (gate {gate} deg)  ->  {verdict}")
        if verdict == "FAIL":
            raise SystemExit(1)

    print("\nAll dry-run gates passed: parser + estimator chain is ready for "
          "real bytes.\nHardware day: swap synthesize for SerialReader.frames() "
          "and rerun analyze_frames().")


if __name__ == "__main__":
    main()
