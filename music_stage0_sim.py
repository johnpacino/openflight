#!/usr/bin/env python3
"""Stage-0 simulation: can FBSS-MUSIC on IWR6843AOP-class elevation arrays
separate the coherent floor bounce and beat 2 deg launch-angle MAE?

Pre-registered pass gate (agreed 2026-07-01, before hardware purchase):
  FBSS-MUSIC on the AOP-class 4-element elevation array achieves
  LA MAE < 2.0 deg at SNR >= 20 dB with 3 deg rms per-element phase-cal
  error, across the launch matrix, with range-gating physics included.

Signal model: monostatic two-ray multipath -> FOUR paths (DD, ID, DI, II).
The TX-side floor path does not change RX arrival angle, so the array sees
two coherent arrivals (theta_direct, theta_image) whose amplitudes carry the
extra bounce path phase. Each path is weighted by a Hann-mainlobe range-gate
response around the direct-return range bin (4 GHz sweep -> 3.75 cm bins),
so range-gating benefit is included per frame automatically.

Credibility check: the 2-element interferometry baseline (K-LD7-class) must
reproduce the field-observed failure signature: low-ball frames biased LOW.
"""

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------------------------------------------- constants
# Constants + array model + estimators now live in music_core (matplotlib-free,
# importable by runtime code). Re-imported here; main() rebinds LAM/RANGE_RES
# (Coleman proxy) in THIS module's namespace, not music_core's.
from music_core import (  # noqa: E402
    C, LAM, RANGE_RES, G, steer,
    est_interferometry, est_bartlett, est_music_fbss,
)

# ------------------------------------------------------------ shot geometry
H_RADAR = 0.25               # 10-inch mount (enclosure constraint)
X_TEE = 2.0                  # radar 2 m behind tee
X_NET = 3.0                  # measure until net, 3 m past tee
H_TEE = 0.04
FRAME_S = 0.007              # ~143 fps custom chirp config
MAX_FRAMES = 16

GAMMA = 0.70                 # specular floor reflection magnitude (hard floor)
CAL_RMS_DEG = 3.0            # per-element phase calibration error (rms)
SNR_DB = 20.0                # per-element post-2D-FFT SNR at pass gate
RANGE_NOISE_M = 0.01
N_TRIAL = 150
PASS_GATE_DEG = 2.0

# (launch angle deg, ball speed m/s): driver stinger -> wedge
LAUNCHES = [(8, 72), (12, 65), (16, 55), (20, 45), (25, 30)]

RNG = np.random.default_rng(7)

# ------------------------------------------------------------------ physics


def trajectory(la_deg, v):
    la = np.radians(la_deg)
    vx, vz = v * np.cos(la), v * np.sin(la)
    t = (np.arange(MAX_FRAMES) + 0.5) * FRAME_S
    x = vx * t
    keep = x <= X_NET
    t, x = t[keep], x[keep]
    h = H_TEE + vz * t - 0.5 * G * t * t
    d = X_TEE + x
    return dict(
        t=t, x=x, h=h, d=d, vx=vx,
        r_d=np.hypot(d, h - H_RADAR),
        r_i=np.hypot(d, h + H_RADAR),
        th_d=np.arctan2(h - H_RADAR, d),
        th_i=np.arctan2(-(h + H_RADAR), d),
    )


def gate_w(delta_m):
    """Hann-mainlobe range-gate response at a path offset from the DD bin."""
    b = np.abs(delta_m) / RANGE_RES
    return np.where(b <= 2.0, np.cos(np.pi * b / 4.0) ** 2, 0.006)


def snapshot(tr, i, n_el, snr_db, cal_phase, rng):
    """Per-antenna complex vector at the ball's range-Doppler bin, one frame."""
    k = 2.0 * np.pi / LAM
    d_r = tr["r_i"][i] - tr["r_d"][i]
    w_id = gate_w(d_r / 2.0)         # single-bounce cross terms (ID and DI)
    w_ii = gate_w(d_r)               # double-bounce
    g1 = GAMMA * np.exp(-1j * k * d_r)
    a_d = steer(tr["th_d"][i], n_el)
    a_i = steer(tr["th_i"][i], n_el)
    s = (1.0 + g1 * w_id) * a_d + (g1 * w_id + g1 * g1 * w_ii) * a_i
    s = s * np.exp(1j * cal_phase)
    sigma = 10.0 ** (-snr_db / 20.0)
    noise = (rng.standard_normal(n_el) + 1j * rng.standard_normal(n_el)) * sigma / np.sqrt(2.0)
    return s + noise, sigma ** 2, w_id


# ------------------------------------------------------------ LA extraction


def fit_launch_angle(r_meas, th_hat, vx, quality=None):
    """Per-frame (range, elevation) -> heights -> gravity-corrected slope fit.

    Robust version, mirroring the production pipeline's behavior: deep-fade
    frames are rejected on measured snapshot power, and the slope comes from
    a Theil-Sen (median of pairwise slopes) estimator so a residual outlier
    frame cannot capsize the fit. Deterministic, no training data.
    """
    h = H_RADAR + r_meas * np.sin(th_hat)
    x = r_meas * np.cos(th_hat) - X_TEE
    y = h + (G / (2.0 * vx * vx)) * x * x        # remove known ballistic sag
    if quality is not None and len(x) >= 4:
        keep = quality >= 0.5 * np.median(quality)   # drop deep fades
        if keep.sum() >= 3:
            x, y = x[keep], y[keep]
    n = len(x)
    slopes = [
        (y[j] - y[i]) / (x[j] - x[i])
        for i in range(n) for j in range(i + 1, n)
        if abs(x[j] - x[i]) > 1e-6
    ]
    return float(np.degrees(np.arctan(np.median(slopes))))


# ------------------------------------------------------------------ configs
CONFIGS = [
    # (label, family, n_el, snr_bonus_db, estimator)
    ("Interferometry 2-el\n(K-LD7-class)", "interf", 4, 0.0, "interf"),
    ("Beamforming 4-el\n(stock-demo-class)", "bartlett", 4, 0.0, "bartlett"),
    ("FBSS-MUSIC\nAOP 3-el", "music", 3, 0.0, "music"),
    ("FBSS-MUSIC\nAOP 4-el", "music", 4, 0.0, "music"),
    ("FBSS-MUSIC\nLEVM 4-el (+6 dB)", "music", 4, 6.0, "music"),
    ("FBSS-MUSIC\ncustom 6-el", "music", 6, 0.0, "music"),
    ("FBSS-MUSIC\ncustom 8-el", "music", 8, 0.0, "music"),
]


def run_config(n_el, snr_db, estimator, launches=LAUNCHES, n_trial=N_TRIAL,
               collect_frames=False, seed=1234):
    rng = np.random.default_rng(seed)
    la_err = {la: [] for la, _ in launches}
    frames = []          # (ball_h, th_err_deg, w_id) for the height plot
    gate_flags = []
    for la_deg, v in launches:
        tr = trajectory(la_deg, v)
        for _ in range(n_trial):
            cal = rng.normal(0.0, np.radians(CAL_RMS_DEG), n_el)
            th_hat = np.empty(len(tr["t"]))
            power = np.empty(len(tr["t"]))
            for i in range(len(tr["t"])):
                x, nv, w_id = snapshot(tr, i, n_el, snr_db, cal, rng)
                if estimator == "interf":
                    th = est_interferometry(x)
                elif estimator == "bartlett":
                    th = est_bartlett(x)
                else:
                    th = est_music_fbss(x, nv)
                th_hat[i] = th
                power[i] = float(np.vdot(x, x).real)
                gate_flags.append(w_id < 0.5)
                if collect_frames:
                    frames.append(
                        (tr["h"][i], np.degrees(th - tr["th_d"][i]),
                         w_id, power[i])
                    )
            r_meas = tr["r_d"] + rng.normal(0.0, RANGE_NOISE_M, len(tr["t"]))
            la_hat = fit_launch_angle(r_meas, th_hat, tr["vx"], quality=power)
            la_err[la_deg].append(la_hat - la_deg)
    per_la = {la: np.mean(np.abs(np.array(e))) for la, e in la_err.items()}
    mae = float(np.mean(np.abs(np.concatenate([la_err[la] for la, _ in launches]))))
    return dict(
        mae=mae, per_la=per_la,
        gated_frac=float(np.mean(gate_flags)),
        frames=np.array(frames) if collect_frames else None,
    )


# --------------------------------------------------------------------- main
def main():
    global H_RADAR
    print("=" * 78)
    print("STAGE-0 SIM  |  radar h=%.2f m, tee at %.1f m, net +%.1f m, "
          "Gamma=%.2f, SNR=%g dB, cal=%g deg rms, %d trials"
          % (H_RADAR, X_TEE, X_NET, GAMMA, SNR_DB, CAL_RMS_DEG, N_TRIAL))
    print("=" * 78)

    results = {}
    for idx, (label, fam, n_el, bonus, est) in enumerate(CONFIGS):
        res = run_config(
            n_el, SNR_DB + bonus, est,
            collect_frames=(label.startswith("Interferometry")
                            or "AOP 4-el" in label),
            seed=1000 + idx,
        )
        results[label] = (fam, res)
        flat = label.replace("\n", " ")
        per = "  ".join("%2d deg:%5.2f" % (la, res["per_la"][la])
                        for la, _ in LAUNCHES)
        print("%-38s MAE %5.2f deg   [%s]" % (flat, res["mae"], per))

    gated = next(iter(results.values()))[1]["gated_frac"]
    print("-" * 78)
    print("Range-gating alone removes the bounce in %.0f%% of frames "
          "(w_cross < 0.5) at h_radar=%.2f m" % (100 * gated, H_RADAR))

    # credibility check: field K-LD7 signature = HIGH-SNR low-ball frames
    # biased LOW (constructive two-ray frames pull toward the image). Fade
    # frames are wild in both directions; the field pipeline only trusted
    # strong frames, so test the strong-frame population.
    fr = results["Interferometry 2-el\n(K-LD7-class)"][1]["frames"]
    low = fr[fr[:, 0] < 0.30]
    strong = low[low[:, 3] >= np.percentile(low[:, 3], 75)]
    bias = float(np.mean(strong[:, 1]))
    spread = float(np.mean(np.abs(low[:, 1])))
    print("Credibility check  - interferometry on low-ball frames: "
          "high-amplitude bias %+.2f deg (field: biased low), "
          "all-frame |err| %.1f deg (field: degrees-scale)  -> %s"
          % (bias, spread,
             "REPRODUCED" if bias < -0.5 and spread > 1.5
             else "NOT reproduced - model suspect"))

    aop4 = results["FBSS-MUSIC\nAOP 4-el"][1]["mae"]
    print("-" * 78)
    print("PRE-REGISTERED GATE: FBSS-MUSIC AOP 4-el MAE %.2f deg  vs  %.1f deg  ->  %s"
          % (aop4, PASS_GATE_DEG, "PASS" if aop4 < PASS_GATE_DEG else "FAIL"))
    print("=" * 78)

    # SNR sweep (AOP 4-el and custom 8-el)
    snrs = [15.0, 20.0, 25.0, 30.0]
    sweep = {}
    for tag, n_el in [("AOP 4-el", 4), ("custom 8-el", 8)]:
        sweep[tag] = [run_config(n_el, s, "music", n_trial=60,
                                 seed=int(9000 + s))["mae"] for s in snrs]
        print("SNR sweep  %-11s : " % tag
              + "  ".join("%gdB:%5.2f" % (s, m) for s, m in zip(snrs, sweep[tag])))

    # radar-height sweep: the enclosure question. 0.25 m = the 10-inch cap.
    print("-" * 78)
    h_saved = H_RADAR
    for h_try in (0.25, 0.45, 0.65, 0.85):
        H_RADAR = h_try
        r4 = run_config(4, SNR_DB, "music", n_trial=60, seed=777)
        r8 = run_config(8, SNR_DB, "music", n_trial=60, seed=778)
        print("Height sweep h_radar=%.2f m : AOP 4-el MAE %5.2f  |  8-el %5.2f"
              "  |  bounce gated in %2.0f%% of frames  |  8deg-launch 4-el:%5.2f"
              % (h_try, r4["mae"], r8["mae"], 100 * r4["gated_frac"],
                 r4["per_la"][8]))
    # Coleman-board proxy: 24 GHz CW illumination (OPS) -> zero sweep
    # bandwidth -> NO range gating (every bounce path fully in-band),
    # coherent 4-el array (2x BGT24AR2, shared VCO). Same estimator.
    global LAM, RANGE_RES
    lam_s, rr_s = LAM, RANGE_RES
    LAM, RANGE_RES = C / 24.125e9, 1e12
    print("-" * 78)
    for h_try in (0.25, 0.85):
        H_RADAR = h_try
        rc = run_config(4, SNR_DB, "music", n_trial=60, seed=555)
        print("Coleman proxy (24 GHz CW 4-el, no range gating) h=%.2f m : "
              "MAE %5.2f deg  | 8deg-launch %5.2f" %
              (h_try, rc["mae"], rc["per_la"][8]))
    LAM, RANGE_RES = lam_s, rr_s
    H_RADAR = h_saved

    make_plots(results, sweep, snrs)


# -------------------------------------------------------------------- plots
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRIDC = "#e8e7e4"
BLUE = "#2a78d6"     # MUSIC family
AQUA = "#1baf7a"     # beamforming
YELLOW = "#eda100"   # interferometry baseline (relief rule: direct labels)


def _style(ax):
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRIDC)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.yaxis.grid(True, color=GRIDC, lw=0.8)
    ax.set_axisbelow(True)


def make_plots(results, sweep, snrs):
    fam_color = {"interf": YELLOW, "bartlett": AQUA, "music": BLUE}

    # ---- money plot: LA MAE by config
    fig, ax = plt.subplots(figsize=(9.6, 5.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    labels = list(results.keys())
    maes = [results[k][1]["mae"] for k in labels]
    colors = [fam_color[results[k][0]] for k in labels]
    bars = ax.bar(range(len(labels)), maes, width=0.62, color=colors, zorder=3)
    for b, v in zip(bars, maes):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.06, "%.2f" % v,
                ha="center", va="bottom", fontsize=9.5, color=INK)
    ax.axhline(PASS_GATE_DEG, color=INK2, lw=1.4, ls=(0, (5, 4)), zorder=4)
    ax.text(len(labels) - 0.45, PASS_GATE_DEG + 0.07, "2.0 deg pass gate",
            ha="right", fontsize=9, color=INK2)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, fontsize=8.6, color=INK)
    ax.set_ylabel("Launch-angle MAE (deg)", color=INK, fontsize=10)
    ax.set_title(
        "Stage 0 - launch-angle error vs estimator and elevation array\n"
        "coherent floor bounce ($\\Gamma$=%.1f), radar at %.2f m, SNR %g dB, "
        "cal %g deg rms" % (GAMMA, H_RADAR, SNR_DB, CAL_RMS_DEG),
        fontsize=10.5, color=INK, loc="left")
    _style(ax)
    fig.tight_layout()
    fig.savefig("stage0_money_plot.png", facecolor=SURFACE)
    print("wrote stage0_money_plot.png")

    # ---- per-frame elevation error vs ball height + SNR sweep
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)

    bins = np.arange(0.05, 1.35, 0.10)
    for label, color, name in [
        ("Interferometry 2-el\n(K-LD7-class)", YELLOW, "interferometry (2-el)"),
        ("FBSS-MUSIC\nAOP 4-el", BLUE, "FBSS-MUSIC (AOP 4-el)"),
    ]:
        fr = results[label][1]["frames"]
        hb, err = fr[:, 0], fr[:, 1]
        med, lo, hi, ctr = [], [], [], []
        for b0, b1 in zip(bins[:-1], bins[1:]):
            m = (hb >= b0) & (hb < b1)
            if m.sum() < 25:
                continue
            q = np.percentile(err[m], [25, 50, 75])
            lo.append(q[0]); med.append(q[1]); hi.append(q[2])
            ctr.append(0.5 * (b0 + b1))
        ax1.fill_between(ctr, lo, hi, color=color, alpha=0.18, lw=0)
        ax1.plot(ctr, med, color=color, lw=2)
        ax1.text(ctr[-1] + 0.02, med[-1], name, fontsize=9, color=INK,
                 va="center")
    ax1.axhline(0, color=INK2, lw=1)
    ax1.set_xlabel("ball height (m)", color=INK, fontsize=10)
    ax1.set_ylabel("per-frame elevation error (deg)", color=INK, fontsize=10)
    ax1.set_title("Per-frame error vs ball height (median, IQR band)\n"
                  "low ball = bounce not range-gateable = the hard regime",
                  fontsize=10, color=INK, loc="left")
    ax1.set_xlim(0.05, 1.55)
    _style(ax1)

    for tag, color in [("AOP 4-el", BLUE), ("custom 8-el", "#104281")]:
        ax2.plot(snrs, sweep[tag], color=color, lw=2, marker="o", ms=6)
        ax2.text(snrs[-1] + 0.4, sweep[tag][-1], tag, fontsize=9,
                 color=INK, va="center")
    ax2.axhline(PASS_GATE_DEG, color=INK2, lw=1.4, ls=(0, (5, 4)))
    ax2.text(snrs[0], PASS_GATE_DEG + 0.06, "2.0 deg pass gate",
             fontsize=9, color=INK2)
    ax2.set_xlabel("per-element SNR (dB)", color=INK, fontsize=10)
    ax2.set_ylabel("launch-angle MAE (deg)", color=INK, fontsize=10)
    ax2.set_title("FBSS-MUSIC accuracy vs SNR", fontsize=10, color=INK,
                  loc="left")
    ax2.set_xlim(13.5, 34.5)
    _style(ax2)

    fig.tight_layout()
    fig.savefig("stage0_frame_error.png", facecolor=SURFACE)
    print("wrote stage0_frame_error.png")


if __name__ == "__main__":
    main()
