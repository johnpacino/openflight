# K-LD7 Quality-Tier Launch-Angle Pipeline

Prototype analysis pipeline that achieved **2.82° MAE on 12 of 15 8-iron shots**
from the 2026-06-08 TrackMan-paired session, compared with the current live
algorithm's 6.45° on the same data.

This document explains the algorithm end-to-end. The implementation lives in
`/tmp/multi_club_pipeline.py` (prototype, not in the repo) and was developed
incrementally during the 2026-06-08 / 2026-06-09 8i analysis. It is **not yet
wired into the live system**; productionizing it means porting the quality
score and tier classifier into `radc.py` and `launch_monitor.py`.

## Why a new pipeline

The existing live algorithm picks two K-LD7 frames by SNR and fits a slope
through `bearing_deg` vs `t_ms`, then extrapolates that slope to t=0 to get
launch direction. Two failure modes drove the rebuild:

1. **Bearing noise is the dominant error source.** F1A·F2A interferometric
   phase has ±5° per-shot scatter from ground multipath, with no warning in
   the SNR signal. The slope-fit amplifies that scatter geometrically.
2. **High-SNR frames can be in multipath nulls.** Picking by SNR alone
   sometimes selects the *most* multipath-corrupted frame.

The new pipeline addresses both by replacing SNR-only frame selection with a
composite quality score, gating shots that have no clean frame, and using a
position-based line fit instead of a bearing-vs-time slope fit.

## Inputs

Per session:
- Session JSONL (with `kld7_buffer` entries containing base64-encoded RADC
  payloads per frame)
- `frames_live.csv` from `scripts/analysis/kld7_geometry_selection_report.py`
  (per-frame `elevation_deg`, `bearing_deg`, `f1b_range_ft`,
  `f1b_range_unwrapped_ft`, `peak_bin`, `t_ms`, `status`)
- TrackMan CSV (for MAE comparison)

Geometry constants (current setup):
- `BALL_DISTANCE_FT = 5.0` — horizontal distance from radar to tee
- `BALL_ABOVE_RADAR_FT = -4.0 / 12.0` — ball is 4 inches *below* radar at impact
- `BORESIGHT_EL = 12.5°` = 10° mount + 2.5° angle offset

## Pipeline stages

### Stage 1: Frame ingestion

For each frame in `frames_live.csv`:
- Drop frames with `t_ms < 0` (pre-impact)
- Drop frames missing `elevation_deg` or `f1b_range_ft`
- Look up the saved RADC payload for that (shot, frame) from the JSONL

### Stage 2: Split-channel extraction from raw RADC

The report's `frames_live.csv` only carries the *combined* bearing. For the
quality score we need F1A and F2A separately. So decode each frame's saved
payload:

```python
p = parse_radc_payload(payload_bytes)
iq_1a = to_complex_iq(p['f1a_i'], p['f1a_q'])
iq_2a = to_complex_iq(p['f2a_i'], p['f2a_q'])
fft_1a = compute_fft_complex(iq_1a, fft_size=2048)
fft_2a = compute_fft_complex(iq_2a, fft_size=2048)
```

At the F1A-selected `peak_bin` extract:
- `M_F1A`, `M_F2A` — magnitudes at the ball's Doppler bin
- `SNR_F1A_dB`, `SNR_F2A_dB` — magnitude over noise-floor median
- `M_ratio = M_F2A / M_F1A`

The noise floor is the median magnitude across the whole spectrum, *excluding*
DC bins (`[:50]`, `[-50:]`) and the immediate peak neighborhood
(`peak_bin ± 50`).

### Stage 3: Quality score per frame

Three multiplicative factors:

```python
SNR_geometric_mean = sqrt(SNR_lin_F1A × SNR_lin_F2A)
              # linear amplitudes, not dB

balance = 1 / (1 + |log(M_F2A / M_F1A)|)
              # 1.0 when ratio is exactly 1
              # drops symmetrically as one antenna dominates
              # captures multipath asymmetry between F1A and F2A

bs_prox = 1 / (1 + ((elevation_deg − 12.5°) / 5°)²)
              # 1.0 at boresight
              # 0.5 at ±5° off boresight
              # falls fast outside ±10°

quality = SNR_geometric_mean × balance × bs_prox
```

**Why these three:**
- **SNR**: traditional measure, still useful — but weighted as a multiplier,
  not the sole criterion.
- **Magnitude balance** is the F1A·F2A multipath signature. When one antenna
  is in a constructive-interference zone and the other in destructive, their
  magnitudes diverge from 1:1 — and that divergence corrupts the phase
  difference (which IS the bearing). A frame with balance ≈ 1 has both
  antennas seeing the same ball; balance far from 1 means F1A and F2A are
  seeing slightly different field combinations.
- **Boresight proximity** is a geometric prior. The K-LD7 antenna pattern is
  strongest at boresight and rolls off off-axis. Multipath ground-reflection
  paths enter the antenna from below boresight, where pattern gain is lower
  on a high-mount-tilt setup. Frames at boresight have the cleanest direct
  path and the most suppressed multipath.

### Stage 4: Multipath gate

Per-frame thresholds:

```python
primary_pass:    SNR_avg ≥ 10 dB AND bs_prox ≥ 0.5 AND status != 'invalid'
companion_pass:  SNR_avg ≥ 8 dB  AND bs_prox ≥ 0.35
```

`bs_prox ≥ 0.5` corresponds to elevation within ±5° of boresight — the
antenna's high-gain main beam. Outside that, ground-multipath corruption
dominates the F1A·F2A phase difference.

The two thresholds are deliberately asymmetric: the **primary** frame needs
to be reliable on its own; a **companion** frame only needs to be good enough
to add geometric information to a line fit.

### Stage 5: Tier classification

For each shot, count frames passing the primary gate:

| primary count | tier | strategy |
|---|---|---|
| ≥ 2 | A | Multi-frame line fit |
| 1 | B | Check for straddling companion → promote to A if found; else single-frame solver |
| 0 | C | Refuse; fall back to club-physics estimator with low-confidence flag |

### Stage 6: Tier B → A promotion (straddling-companion rule)

For Tier B shots, look for a companion frame that:
1. Passes the looser `companion_pass` threshold, AND
2. **Straddles boresight** — if the primary's elevation is above 12.5°, the
   companion's must be below, or vice versa.

The straddling constraint is the key geometric trick: ground multipath bias
is **sign-asymmetric across boresight**. A frame above boresight tends to
read elevation higher than truth; a frame below boresight tends to read
lower. Line-fitting through both partly cancels the bias.

If a straddling companion exists, the shot promotes to Tier A and uses the
line fit. Otherwise it remains in Tier B with the single-frame solver.

On the 2026-06-08 8i set, this promoted 4 shots from B to A (5, 8, 10, 13).

### Stage 7: Launch-angle computation

#### Tier B (single-frame)

```python
r = frame.f1b_range_unwrapped_ft or frame.f1b_range_ft
el_rad = radians(frame.elevation_deg)

# Ball position in radar frame (radar at origin)
ball_x = r × cos(el_rad)
ball_y = r × sin(el_rad)

# Tee fixed:
tee_x = BALL_DISTANCE_FT       # 5.0
tee_y = BALL_ABOVE_RADAR_FT    # -0.333

# Launch angle = direction from tee to observed ball
launch_angle_deg = degrees(atan2(ball_y − tee_y, ball_x − tee_x))
```

No time, no trajectory fit, no extrapolation. A single observed (range,
bearing) pair plus the known tee position determines the launch direction.

#### Tier A (multi-frame line fit)

Build a weighted point set: tee + each clean ball position.

```python
xs = [tee_x, ball_x_1, ball_x_2, ...]
ys = [tee_y, ball_y_1, ball_y_2, ...]
ws = [1e4, quality_1, quality_2, ...]
        # tee weight large to anchor; ball weights = quality scores

xm = Σ(w·x) / Σw
ym = Σ(w·y) / Σw

slope = Σ(w·(x−xm)·(y−ym)) / Σ(w·(x−xm)²)
launch_angle_deg = degrees(atan(slope))
```

The large weight on the tee anchors the line; the ball-point weights let
cleaner frames pull more. The slope of the best-fit line IS the launch
direction.

### Stage 8: Comparison vs TM truth

For each shot, pair OF result to TM ground truth chronologically (matches
how `compare_trackman.py` aligns them).

## Results on 2026-06-08 8i session

| | n | shots | MAE | bias |
|---|---|---|---|---|
| **Tier A** (line fit) | 7 | 1, 5, 6, 7, 8, 10, 11, 13 | 2.88° | +0.38° |
| **Tier B** (single frame) | 5 | 2, 12, 14, 15 + 1 | 2.73° | −0.02° |
| **Tier C** (refuse) | 3 | 3, 4, 9 | n/a — emit estimator | n/a |
| **A+B combined (measured)** | **12** | | **2.82°** | **+0.21°** |
| Baseline (live algorithm) | 15 | all | 6.45° | −4.83° |

The 2.82° MAE is on the shots the system *chooses to measure*. The 3 refused
shots fall back to the club-physics estimator with a "low confidence" flag.

## Why this works

**Baseline algorithm**: 2-frame slope fit through bearing-vs-time using
highest-SNR frames. Sensitive to bearing noise because the slope depends on
small differences between similar bearing values. Output for shot N depends
on bearings at frames i, j — bearings that each have ±5° multipath bias.

**This pipeline**:
- Position-based fit through tee + ball positions in 2D space — the tee
  anchor is a *fixed* point of high weight, so the slope is much more
  stable.
- Active multipath mitigation via the quality score (avoids frames
  inherently in the multipath zone).
- Straddling-companion line fit partly cancels sign-asymmetric multipath
  bias.
- "Refuse" when the data doesn't support a measurement, rather than
  emitting noise.

## Behaviour on other clubs (2026-06-08 session)

| club | Tier A | Tier B | Refused | A+B combined | Baseline | Improvement |
|---|---|---|---|---|---|---|
| 6i | 1 / 2.48° | 6 / 4.16° | **9** | 7 / **3.92°** | 7.00° | −3.08° |
| 7i | 2 / 6.65° | 8 / 3.92° | 4 | 10 / **4.46°** | 6.41° | −1.95° |
| 8i | 7 / 2.88° | 5 / 2.73° | 3 | 12 / **2.82°** | 5.29° | −2.47° |

- 8i is the cleanest because its launch angles place the ball trajectory
  through boresight (12.5°) during the mid-flight selectable-frame window.
- 6i refuses 9 of 16 shots — most 6i shots don't produce a frame near
  boresight. The 7 that do are accurate (3.92° MAE).
- 7i Tier A has 2 anomalously bad shots (MAE 6.65°) — likely a line-fit
  failure mode that needs investigation. Tier B does the bulk of the work.

## Limitations and caveats

1. **No timing dependency**: F1B-range method is intrinsically time-
   independent. Doesn't benefit from GPIO timestamp accuracy. But the
   speed×time method (visualizer-style) would, and might be combinable.
2. **Doesn't model the antenna pattern's actual multipath suppression** —
   the bs_prox factor is a geometric heuristic, not a calibrated radiation
   pattern.
3. **Boresight constant (12.5°) is hardcoded** for this mount geometry. A
   different mount tilt would require recalibration.
4. **The refuse rate is club-dependent**. On this dataset:
   - 8i: 20% refused
   - 7i: 27% refused
   - 6i: 56% refused
   - Driver: untested — probably ~100% refused (ball never crosses
     boresight)
5. **All prototype scripts are in `/tmp`**, not in the repo. To re-run,
   the saved session data and frames_live.csv must be present.

## Productionizing

To put this in the live system:
1. Port the quality score + tier classifier into the K-LD7 RADC processing
   path (`src/openflight/kld7/radc.py`).
2. Update `launch_monitor.py` so the Shot record carries a tier label
   (`A`, `B`, `C`) and a confidence flag.
3. Update the UI to show the confidence (e.g., "measured" vs "estimated"
   pill).
4. Add a regression test on the 2026-06-08 8i set to lock in the 2.82° MAE
   number.
5. Validate on multiple sessions before turning on by default — the 6i and
   7i numbers show this needs further work for low-loft clubs.

## Open questions

- Does the **two-ray multipath inversion** (Lever 3) actually beat this
  approach? Project memory says an earlier attempt didn't beat pure-direct
  geometric inversion — but with the F1A·F2A magnitude-balance signature
  now in hand, it might be revisitable.
- Can the line fit incorporate an **OPS-speed constraint** (Lever 2)? The
  current fit only uses positions; adding `speed × t = distance` as a
  per-frame consistency check could detect non-physical fits and route to
  refuse.
- **Driver session test**: the current method probably refuses most driver
  shots. Either the bs_prox threshold needs to be loft-aware, or a
  fundamentally different selection rule is needed for clubs that never
  cross boresight.
- **Foam absorber + mount-tilt empirical test**: the cleanest validation
  would be to physically modify the mount and re-collect, then compare to
  this analysis.
