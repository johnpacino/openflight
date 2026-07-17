# TrackMan validation session — 2026-07-14 — offline analysis

The analysis behind the estimator-v2 / observable-policy commits (e1eb6af,
68f7ece) and the field report's "TrackMan verdict" section. TrackMan is the
truth reference throughout; scoring is split-half holdout (per-club offsets
fit on half the shots, error measured on the untouched half).

## Data

- `session_aligned_2026-07-14.csv` — the master truth table: 102 TrackMan
  strokes (spine) aligned by timestamp to the TI radar dumps and the
  OPS/OpenFlight session logs. Built the night of the session.
- `firmware/l3_dump/trackman_session_data_7_14.json` — raw TrackMan export.
- Raw radar dumps (~101 x shot_vB_*.l3dump) live outside the repo in
  `~/openflight_sessions/tm_0714/` (too large to commit).

## Pipeline

1. `tm0714.py` — shared loaders + truth model (TM launch params + gravity
   over the radar's 1.6–4.9 m window, per-mount-block geometry), the
   TM-guided tracker (selection window only; speeds stay radar-measured),
   ungated raw snapshot extraction.
2. `extract_tm0714.py` — one pass over all dumps -> snapshot cache
   (`cache_tm0714.npz` + `.json` shot table). Everything downstream reads
   the cache. Optional argv: a numpy window name (e.g. `blackman`).
3. `analysis_tee_cosine.py` — finding 1 (tee-anchor height bug) and
   finding 2 (cosine speed projection) rescored against truth.
4. `manifold_measure.py` — THE calibration diagnosis: per-element residual
   vs TM-truth arrival angles across all three mount blocks. Result: no
   fittable static manifold; a late-track corrupted zone instead.
5. Mechanism eliminations for the corrupted zone:
   `tdm_fix_test.py` (truth-velocity TDM: partial, ~30%),
   `split_regress.py` (TX azimuth offset: dead),
   `window_compare.py` (range-FFT windowing / net sidelobes: dead),
   `diag_image.py` + `third_source.py` (image-path / extra arrival:
   corruption peaks in the direct-image interference regime).
6. `est_v2.py` — the estimator matrix with quality gates, scored per club
   on holdout. The winning configs shipped as `shot.POLICY_*`.

## Headline result (block A, production mount, 60 truth shots)

Driver 1.17 / SW 1.27 / 9i 2.59 / 7i 3.26 / 5i 3.44 deg holdout MAE,
pooled 2.58 deg at 85% coverage; ball speed −2.4 ± 1.7 mph after the
cosine correction. Baseline before this work: ~4.6 deg at 80%.

Run any script with `uv run python <script>` from this directory (matplotlib
needed for the plotting ones: `uv run --with matplotlib ...`).

## Two-ray v2: joint whole-shot fit (2026-07-15 — NEGATIVE result, informative)

`tworay_v2.py` — parameterizes one trajectory (LA through the known tee,
gravity from measured speed) and fits ALL snapshots jointly (per-snapshot
amplitudes closed-form). Six variants scored; ALL lose to v1's
per-snapshot-heights + weighted line (best v2 = 3.21 vs v1 2.58 pooled,
even on identical snapshot pools). Interpretation: the binding constraint
is FORWARD-MODEL fidelity, not estimator efficiency — v1's per-snapshot
height freedom absorbs the systematic two-source model error (corrupted
zone, low-flight suppression) that the joint ML fit faithfully converts
into launch-angle error. RE-TRY v2 after the wide-angle calibration /
corruption root-cause lands: with a correct model the ordering should flip.

## Frame-stage audit: fixed-positive sign (2026-07-16)

`frame_stage_audit.py` compares anchored fits from the first, middle, and last
three chronological valid frames of every Block-A 9i and 7i. It converts the
historical auto-sign cache to fixed-positive and scores both range-walk local
velocity and TrackMan truth-velocity TDM correction.

On shots with TrackMan launch angle at least 10 degrees, truth-velocity errors
progressed from `-6.12` to `-4.20 deg` for 9i and from `-6.69` to `-4.87 deg`
for 7i. The two-ray height solve stayed approximately `10 cm` below TrackMan's
predicted height in every frame group. Exact horizontal range did not change
the result, and truth velocity recovered only about `0.5 deg`.

The original three-position reflector calibration remains accurate to about
`0.15 deg`; applying the moving-ball truth residual shifts those static angles
about `+2 deg` and makes them wrong. The evidence therefore favors a
moving-ball/environment-dependent manifold or multipath-model term rather than
a global board calibration error, a simple early-frame-only failure, or TDM
speed error alone. The generated `frame_stage_audit.csv` contains per-shot
frame-group evidence.

## Multipath and mount-geometry audit (2026-07-16)

The new audits deliberately avoid per-club offsets and separate mechanism
tests from deployable inputs:

- `multipath_channel_audit.py` compares each TX independently and scores
  two-, three-, and four-path physical MIMO dictionaries with leave-one-channel-
  out PRESS error. The production-input run uses OPS ball speed and TI
  range-walk velocity; TrackMan is used only after estimation to score error.
- `range_path_audit.py` measures complex-range power asymmetry at the predicted
  DG/GD and GG floor-path delays with rectangular and Hann windows.
- `fast_time_multipath_audit.py` jointly fits calibrated antenna phase and the
  neighboring complex range bins. The useful variant uses a Hann window,
  centers on the observed local peak, and uses the chronological second half
  of the track.
- `block_geometry_audit.py` uses exact permutation tests, bootstrap intervals,
  and 9-iron matching on TrackMan speed and launch to compare Blocks A/B/C.
- `ops_guide_audit.py` verifies that OPS speed can guide TI track selection
  without TrackMan. It found a track on 101/101 shots and reproduced 68/69
  strict-pool tracks; the exception had a similar slope but a shorter tail.
- `honest_offset_scorecard.py` demonstrates that the old 2.58-degree headline
  is not a no-offset expectation. The best old single estimator is 3.50-degree
  raw MAE; a global cross-fitted offset is 3.53 degrees and leave-one-club-out
  transfer is 3.93 degrees.

The radar reports half the round-trip propagation distance. Therefore a
one-leg floor path has apparent-range excess `(image-direct)/2`, and the GG
path has excess `image-direct`. For 9-irons, median GG separation was 4.73 cm
in A, 13.86 cm in B, and 6.71 cm in C. With the production-input four-path
channel model, raw 9-iron MAE was 2.99, 1.64, and 3.50 degrees respectively.
The A-vs-B exact permutation p-value was 0.040, but this is post-hoc and B has
only six strict shots.

This supports a useful high-mount regime, but does not isolate height by
itself: radar height, tilt, tee distance, and shot population all changed.
The fact that B improved despite smaller direct/image angular separation,
while its GG delay grew to about three native range bins, favors range-path
separability over a simple boresight or angular-separation explanation. C did
not improve like B, so the effect is threshold-like rather than "higher is
always linearly better."

The power-only range audit did not reveal a universal far-side hump. B was
nearly symmetric and C had a strong near-side skew under both FFT windows.
Coherent destructive interference can cause that pattern, so it does not rule
out multipath, but it does rule out using a positive range shoulder as a
standalone detector.

## Frozen exploratory candidate

Late-Flight Complex Multipath Fusion v1 (`LCMF-v1`) equally averages five
truth-independent estimates through `hybrid_estimator_audit.py`:
the channel-only direct+GG and full four-path models, plus the late-half
fast-time direct, direct+GG, and four-path models. It uses OPS ball speed and
TI local radial velocity. No component weight is fitted.

On 53 strict Block-A shots, raw MAE/bias is 2.94/-2.84 degrees. A single
device-wide correction, never a per-club correction, gives:

- 1.19-degree alternating-fold global-offset MAE.
- 0.93-1.48-degree bootstrap 95% interval for that MAE.
- 1.17-1.33-degree 5th-95th percentile over 5,000 random split-half scores.
- 1.19-degree leave-one-club-out MAE; held-club calibration constants span
  only 2.71-2.94 degrees.
- 1.17-degree Block-B and 1.27-degree Block-C MAE when the Block-A correction
  is applied unchanged.

The first half of flight is substantially worse than the second half, which
supports late-flight gating as a mechanism choice rather than a cosmetic
filter. The frozen specification and `+2.8387689102` degree correction are in
`frozen_candidate_20260716.json`. Do not refit it on the next TrackMan data.

These remain development results because model families were explored on the
same session. The next session is the first valid independent performance
test. Pre-register raw MAE, frozen-correction MAE, bias, 95th-percentile error,
and coverage. Do not report a newly fitted offset as the primary result.

## Next controlled session

Use one ball type and one unchanged surface. Record measured antenna-center
height, horizontal tee distance, tilt, TX order, and ball height for every
block. The highest-value 2x2 test is low vs high mount crossed with normal vs
reversed TX order, using 9-iron shots in balanced ABBA blocks so warm-up and
fatigue do not line up with one condition. Reproduce Block A and Block B
geometry if practical; Block B is the only observed regime with roughly
three-bin GG separation.

Use at least 12 accepted 9-irons per cell (48 total), then add untouched club
transfer shots if time permits. Score the frozen candidate first, with no
session, block, club, or player adjustment. Treat fitted corrections and
threshold changes as secondary development analyses for a later session.
