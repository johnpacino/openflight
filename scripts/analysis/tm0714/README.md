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
