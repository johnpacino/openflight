# Experimental Vertical Recovery

This branch banks an offline-only second pass for IWR6843 captures rejected by
the production ball tracker. It does not replace an accepted LCMF-v1 result and
is not connected to the live OpenFlight UI.

## Approach

The batch replay first learns robust medians and median absolute deviations
from accepted captures for three truth-free observables:

- TI range-track speed divided by OPS ball speed
- ball-derived impact time
- range-track duration

For a rejected capture, it generates multiple burst- and window-MTI range
walks. It rejects thin, short, ragged, or session-inconsistent tracks, ranks
the survivors, and runs the selected track through the unchanged LCMF spatial
models. A recovery is retained only when both conditioned vertical channels
contribute and at least four frames support the angle.

TrackMan launch angle is never passed to candidate generation or ranking.

## 53-Bin Holdout, 2026-08-11

The replay corpus contained 79 TrackMan-aligned 53-bin captures:

| Dataset | Baseline coverage | Baseline MAE | Recovery coverage | Recovery MAE |
|---|---:|---:|---:|---:|
| July 22 | 19/20 | 1.545 degrees | 20/20 | 1.522 degrees |
| August 9 wide IQ16 | 59/59 | 0.781 degrees | unchanged | unchanged |
| Combined | 78/79 | 0.967 degrees | 79/79 | 0.969 degrees |

The deterministic selector recovered the one naturally rejected capture at
21.702 degrees against TrackMan's 20.613 degrees, an absolute error of 1.089
degrees. Its selected track used 23 inliers across seven frames with 0.248-bin
RMS. An earlier stochastic prototype found a nearby 0.990-degree candidate;
that result is not claimed here because it changed with the random seed.

## Why This Is Not Production Yet

- Only one naturally rejected 53-bin TrackMan capture was available.
- The batch prior used accepted shots from the complete session. An early live
  rejection may occur before enough accepted shots exist to learn that prior.
- When forced to rediscover already accepted tracks, the selector changed
  August 9 MAE from 0.781 to 0.814 degrees and produced one 4.61-degree
  disagreement with the production track.
- Relaxing only the existing quality gate was unsafe: it produced 25.0 degrees
  on the rejected shot, 4.35 degrees above TrackMan truth.

Treat recovered values as research diagnostics or a future low-confidence UI
read until another independent truth session contains substantially more
naturally rejected shots.

## Offline Usage

```bash
uv run python scripts/analysis/replay_iwr6843_low_confidence.py \
  ~/openflight_sessions/session_a.jsonl \
  ~/openflight_sessions/session_b.jsonl \
  --tee-m 1.626 \
  --net-m 5.131 \
  --tilt-deg 12.1 \
  --radar-height-m 0.1524 \
  --club 9i \
  --out ~/openflight_sessions/iwr6843_recovery.jsonl
```
