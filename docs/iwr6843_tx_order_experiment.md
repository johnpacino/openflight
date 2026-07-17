# IWR6843 TX-order experiment

## Question

Does changing which physical TX antenna fires first reduce the moving-ball
array-manifold corruption without changing the board, mount, waveform, frame
density, estimator, or calibration?

This is a causal timing experiment. Do **not** rotate the board. Normal variant
B fires TX1 then TX3; variant BR fires TX3 then TX1. Host processing restores
both captures to the same physical eight-element order before applying the
existing corner-reflector calibration.

## Fixed setup

- Board orientation: TX antennas above RX antennas, unchanged.
- Tilt: `10.40495 deg`.
- Antenna-center height: `0.1524 m`.
- Tee slant range: `1.575 m`.
- Ball height: `0.040 m` for irons.
- Club: 9-iron.
- Trigger delay: `50 ms`, matching the July 13 outdoor captures.
- TDM sign: locked `positive` for both TX orders. Do not use `auto` for the
  TrackMan headline; replay `auto` later as a diagnostic baseline.
- Keep tee, radar, net, mat, and nearby metal stationary for all four blocks.

## Shot order

Use four eight-shot blocks: **A normal, B reversed, B reversed, A normal**.
This ABBA order makes warm-up and fatigue less likely to masquerade as a TX
effect. Take all 32 swings even when a shot is poor. For every shot, record one
of `skull`, `normal`, `high`, or `mishit`, plus a short note if useful.

## Capture commands

Run each command from `~/openflight`; stop the current block with `Ctrl-C`
before starting the next one.

```bash
GPIOZERO_PIN_FACTORY=lgpio uv run --with gpiozero --with lgpio \
  python firmware/l3_dump/shot_test.py \
  --trigger-pin 17 --delay-ms 50 --tee-m 1.575 --net-m 4.6 --club 9i \
  --tilt-deg 10.40495 --radar-height-m 0.1524 --ball-height-m 0.040 \
  --cfg config/iwr6843_l3dump_vB.cfg --tx-order normal --tdm-sign positive \
  --outdir ~/openflight_sessions/txorder_20260717/A_normal
```

```bash
GPIOZERO_PIN_FACTORY=lgpio uv run --with gpiozero --with lgpio \
  python firmware/l3_dump/shot_test.py \
  --trigger-pin 17 --delay-ms 50 --tee-m 1.575 --net-m 4.6 --club 9i \
  --tilt-deg 10.40495 --radar-height-m 0.1524 --ball-height-m 0.040 \
  --cfg config/iwr6843_l3dump_vBR.cfg --tx-order reversed --tdm-sign positive \
  --outdir ~/openflight_sessions/txorder_20260717/B_reversed
```

```bash
GPIOZERO_PIN_FACTORY=lgpio uv run --with gpiozero --with lgpio \
  python firmware/l3_dump/shot_test.py \
  --trigger-pin 17 --delay-ms 50 --tee-m 1.575 --net-m 4.6 --club 9i \
  --tilt-deg 10.40495 --radar-height-m 0.1524 --ball-height-m 0.040 \
  --cfg config/iwr6843_l3dump_vBR.cfg --tx-order reversed --tdm-sign positive \
  --outdir ~/openflight_sessions/txorder_20260717/C_reversed
```

```bash
GPIOZERO_PIN_FACTORY=lgpio uv run --with gpiozero --with lgpio \
  python firmware/l3_dump/shot_test.py \
  --trigger-pin 17 --delay-ms 50 --tee-m 1.575 --net-m 4.6 --club 9i \
  --tilt-deg 10.40495 --radar-height-m 0.1524 --ball-height-m 0.040 \
  --cfg config/iwr6843_l3dump_vB.cfg --tx-order normal --tdm-sign positive \
  --outdir ~/openflight_sessions/txorder_20260717/D_normal
```

The live startup line must print the intended TX order and `TDM sign: positive`.
Stop if either value is wrong.

## Offline analysis

After the four blocks are available on the analysis machine:

```bash
uv run python scripts/analysis/iwr6843_tx_order_experiment.py \
  --normal ~/openflight_sessions/txorder_20260717/A_normal \
  --normal ~/openflight_sessions/txorder_20260717/D_normal \
  --reversed ~/openflight_sessions/txorder_20260717/B_reversed \
  --reversed ~/openflight_sessions/txorder_20260717/C_reversed \
  --tdm-sign positive \
  --output ~/openflight_sessions/txorder_20260717/tx_order_metrics.csv
```

The first run creates `tx_order_metrics_labels.csv`. Fill its `label` column,
then rerun with:

```bash
uv run python scripts/analysis/iwr6843_tx_order_experiment.py \
  --normal ~/openflight_sessions/txorder_20260717/A_normal \
  --normal ~/openflight_sessions/txorder_20260717/D_normal \
  --reversed ~/openflight_sessions/txorder_20260717/B_reversed \
  --reversed ~/openflight_sessions/txorder_20260717/C_reversed \
  --tdm-sign positive \
  --labels ~/openflight_sessions/txorder_20260717/tx_order_metrics_labels.csv \
  --output ~/openflight_sessions/txorder_20260717/tx_order_metrics.csv
```

## Decision criteria

Do not compare mean launch angle without truth. Within the same strike labels,
the more credible order should show most of the following:

- Equal or better accepted-read rate.
- Higher two-ray explained fraction and model-pass percentage.
- Lower TX-block seam phase magnitude and scatter.
- Lower early-versus-late trajectory disagreement.
- Lower disagreement among free, tee, and two-ray estimators.

A launch-angle shift by itself is not evidence of improvement. TrackMan remains
the final test of MAE and bias.

## TrackMan scoring

Pair every accepted radar dump to the matching TrackMan shot before looking at
errors. Report the following without applying a fitted angle offset:

- Coverage, raw MAE, signed bias, error standard deviation, and P90 absolute
  error for normal, reversed, and all shots.
- The same metrics for the locked `positive` replay and the legacy `auto`
  replay. Locked `positive` is the preregistered primary result.
- Gated and ungated two-ray MAE on the identical accepted-shot set. This tests
  the `-2.5 deg` rule without letting coverage changes choose the winner.
- The old `+6.35 deg` 9-iron correction only as a pre-existing transfer test;
  do not refit an offset from this session or call that raw device MAE.
