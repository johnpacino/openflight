# IWR6843 Launch Angle Radar

This branch adds TI IWR6843 support as a second radar alongside the OPS243. The
production shape is intentionally **OPS + IWR6843**, not TI-only:

- OPS243 owns the sound-triggered shot detection path and ball/club speed.
- IWR6843 owns the short-window L3 capture and vertical launch-angle estimate.
- The server and calibration tooling use the same sound edge so both devices are
  looking at the same swing.

## Hardware Setup

Current validated geometry inputs:

- `--iwr6843-tee-m`: radar antenna center to ball slant range.
- `--iwr6843-net-m`: radar antenna center to net/screen range.
- `--iwr6843-tilt-deg`: antenna-face mount tilt.
- `--iwr6843-radar-height-m`: antenna-center height above the floor.
- `--iwr6843-ball-height-m`: ball-center height above the floor.
- `--iwr6843-tx-order`: `normal`, `reversed`, or `auto`.

The IWR6843 board and OPS243 should share the same physical sound-trigger event.
The OpenFlight server uses the OPS shot timestamp to select the matching TI L3
dump, then runs LCMF-v1 with the OPS radial ball speed.

## Firmware And Config

Use the custom L3 rolling-buffer firmware and matching cfg:

- Normal TX order: `config/iwr6843_l3dump_vB.cfg`
- Reversed TX order: `config/iwr6843_l3dump_vBR.cfg`
- Array calibration: `config/iwr6843_cal_20260712.json`

The custom firmware keeps recent frames in the IWR6843 L3 RAM. When the sound
trigger fires, the Pi waits briefly for late-flight frames, freezes the ring,
and dumps it over serial. The server processes those bytes in memory during
normal operation. Raw `.l3dump` files are only written when `--debug` is used,
and the session JSONL stores the resulting file path in the `iwr6843_capture`
entry.

## Server Usage

Example launch command:

```bash
scripts/start-kiosk.sh --debug --iwr6843 \
  --iwr6843-tee-m 1.575 \
  --iwr6843-net-m 4.6 \
  --iwr6843-tilt-deg 10.4 \
  --iwr6843-radar-height-m 0.1524 \
  --iwr6843-ball-height-m 0.040 \
  --session-location home \
  --calculated-spin
```

For reversed TX order, use the reversed cfg and matching flag:

```bash
scripts/start-kiosk.sh --debug --iwr6843 \
  --iwr6843-config config/iwr6843_l3dump_vBR.cfg \
  --iwr6843-tx-order reversed \
  --iwr6843-tee-m 1.575 \
  --iwr6843-net-m 4.6 \
  --iwr6843-tilt-deg 10.4 \
  --iwr6843-radar-height-m 0.1524 \
  --iwr6843-ball-height-m 0.040
```

## OPS + TI Calibration Session

The calibration script is designed to mimic the server as closely as possible:

1. OPS waits for the production sound trigger.
2. OPS processes the rolling buffer and emits a normal `Shot`.
3. The IWR6843 runtime matches the same trigger edge to a saved L3 dump.
4. LCMF-v1 runs with the OPS ball speed.
5. The script prints per-shot angle/status and writes JSONL diagnostics.

Run it from the repo root on the Pi:

```bash
GPIOZERO_PIN_FACTORY=lgpio uv run \
  --with gpiozero \
  --with lgpio \
  python scripts/iwr6843/calibrate.py \
  --shots 20 \
  --club 7i \
  --tee-m 1.575 \
  --net-m 4.6 \
  --tilt-deg 10.4 \
  --radar-height-m 0.1524 \
  --ball-height-m 0.040
```

Outputs are written under `~/openflight_sessions/iwr6843_calibration/<timestamp>/`
unless `--outdir` is supplied. Normal calibration writes JSONL diagnostics only;
add `--debug` when you also want raw `.l3dump` files saved for offline replay.

The live output includes:

- OPS ball speed and club speed.
- TI LCMF launch angle or withheld reason.
- Track RMS and inlier count.
- Estimated ball-start range from the TI range walk.
- A radar-consistency tilt candidate.

The tilt candidate is not TrackMan truth. It is the mount tilt where the LCMF
component models agree best for that shot. It can catch large setup mistakes,
but final accuracy still depends on measured geometry and source-of-truth
validation.

For faster sessions, skip the tilt sweep:

```bash
GPIOZERO_PIN_FACTORY=lgpio uv run \
  --with gpiozero \
  --with lgpio \
  python scripts/iwr6843/calibrate.py \
  --shots 20 \
  --club 7i \
  --tee-m 1.575 \
  --net-m 4.6 \
  --tilt-deg 10.4 \
  --radar-height-m 0.1524 \
  --ball-height-m 0.040 \
  --no-tilt-sweep
```

## Operator Notes

- If the server reports `rejected_by_ball_tracker`, the TI range walk was too
  thin, noisy, slow, or net-contaminated for LCMF.
- If it reports `rejected_missing_tdm_sign`, the track was found but the two-TX
  timing sign evidence was not strong enough for a production angle.
- If GPIO is busy, another process likely still owns BCM17. Stop old
  `shot_test.py`, calibration, or kiosk processes before restarting.
- If the IWR6843 port is not found, unplug/replug the board and confirm the
  custom single-port firmware is flashed.

## Offline Replay

Use the replay script when you have saved `.l3dump` files from a debug session
and want to rerun LCMF-v1 without the Pi hardware connected. This is the safest
way to compare estimator changes because it uses the same raw TI capture and
OPS ball speed every time.

Replay a session JSONL that contains `iwr6843_capture.capture_path` entries:

```bash
uv run python scripts/iwr6843/replay.py \
  --input ~/openflight_sessions/session_20260717.jsonl \
  --tee-m 1.655 \
  --net-m 5.156 \
  --tilt-deg 10.4 \
  --radar-height-m 0.1524 \
  --ball-height-m 0.040 \
  --club 9i \
  --out replay.csv
```

Replay a single dump:

```bash
uv run python scripts/iwr6843/replay.py \
  --input ~/openflight_sessions/iwr6843/shot.l3dump \
  --ball-speed-mph 105.9 \
  --club 9i \
  --tee-m 1.655 \
  --net-m 5.156 \
  --tilt-deg 10.4 \
  --radar-height-m 0.1524 \
  --ball-height-m 0.040
```

Important replay notes:

- A session JSONL can only replay captures that were saved during `--debug`.
- A single `.l3dump` requires `--ball-speed-mph` because OPS speed is not inside
  the TI dump.
- `--tx-order auto` reads the chirp order from `--cfg`; use the reversed cfg
  when replaying reversed-TX experiments.
- Output defaults to a terminal table, or CSV/JSONL when `--out` is supplied.
