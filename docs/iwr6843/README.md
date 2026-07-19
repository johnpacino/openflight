# IWR6843 Launch Angle Radar

This branch adds TI IWR6843 support as a second radar alongside the OPS243. The
production shape is intentionally **OPS + IWR6843**, not TI-only:

- OPS243 owns the sound-triggered shot detection path and ball/club speed.
- IWR6843 owns the short-window L3 capture and vertical launch-angle estimate.
- The server and calibration tooling use the same sound edge so both devices are
  looking at the same swing.

## Hardware Setup

### USB / UART Wiring

Do not run the OPS243 and the TI IWR6843 board from the same USB hub. In the
current production-style rig the TI board uses USB for its CLI/data connection,
while the OPS243 is wired directly to the Raspberry Pi UART header for power,
ground, RX, and TX.

Recommended connection layout:

| Device | Connection | Notes |
|---|---|---|
| IWR6843 board | USB to Pi or a powered USB hub | Used for TI CLI/config and L3 dump transfer. Give it a stable power path. |
| OPS243 power | Pi 5V pin to OPS `VIN` / `5V` | Use a 5V pin, not 3.3V, unless your OPS wiring/regulator setup explicitly says otherwise. |
| OPS243 ground | Pi GND to OPS `GND` | Must share ground with the Pi serial pins. |
| OPS243 TX | Pi UART RX, GPIO15 / physical pin 10 | OPS transmits data into the Pi. |
| OPS243 RX | Pi UART TX, GPIO14 / physical pin 8 | Pi sends commands to OPS. |
| Sound trigger `GATE` | Split to OPS `HOST_INT` and Pi BCM17 / physical pin 11 | One GATE output drives both devices: OPS freezes its rolling buffer, and the Pi tells the IWR6843 to dump L3. |
| Sound trigger power | Pi 3.3V and GND to sound trigger `VCC` / `GND` | The sound trigger, OPS, and Pi must share ground. |

Pi header reference:

| Pi physical pin | BCM name | Use |
|---|---|---|
| Pin 2 or 4 | 5V | OPS power |
| Pin 6, 9, 14, 20, 25, 30, 34, or 39 | GND | OPS ground |
| Pin 8 | GPIO14 / TXD0 | Pi TX to OPS RX |
| Pin 10 | GPIO15 / RXD0 | Pi RX from OPS TX |
| Pin 11 | GPIO17 | Sound trigger `GATE` edge for IWR6843 capture |

Sound trigger splice:

```text
Sound trigger GATE
  ├── OPS243 HOST_INT
  └── Pi GPIO17 / physical pin 11

Sound trigger VCC
  └── Pi 3.3V

Sound trigger GND
  └── Pi GND, shared with OPS GND
```

You can splice the GATE wire so the same trigger output feeds both OPS `HOST_INT`
and Pi GPIO17. Do not create two separate trigger sensors; the whole point is
that OPS and TI are timestamping the same acoustic impact edge.

Enable the Pi UART before using this wiring:

```bash
sudo raspi-config
```

Then choose `Interface Options` -> `Serial Port`, disable the login shell on the
serial port, and enable the serial hardware. After reboot, the OPS UART is
usually available as:

```text
/dev/serial0
```

Use that as the OPS port when launching OpenFlight:

```bash
scripts/start-kiosk.sh --port /dev/serial0 --iwr6843 ...
```

Important electrical notes:

- Cross TX/RX: OPS `TX` goes to Pi `RX`; OPS `RX` goes to Pi `TX`.
- Do not connect 5V to any Pi GPIO signal pin.
- Sound trigger `GATE` must be safe for Pi GPIO input. The SparkFun sound
  detector used in this rig is powered from Pi 3.3V so `GATE` is 3.3V logic.
- Confirm the OPS serial pins are 3.3V TTL-level before wiring directly to the
  Pi UART. Do not connect RS-232 voltage levels to Pi GPIO.
- If either radar resets when both are active, assume power first: separate the
  USB power paths or use a powered hub for the TI board.

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
