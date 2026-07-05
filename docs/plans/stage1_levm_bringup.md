# Stage 1 — IWR6843LEVM Bring-Up & Evaluation Plan

**Board:** TI IWR6843LEVM (ordered 2026-07-02 from TI direct, ETA Tue night)
**Reference:** SWRU585 user guide (verified page-by-page pre-purchase)
**Prereq reading:** `docs/plans/custom_radar_board_requirements.md` (decision gates G1–G5),
`music_stage0_sim.py` (Stage-0 simulation + pre-registered criteria)

> **INTENT (for future sessions / Opus 4.8):** This doc is the day-one runbook.
> The goal of Stage 1 is to answer ONE question: *does an 8-virtual-element
> vertical array (rotated LEVM) beat 2.0° launch-angle MAE at the 10-inch
> mount height, with the floor bounce present?* Everything here serves that.
> Software prep (parser, dry-run, configs) is already written and validated
> against synthetic data — see `iwr6843_uart.py`, `stage1_dryrun.py`,
> `config/iwr6843_levm_*.cfg`. Hardware sessions should start at §4.

---

## 1. Pre-registered pass/fail gates (do not move these after data exists)

| Gate | Test | PASS threshold |
|------|------|----------------|
| S1-A | Static corner reflector, stock firmware, rotated board | Recovered elevation within ±0.5° of tape-measured truth, repeatable over ≥5 re-mounts |
| S1-B | Stock demo on real shots | Ball detected on ≥95% of shots ≥35 mph; plausible speed vs OPS same-shot |
| S1-C | Height sweep 10/18/30 in, real shots, our FBSS-MUSIC | LA MAE < 2.0° at 10 in (G1) or identify which height passes (G2) |
| S1-D | Per-frame SNR on ball at 2–5 m | ≥20 dB post-2D-FFT (Stage-0 sim assumption — verify it) |

Stage-0 sim predictions to compare against: 8-el @ 0.25 m → 1.45° MAE;
4-el @ 0.25 m → 4.0° (fails); bounce range-gated in 43% of frames at 0.25 m,
72% at 0.45 m. If reality lands far from these, the *model* needs updating
before conclusions are drawn.

## 2. Hardware setup

### 2.1 S1 DIP switch settings (SWRU585 Table 5-5 / 5-6)

| Mode | S1.1 | S1.2 | S1.3 | S1.4 | S1.5 |
|------|------|------|------|------|------|
| **Flashing** | On | Off | On | On | Off |
| **Functional** (normal use) | Off | Off | On | On | Off |
| DCA1000 raw capture (deferred) | Off | On | On | Off | Off |

Press the reset switch (S2) after every mode change and after power-up
(TI recommends one NRST press for reliable boot).

### 2.2 USB / serial

- Micro-USB J5 (cable in box) = power + dual CP2105 USB-UART.
- Two virtual COM ports appear: **Enhanced** = config/CLI UART (115200 baud),
  **Standard** = data UART (921600 baud).
- macOS: if `ls /dev/tty.usbserial*` shows nothing, install the Silicon Labs
  CP210x VCP driver. Linux/Pi: cp210x is in-kernel, no driver needed —
  ports appear as `/dev/ttyUSB0` (Enhanced) and `/dev/ttyUSB1` (Standard).
- Power: single USB port is fine for TDM-MIMO (one TX at a time). Use a
  solid 5 V source; if flaky, powered hub.

### 2.3 Flashing (UniFlash, works on macOS)

1. S1 → Flashing mode, press reset.
2. UniFlash → IWR6843 → serial connect on the **Enhanced** port.
3. Load the demo image: `xwr68xx_mmw_demo.bin` from mmWave SDK 3.x LTS
   (`packages/ti/demo/xwr68xx/mmw/`).
4. S1 → Functional mode, press reset.

### 2.4 Toolchain note (Stage 1c only — not needed day one)

Firmware *building* needs x86 Linux or Windows. John has **Parallels** —
note: on Apple Silicon, use a **Windows 11 ARM VM** (its x64 emulation runs
TI CCS/SDK installers; an ARM Linux VM will NOT run the x86 SDK). CrossOver
may work for UniFlash but is unproven for CCS — prefer the Parallels VM.
Set the VM + SDK download up before Stage 1c, not during.

### 2.5 Rotated mount (the whole point)

- Mount the board **rotated 90°** so the RX1–RX4 line + TX1/TX3 axis is
  **VERTICAL**. In this orientation the demo's "azimuth" axis measures
  **elevation/launch angle** and its "elevation" axis measures aim.
- Antenna end of the board UP (maximizes phase-center height).
- Boresight pointed downrange, level to ~3° up. Measure the tilt with a
  digital level and RECORD it in the session metadata — every unmeasured
  degree of tilt is a degree of launch-angle bias (the K-LD7 +2° lesson).
- Nothing (metal, hands, cables) in front of the antenna region. ≥20 cm
  from people during operation (EN 62311 note in SWRU585).
- Storage: keep the board in its ESD bag between sessions (immersion-silver
  antenna finish oxidizes/blackens in open air — cosmetic but avoidable).

## 3. Bench geometry (record BEFORE first capture)

Tape-measure and write into `session_meta.json` (template in §6):
tee-to-radar horizontal distance, radar phase-center height (each sweep
position: 0.25 / 0.45 / 0.75 m), board tilt (digital level), tee-to-net
distance, floor material. The corner-reflector positions for S1-A need
distance + height measured to ±1 cm (that IS the ground truth).

## 4. Day-one protocol (stock firmware, zero C code)

1. Flash demo (§2.3). Functional mode. Note which /dev port is which.
2. Send `config/iwr6843_levm_static.cfg` line-by-line to the CLI port
   (the parser module's `send_config()` does this), or use TI's browser
   Demo Visualizer for a first smoke test.
3. **S1-A static test:** corner reflector at measured (range, height)
   positions spanning −10°…+20° elevation from the radar. Capture the
   **azimuth static heatmap TLV** (= per-antenna complex data; on the
   rotated board this axis is elevation). Run `stage1_static_analysis`
   (see `stage1_dryrun.py` — same code path, real bytes instead of
   synthetic). Compare recovered angle vs tape-measure truth → Gate S1-A.
4. **S1-B live shots:** switch to `config/iwr6843_levm_ball.cfg`, hit balls,
   confirm detection + speed vs OPS (both radars can run simultaneously —
   24 vs 60 GHz, no interference).
5. **S1-C height sweep:** same shots at 10 / 18 / 30 in mount heights.
   This dataset decides Variant A vs Variant B in the board requirements doc.

## 5. Known constraints (learned in prep — don't rediscover)

- **Stock firmware cannot emit per-antenna data for MOVING targets** over
  UART. The static azimuth heatmap TLV (type 4) is zero-Doppler only —
  perfect for S1-A, useless for ball flight. Moving-target per-antenna data
  requires the Stage-1c custom TLV (C firmware mod) or DCA1000 raw capture.
  Plan S1-C analysis accordingly: day-one ball data gives detection/speed
  (S1-B) and TI's own angle estimate as a baseline — NOT our MUSIC numbers.
- **UART bandwidth budgets the heatmap TLV.** 256 range bins × 8 virt-ant ×
  4 B ≈ 8 KB/frame. At 921600 baud (~90 KB/s) that caps ~10 fps with the
  heatmap enabled. Fine for static tests; the ball config disables it.
- **TLV `length` field convention** (SDK 3.x: payload-only, excludes the
  8-byte TLV header) is what the parser assumes — `stage1_dryrun.py` proves
  self-consistency, but VERIFY against first real frame (parser will log a
  warning if `totalPacketLen` doesn't reconcile).
- Virtual-antenna ordering in the heatmap TLV is assumed RX-major per TX
  (TX1:RX1-4 then TX3:RX1-4 → 8 contiguous λ/2 positions). Verify with the
  S1-A phase-ramp check: a reflector off boresight must produce a *linear*
  phase ramp across the 8 antennas. If the ramp has jumps, the ordering
  assumption is wrong — fix `VIRT_ANT_ORDER` in `iwr6843_uart.py`.

## 6. Session data conventions

```
~/openflight_sessions/stage1/
  2026-07-0X_static_h250/          # one dir per (test, height)
    session_meta.json              # geometry ground truth (template below)
    raw_uart.bin                   # verbatim data-port byte stream
    frames.parquet|npz             # parsed output (parser writes this)
    notes.md                       # anything weird, in the moment
```

`session_meta.json` template:

```json
{
  "date": "", "test": "static|ball", "firmware": "oob-demo-3.x",
  "cfg_file": "iwr6843_levm_static.cfg",
  "radar_height_m": 0.25, "board_tilt_deg": 0.0, "tilt_method": "digital level",
  "tee_distance_m": 2.00, "net_distance_m": 3.00, "floor": "carpet|concrete",
  "rotated_90deg": true, "antenna_end_up": true,
  "reflector_positions": [{"range_m": 3.00, "height_m": 0.50}],
  "ops_rig_running": true
}
```

Rule: **raw bytes are always saved.** Parsing bugs are fixable later only
if the raw stream exists.
