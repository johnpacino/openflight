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

## 0. Hardware Day 1 findings (2026-07-07) — READ FIRST

Board arrived pre-flashed with the stock mmw demo (SDK **3.5.0.4**, platform
`0xa6843`) — no flashing needed. Bring-up on the **Pi** (macOS CP210x driver is
blocked by the corporate MDM; the Pi's in-kernel `cp210x` sidesteps it). Ports:
`/dev/ttyUSB0` = Enhanced/CLI (115200), `/dev/ttyUSB1` = Standard/data (921600).

**What works now:** config → `sensorStart` → continuous azimuth-heatmap stream
at 5 fps, zero frame drops. The `iwr6843_uart.py` parser is validated on real
bytes: **A1 (TLV length = payload only) confirmed on 74/74 frames**, heatmap TLV
= 8192 B = 256 bins × 8 ant × 4 B, all 8 virtual antennas live, FBSS-MUSIC runs
clean end-to-end. Capture/analyze split: **Pi captures raw bytes, Mac analyzes**.

**THE big gotcha — dedicated power.** The whole first night was lost to a
"sensor starts but streams zero bytes" mystery. Root cause was **power
starvation**: the LEVM shared a USB hub with the OPS243, and the 5 V rail sagged
under RF/DSP load. Signature: `sensorStart` returns `Done`, CLI answers
`version`, 5th LED on — but **zero frames on either UART, regardless of frame
size** (ruled out bandwidth, DTR/RTS, and port mapping first). Fix: give the
board its **own** USB port (direct-to-Pi), not a shared hub. Identical config
then streamed immediately. *Never share USB power between the LEVM and the OPS.*
> **OPEN ITEM (2026-07-08) — dual-radar USB power:** running the LEVM **and**
> OPS on the Pi 5's USB simultaneously trips **USB over-current** warnings
> (`over-current change` on all root hubs; on-screen notice) even though the
> 5 V rail held (`throttled=0x0`, `EXT5V_V`≈5.14 V steady, `usb_max_current_enable=1`
> already set). So the rail didn't sag, but the Pi is current-limiting USB with
> both attached — we're at/over the USB budget. RESOLVE before dual-radar ball
> testing: (1) confirm a genuine 27 W/5 A Pi-5 PSU, or (2) put the OPS on a
> powered USB hub (sure fix). Not a product concern — the custom board uses
> GPIO UART + its own 5 V rail.
> **PLANNED FIX (hardware on hand):** the X1206 UPS HAT has 2× USB-A
> **power-output** sockets (power only, no data). So: **OPS power → X1206 USB**,
> **OPS data → Pi GPIO UART** (OPS243 J3 3.3 V TTL: TX→GPIO15/pin10,
> RX→GPIO14/pin8, shared GND — same J3 header as the HOST_INT trigger); **TI
> stays on the Pi's USB.** OPS current then flows from the UPS, not the Pi's USB
> controller, so the Pi's USB budget feeds only the TI → no over-current.
> Pi side: `raspi-config` enable serial hw / disable login shell; `ops243.py`
> opens `/dev/serial0` instead of USB (same API over UART, match baud). Verify
> OPS243 J3 UART pins + baud against its datasheet. This also moves the OPS onto
> the same GPIO-UART path the future custom TI board will use. Retest pending.

**Config fixes needed for SDK 3.5** (now baked into `config/*.cfg`):
- Ramp peak must stay ≤ 64 GHz. Draft `start 60.25 + slope 62.5 × 65 µs ramp`
  peaked 64.3 GHz → `sensorStart` **Error -1**. Verified static profile:
  `profileCfg 0 60 30 6 60 0 0 60 1 256 5000 0 0 30` (peaks 63.6 GHz).
- `bpmCfg -1 0 0 0` is **mandatory** even when unused, else `sensorStart` →
  "Full configuration must be provided".
- `compRangeBiasAndRxChanPhase` needs the full **25-arg** (12-virtual-antenna)
  form; the 17-arg (8-ant) form is rejected "Invalid usage".
- Heatmap TLV ≈ 9 KB/frame → keep static cfg at **5 fps** (200 ms); 10 fps
  overruns the 921600-baud UART.

**Range resolution corrected:** verified slope 60 MHz/µs → **4.88 cm/bin**
(not the 4.68 cm the draft slope 62.5 implied). `RANGE_RES` and
`stage1_analyze` default updated; put `range_res_m: 0.0488` in `session_meta`.

**Observation to chase in S1-A:** the uncalibrated per-antenna phase shows a
~54° step at the **TX1→TX3 seam** (antennas 3→4); within each 4-element TX
sub-array the ramp is linear. That seam offset is exactly what
`measureRangeBiasAndRxChanPhase` (step 2) must remove before angles are trusted.

**Fixed since:** the `stage1_capture.py` hang (commit 48de555) — `frames()` had
no wall-clock budget so a silent/stalled stream span forever; `send_config` now
uses a bounded read window. Also added `--overwrite` so re-runs don't append.

### Static multipath height sweep (2026-07-07) — the Stage-1a result

Rotated board, corner reflector at radar co-height (truth ≈ 0° elevation),
82 in range, 100 frames/height, FBSS-MUSIC range-gated to the reflector bin,
calibrated to the clean 37.5" baseline. Elevation deviation from baseline
(= multipath bias; ~1° of it is hand-remount slop):

| Mount | 37.5" | 24" | 18" | 14" | 10" | 6" | 4" |
|-------|-------|-----|-----|-----|-----|----|----|
| bias  | 0 (ref) | −0.7° | −1.3° | −1.1° | −1.3° | −0.7° | −3.2° |

Within-capture spread ≈ 0° everywhere (static target). **Bias ≤1.3° across the
whole 37.5"→6" range; only the sub-spec 4" mount breaks (−3.2°) where the floor
bounce fully merges sub-bin.** A3 phase-ramp residual is non-monotonic (peaks
~21° at 6" partial-merge, drops to ~7° at 4" full-merge) — a linear-ramp fit is
NOT a reliable multipath gauge; the recovered MUSIC angle is.

**Decisions from this sweep:**
- **Target mount = 6"** (launch-monitor enclosure constraint; 10" needs a
  redesign). 10" retained as a rigid 4"-riser A/B. 6" is only ~2" above the 4"
  cliff — watch hitting-mat/floor thickness in practice.
- **Mount ~10° up** (beam off floor → less bounce; centers typical launch
  angles; matches OPS enclosure) and ALWAYS record the tilt (K-LD7 +2° lesson;
  `true_LA = measured + mount_tilt`). Optimal tilt is a Stage-1c tuning knob.
- Uncalibrated per-antenna phase shows a ~+38° TX1→TX3 seam offset; calibrate
  (`measureRangeBiasAndRxChanPhase` or a Python per-antenna vector) first.
- Range-gate the analysis to the reflector bin — global-argmax gets hijacked by
  near-field floor clutter at low mounts (the 10" first-try contamination).

**Caveat:** a strong STATIC reflector is a best-case proxy. The moving-ball
answer (weaker target, rising from the tee, hard low-ball early frames) still
needs the **Stage-1c custom TLV**. This sweep is the strongest justification to
build it, and the 6"-vs-10"-riser A/B becomes decision-grade once a real ball
can run through it.

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
  Boxed cable is likely micro-B -> USB-A: an M1 Mac needs a USB-A->C
  adapter or a micro-B->USB-C cable. **Must be a DATA cable** — a
  charge-only cable powers the LEDs but no serial ports appear (looks
  exactly like a driver problem; swap cables before debugging drivers).
- Two virtual COM ports appear: **Enhanced** = config/CLI UART (115200 baud),
  **Standard** = data UART (921600 baud).
- macOS: if `ls /dev/tty.usbserial*` shows nothing, install the Silicon Labs
  CP210x VCP driver. Linux/Pi: cp210x is in-kernel, no driver needed —
  ports appear as `/dev/ttyUSB0` (Enhanced) and `/dev/ttyUSB1` (Standard).
- Power: single USB port is fine for TDM-MIMO (one TX at a time). Use a
  solid 5 V source; if flaky, powered hub.

### 2.3 Flashing (UniFlash)

UniFlash options: **macOS desktop build** (preferred — offline, reliable
serial), **browser flash at dev.ti.com** (installs a local TI Cloud Agent
helper for port access), or Linux **x86** desktop (Parallels VM / PC — not
the Pi). Flashing is macOS-friendly; only firmware *building* (Stage 1c)
needs the x86 toolchain. The demo binary itself comes from the mmWave SDK
download either way.

1. S1 → Flashing mode, press reset.
2. UniFlash → IWR6843 → serial connect on the **Enhanced** port.
3. Load the demo image: `xwr68xx_mmw_demo.bin` from mmWave SDK 3.x LTS
   (`packages/ti/demo/xwr68xx/mmw/`).
4. S1 → Functional mode, press reset.

### 2.4 Toolchain note (Stage 1c only — not needed day one)

Firmware *building* needs an x86 toolchain (the SDK build chain is x86, and
some TI components are 32-bit). On John's M1:
- **Windows 11 ARM VM in Parallels = the safe path** (Windows emulates both
  x64 and 32-bit x86; runs Windows CCS/SDK).
- ARM Linux VM + Rosetta translates x86-64 Linux binaries only — any 32-bit
  i386 component in TI's chain breaks it. Coin flip; don't depend on it.
- Docker --platform linux/amd64 (QEMU) works in principle, slowest.
Set this up before Stage 1c, not during.

**Day one does NOT need the SDK installed anywhere:** the prebuilt
`xwr68xx_mmw_demo.bin` can be downloaded directly from TI Resource Explorer
in a browser (dev.ti.com → mmWave SDK / Radar Toolbox package). Flashing
requires only that .bin + UniFlash (macOS desktop or browser, §2.3).

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
2. Identify ports (`uv run python stage1_capture.py --list-ports`), then
   capture with config + first-frame verification in one command:
   ```
   uv run python stage1_capture.py --cli <port> --data <port> \
       --cfg config/iwr6843_levm_static.cfg \
       --session ~/openflight_sessions/stage1/<date>_static_h250 \
       --seconds 30 --verify 10
   ```
   The `--verify` output runs the A1/A3 checks from §5 automatically.
   (TI's browser Demo Visualizer also works for a first smoke test.)
3. **S1-A static test:** corner reflector at measured (range, height)
   positions spanning −10°…+20° elevation from the radar. Fill the
   geometry fields in the generated `session_meta.json`, then:
   ```
   uv run --extra analysis python stage1_analyze.py --session <dir>
   ```
   prints the per-frame angle table, median vs tape-measure truth, and the
   **Gate S1-A PASS/FAIL verdict** directly. (Both tools are end-to-end
   tested against synthetic sessions — `tests/test_stage1_tools.py`.)
4. **S1-B live shots:** switch to `config/iwr6843_levm_ball.cfg`, hit balls,
   confirm detection + speed vs OPS (both radars can run simultaneously —
   24 vs 60 GHz, no interference).
   - **Keep the OPS + sound trigger running: it is the labeling system.**
     Record the trigger timestamp per shot (the OPS session log already has
     it). Each timestamp marks "impact here" in the LEVM stream, giving
     labeled windows to characterize the real ball signature (points/frame,
     SNR, range-walk shape, clutter) — the future software detector gets
     built from these labels, no ML needed. Alignment only needs to be
     window-accurate: known cross-device jitter is ~±60 ms = ±7 frames,
     and the range-walk pattern within ±100 ms of impact is unambiguous.
     No detection code is needed for Stage 1 itself.
   - **Run sequence (two processes, same Pi):** `ball_capture.py` is the
     device-under-test capture; OpenFlight is the co-running label/reference:
     1. OPS on GPIO UART + TI (LEVM) on USB; `get_throttled=0x0` — the §0
        dual-radar power fix must be in place first.
     2. Start **OpenFlight** (OPS + sound trigger + session logging; no UI/KLD7
        needed) → logs shot speed + `trigger_event` timestamps = the labels.
     3. Start **`ball_capture.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1
        --session .../<date>_ball`** → `raw_uart.bin` + `frames_index.csv`
        (per-frame Unix-epoch timestamps). Ctrl-C to stop.
     4. Hit shots. Pull BOTH to the Mac; align each OpenFlight trigger time to
        `frames_index.csv` rows (shared Pi clock, ~±60 ms window), then
        characterize the ball/club signature and compare detection/speed vs OPS.
   - **Gating to-dos before a real S1-B session:** (a) dual-radar power fix
     (§0); (b) **verify the ball cfg actually streams** — it has never been run
     on hardware (only the static cfg has); (c) EDIT geometry +
     `openflight_session_log` in the generated `session_meta.json`.
   - **Step-0 pre-flight smoke test (clap + hand-wave)** — run this indoors
     first; it proves the whole chain end-to-end in ~2 min (plumbing, NOT
     detection quality). With OpenFlight + `ball_capture.py` both running:
     1. **Clap** near the SEN-14262 → OpenFlight logs a `trigger_event`
        (validates trigger chain + OPS-over-GPIO-UART).
     2. **Wave a hand back-and-forth** through the beam → OPS records a speed
        (a labeled event) AND `frames_index.csv` shows `n_det > 0` at that time
        (both radars alive + TI streaming + timestamped).
     3. Ctrl-C; confirm both logs have entries at matching wall-clock times
        (alignment works) and `get_throttled=0x0` held (dual-radar power OK).
     PASS = trigger logged + OPS speed + TI frames, timestamps aligned, power
     steady → then go outside for real balls. No-ball trigger *rejects* are
     expected; this tests the plumbing, not object detection.
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

## 5.5 Ball detection & launch-angle recovery — clutter separation (Stage-1c design)

The moving ball must be separated from static clutter (floor, walls, mat, tee)
before its per-antenna snapshot feeds MUSIC. The *simplest* separation is
Doppler — the ball sits at non-zero Doppler, all static clutter at zero — and
it works for the vast majority of shots. **But it fails in the Doppler
wrap-to-zero blind bands**, and the detector must NOT rely on Doppler alone.

**The blind bands (angle problem, not speed):** with FMCW-MIMO the max
unambiguous velocity is `v_max = λ/(4·N_TX·Tc)` (~12 m/s for the 2-TX ball
draft). Ball speeds near `N·2·v_max` (~54 / 107 / 161 mph, ~5–10 mph wide,
wider with fewer chirps/frame) alias to ~0 Doppler → the ball collapses onto
the static-clutter cell → MUSIC sees ball+clutter → **launch angle is
corrupted.** Speed is unaffected (OPS243 CW-Doppler has no such band; range-walk
is alias-immune). This is inherent to *all* FMCW-MIMO parts, not this chip.

**Recovery — build the detector around these, in our Python pipeline (keep the
demo's `clutterRemoval` OFF so the zero-Doppler cell is never zeroed):**

1. **Static-clutter model + subtraction (primary).** Capture a short no-ball
   reference (N frames of the empty scene) per session, model the static
   range–antenna field, and subtract it. A zero-Doppler ball then stands out
   even when its Doppler has wrapped — separation no longer depends on Doppler.
2. **Range-walk gating.** The ball's range-vs-frame trajectory is unambiguous
   even when Doppler wraps; use it to pick the ball's range bin and pull the
   per-antenna snapshot *there*, rather than trusting the Doppler bin.
3. **Club-adaptive `v_max` presets.** On club select, load a chirp preset whose
   wrap multiples sit off that club's expected speed (mid-Doppler-span).
4. **Detect-and-flag.** Range-walk gives true speed independently, so when a
   shot lands near a wrap-to-zero multiple, flag launch angle **low-confidence**
   (feeds the existing confidence cascade) instead of reporting a corrupted
   value. Graceful degradation, never silent error.

**Evidence the mechanism works:** the 2026-07-07 static sweep (§0) recovered a
clean ±1° angle from a **zero-Doppler target** (a static reflector) at every
height — precisely because clutter was handled (clutterRemoval off) and the
target dominated its cell. A background-subtracted, range-gated ball in a wrap
band is the same condition. So the fix is proven in principle; Stage-1c must
validate it on a *real* shot that actually lands in a blind band.

**Where this lives:** the future software detector (`shot_detector.py`,
deferred until labeled real-shot data exists) + the Stage-1c custom-TLV angle
path. All clutter handling stays in our Python pipeline — transparent and
tunable, not the demo's black-box `clutterRemoval`.

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
