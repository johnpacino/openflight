# Stage-1c: L3 raw-ADC burst-dump (IWR6843) — plan

**Status:** committed direction 2026-07-11. The productizable path to a launch
angle from the TI. Supersedes both the DCA1000 idea and the stock point-cloud.

## Why this path

- **DCA1000 rejected** — ~$755, Windows/mmWave-Studio-centric, **not Pi-deployable**.
  It would validate raw ADC but the pipeline built around it can't ship, so we'd
  rebuild everything for production. Paying to validate a throwaway path is a bad
  trade.
- **Stock point-cloud rejected for the ANGLE** — 2026-07-11 field test: even with
  the wedge beaten (50 fps), the stock TLV is 2D (`z=0`, coarse demo-AoA) and
  can't cleanly isolate a fast ball from club-swing clutter. Streams fine, but
  gives no real launch angle. See [[project_iwr6843_framerate_200fps]].
- **L3 burst-dump = the product** — raw per-antenna ADC over the **existing USB**,
  Pi runs FBSS-MUSIC. No extra hardware, no LVDS. And it **sidesteps the wedge**:
  the wedge came from *sustained* streaming; a buffer-in-L3 + *triggered burst* is
  the same stable model as the OPS243 rolling buffer.

## Architecture

```
SEN-14262 sound trigger ──► Pi GPIO (IRQ, precise impact time; bypasses OPS/WiFi)
                               │
                               ▼  "dump" command over TI CLI UART (TI RX works)
IWR6843 firmware: continuous L3 rolling buffer of RAW ADC ──freeze+stream burst──►
                                                                                  │
                                              Pi data UART ◄───────────────────────┘
Pi: range-FFT → dealias (blind-zone/range-walk) → FBSS-MUSIC → launch angle
Validate vs OPS 9-iron reference (7 shots, 98–110 mph, launch 17–24° est.)
```

No OPS in the timing loop, no DCA1000, no LVDS — Pi + TI + sound trigger only.

## Trigger timing — rolling buffer + POST-trigger capture

A ball is on the tee at trigger time, so "dump the last N frames" (pure
pre-trigger) is wrong. Instead:

- **Continuous L3 rolling buffer** (overwrites oldest) absorbs trigger latency:
  sound travel ~4.4 ms @ 5 ft + Pi-IRQ→UART-command ~5–10 ms. The impact + first
  ball frames are already buffered when the dump command lands.
- **On trigger, capture ~5–8 MORE frames** (ball flying out), *then* freeze and
  dump. Dumped window straddles the trigger: ~2 pre-frames (latency cushion) +
  ~5–8 post-frames.
- Capture RAW at high rate (~200–250 fps — no per-frame processing), so ~10
  frames ≈ **~40 ms of dense launch-phase snapshots** = ideal for MUSIC.
- **Phase-1 constraint:** the buffer must support pre+post windowing (not just
  pre-trigger).

## Memory / frame count

- L3 ≈ 1.5 MB ÷ ~128 KB/raw-frame ≈ **~10 raw frames** — NOT a hard ceiling.
- More frames (margin for slow shots / longer trajectory): fewer range bins
  (128→64) or chirps (32→16) → ~20; range-gated per-antenna cube (on-chip
  range-FFT, keep corridor bins) → ~20–45; slim per-antenna slice at a detected
  cell → hundreds (but CFAR-dependent → less robust in clutter).
- Default: **full raw ADC, ~10 dense launch frames** — max Pi-side flexibility,
  robust to clutter. Reach for the cube/fewer-bins only if we need more window.

## Phased plan

- **P0 — TOOLCHAIN GATE (do first).** Building custom firmware needs CCS +
  mmWave SDK (x86). On the M1 Mac that's the Win11-ARM VM (per the toolchain
  notes) or a Linux-x86 box/cloud. *Prove we can build + flash the STOCK demo
  unchanged.* Nothing else matters until this works.
- **P1** — raw ADC → L3 ring buffer + UART dump on a CLI command (start from the
  SDK's ADC/CBUFF capture examples; support pre+post windowing).
- **P2** — wire sound → Pi GPIO IRQ → dump command; capture a burst straddling a
  real impact.
- **P3** — Pi-side: range-FFT + dealias + FBSS-MUSIC → angle; validate against the
  OPS 9-iron reference.

## Risks / open questions

- Toolchain on M1 (Win11-ARM VM is the known path).
- Exact usable L3 size; achievable raw-capture frame rate.
- Pi-side dealiasing (the ~120 mph blind zone) + range-walk within the burst.
- Trigger-window tuning (how many pre/post frames for each club speed).
