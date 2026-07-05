# OpenFlight Radar Board — Custom PCB Requirements
*(Pi daughter-board on a short harness — not a stacked HAT; see §3.5)*

**Status:** DRAFT — requirements gathering while awaiting IWR6843LEVM (ordered
2026-07-02). **This board is gated on Stage-1 eval results.** Do not start
layout until the decision gates below are resolved.

**Goal:** Replace the OPS243 + 2× K-LD7 + sound-trigger sensor stack with a
single 60 GHz coherent radar daughter-board — mounted vertically, high in the
enclosure, connected to the Raspberry Pi by an ~8-wire harness (§3.5) — that
streams per-antenna complex data over UART, with all launch-angle processing
(FBSS-MUSIC) staying in Python on the Pi.

---

## 1. Decision gates (resolve from Stage-1 eval before layout)

| Gate | Question | Resolves |
|------|----------|----------|
| G1 | Does the rotated LEVM (8-el vertical line) beat 2° LA MAE at the 10-inch mount? | Variant A vs Variant B (see §2) |
| G2 | If G1 fails: does it pass at 18–33 in? | Variant B + enclosure/tripod change, or project rethink |
| G3 | Measured SNR margin on real golf balls at 2–5 m | Antenna gain requirement; whether 4-patch chains needed vs LEVM's low-gain elements |
| G4 | Real per-unit phase-calibration stability (temp, mounting) | Whether field-cal routine is required in v1 software |
| G5 | Velocity unfold (range-migration + Doppler) validated on real shots | Whether OPS is retired or kept for speed in v1 |

## 2. Two board variants — build ONE, chosen by G1/G2

### Variant A — "Aperture board" (if rotated-8-el passes at 10 in)
- **Chip:** IWR6843 (non-AoP), FCBGA, ~$40 @ qty 100
- **Antenna:** COPY the LEVM/ISK reference antenna layout, **rotated 90°**
  so the 8-virtual-element λ/2 line is vertical (launch angle) and the
  TX2 λ/2 offset row is horizontal (aim). Do not redesign; transfer the
  reference gerbers onto the same stackup.
- **Key de-risk fact (SWRU585):** the LEVM antenna is **FR4 (FR408HR)** with
  TI's "relaxed PCB rules — no micro vias, only through vias, no vias on BGA
  pads," explicitly designed for low-cost fab. A PCBWay/JLCPCB-class shop can
  build this. The LEVM itself is the existence proof; match its substrate
  (FR408HR or equivalent Dk/thickness) so the antenna doesn't detune.
- Antenna gain ~5–6 dBi/element (LEVM-class). If G3 shows thin SNR margin,
  consider 2-patch chains (+3 dB, halves the wide-axis FOV — recheck ball
  corridor coverage before doing this).

### Variant B — "AoP board" (fallback: only if G1 fails and the tall mount passes)
- Same daughter-board-on-harness mechanical concept as Variant A (§3.5) —
  only the antenna implementation differs.
- **Chip:** IWR6843AOP — antenna in package, ZERO antenna copper on our board
- Simplest possible RF: fastest signal on the PCB is the 40 MHz crystal
- Requires enclosure/product change to ~0.45 m+ mount height (G2)
- Package keep-out + radome rules per TI AoP application note (no metal or
  thick material in front of the package; plastic radome with specified standoff)

## 3. Reference design to copy (from SWRU585 / LEVM)

The LEVM block diagram is the BOM starting point. Copy, delete what we don't
need, add the Pi interface.

### 3.1 Power tree (copy from LEVM — proven with this exact chip)
| Rail | LEVM part | Note |
|------|-----------|------|
| 3.3 V | TPS628502HQDRLRQ1 buck | from 5 V in |
| 1.8 V | TPS6285020MQDRLRQ1 buck | |
| 1.2 V | TPS6285018AQDRLRQ1 buck | |
| 1.0 V | TPS628503QDRLRQ1 buck | |
- 5 V input: from Pi 40-pin header (HAT) — budget ~2.5–3 W radar load;
  confirm Pi 5 V rail headroom with Pi 4/5 + our peripherals, else barrel/USB-C aux input.
- TI note: simultaneous multi-TX exceeds USB budget, but TDM-MIMO (our mode)
  fires one TX at a time — fine.
- Power sequencing per IWR6843 datasheet — the discrete-buck approach on the
  LEVM shows sequencing handled with enables/RC; replicate exactly.
- INA226 current monitors: OPTIONAL — keep footprints, DNP for cost.

### 3.2 Digital / boot (copy from LEVM)
| Block | LEVM part | Note |
|-------|-----------|------|
| QSPI boot flash | MX25R1635 | firmware image lives here |
| 40 MHz XTAL | (per TI ref) | tight ppm spec per datasheet |
| Reset | pushbutton + Pi GPIO line | Pi must be able to reset the radar |
| SOP mode straps | switch on LEVM | **wire SOP0/1/2 to Pi GPIOs** → Pi can
  flip flashing/functional mode itself → `./flash.sh` self-flashing, no
  switches for end users |
| EEPROM, temp sensor | CAT24C08, TMP112A | OPTIONAL — DNP |
| CAN-FD transceivers | TCAN1042 | DELETE — not needed |

### 3.3 Data interface to the Pi (replaces LEVM's USB)
- **Primary: UART @ 921600 → Pi GPIO14/15 (PL011 full UART).** ~90 KB/s
  ceiling; slim custom TLV (~100–200 B/frame @ 100–200 Hz) fits with margin.
- Level: 3.3 V both sides — direct connection, no level shifter.
- **Optional secondary: SPI slave → Pi SPI** (several MHz) as the
  bandwidth escape hatch for fatter diagnostic payloads. Wire it; use later.
- **Do NOT fit the CP2105 USB-UART** (LEVM's J5 path) on production units;
  OPTIONAL debug footprint for bench bring-up with a laptop.
- MSS logger UART → spare Pi GPIO or test pads (firmware debug).

### 3.4 Debug / development provisions (v1 prototypes only, DNP later)
- **60-pin DCA1000 HD connector** (Samtec, per LEVM): raw LVDS capture during
  board validation — lets us A/B the custom board against the LEVM with the
  same capture path. Fit on first article; DNP in production.
- JTAG pads (or 60-pin MMWAVEICBOOST connector footprint) — pads only.

### 3.5 Mechanical — daughter-board on harness, NOT a stacked HAT
**Decision (2026-07-02): the flat-stacked "HAT" concept is rejected for BOTH
variants.** Radiation is broadside (out of the board/package face); a board
stacked flat on a horizontal Pi points the beam at the ceiling. Additionally
the Pi already carries an X1206 UPS HAT (header/stack contention).

Adopted concept: **small vertical daughter-board on a short harness.**
- The radar board needs only **~8 lines** from the Pi: 5 V, GND, UART TX/RX,
  SOP0/1/2, NRST. Use a keyed connector (JST-XH/PH class), harness 15–25 cm.
  UART @ 921600 is comfortable at this length; twist/ground-pair the UART.
- Board mounts vertically **at the TOP of the enclosure**, face downrange:
  maximizes radar phase-center height within the 10-in cap (multipath
  separation ΔR ∝ h_radar) while Pi + X1206 UPS sit at the bottom.
- Board outline driven by antenna keep-out, not HAT spec.
  **Size target: ~50 × 60 mm production; ~65 × 75 mm prototype** (extra area
  = DCA1000 connector + debug USB, depopulated later). Small is preferred
  (stiffness = phase-cal stability; cost), but respect the floor: ground-plane
  margin around the array (a few λ), no tall parts in front of the radiating
  face, DCDCs physically separated from the antenna region.
- **Floorplan:** antenna array at the TOP edge (adds ~2 cm phase-center
  height on top of the top-of-enclosure mounting — ΔR ∝ h_radar), chip
  directly below (short feeds), power section + harness connector at the
  bottom edge, digital kept away from the array. Tilt: adjustable bracket on
  prototype; fixed molded angle + software cal in production, set from
  Stage-1 findings.
- Power: radar ~2.5–3 W from the 5 V rail — within X1206 budget; measure
  battery-runtime impact during Stage 1 (UPS was sized before this load).
- Antenna keep-out: no copper/components in front of array; enclosure
  window: plastic radome, thickness/standoff per TI guidance.
  EN 62311 note: ≥20 cm human separation during operation (satisfied by use).
- Storage note from TI: immersion-silver antenna finish oxidizes (cosmetic);
  production finish decision: immersion silver (RF-best) vs ENIG (durable) —
  match reference antenna assumptions. **Open item.**

## 4. Firmware requirements (same for both variants)
- Base: mmWave SDK 3.x LTS demo, modified:
  1. Custom chirp profile: ~7 ms frames (100–200 Hz); subframe A (2-TX fast
     loop, LA workhorse) + subframe B (3-TX, aim) if G1 path chosen.
  2. **Custom TLV**: per-virtual-antenna complex values at detected target's
     range-Doppler bin (the MUSIC input). ~100–200 B/frame.
  3. Keep stock point-cloud TLV (cross-check + fallback).
- Flashing: from the Pi over UART with SOP GPIO control (no external tools).
- All angle processing stays in Python on the Pi (repo, git-updatable).

## 5. Calibration requirements
- Per-unit phase/gain cal: guided routine using corner reflector at known
  position (user-executable; leverages G4 findings). Store cal in EEPROM or
  Pi-side config file (decide: board-resident vs host-resident).
- Boresight/tilt cal: digital-level entry or reflector-based (the K-LD7 +2°
  boresight lesson — do not skip).

## 6. Manufacturing plan
- 4–6 layer FR408HR (or JLC/PCBWay equivalent Dk), through-via only,
  controlled impedance on RF feeds (Variant A) — per LEVM's relaxed rules.
- Assembly: 0.65 mm-pitch flip-chip BGA → PCBA service (stencil + reflow +
  ideally X-ray). NOT hand-solderable. Users buy assembled boards.
- Prototype run: 5 boards assembled, est. $500–900 all-in per spin;
  target ≤2 spins (spin 1 with DCA1000 connector + debug USB fitted).
- Production BOM target: chip ~$40 + board/assembly ~$40–80 + passives
  → **$100–150/unit at qty 50–100** (vs ~$600 of sensors it replaces).
- Validation per spin: A/B against the LEVM (golden reference) on identical
  shots; static corner-reflector angle sweep; per-unit cal repeatability.

## 7. Regulatory notes (before anyone SELLS assembled units)
- 60–64 GHz ISM: right band for consumer product; FCC Part 15 certification
  required for sold assembled devices; kit/DIY route defers but does not
  erase obligations — get real advice before commercializing.
- FCC note printed on the LEVM itself: "For evaluation only; not FCC
  approved for resale" — our board inherits the same distinction between
  eval and product.

## 8. Open items
1. Daughter-board bracket/mount design inside the enclosure (vertical face,
   tilt adjustment or fixed tilt + software cal) and harness connector choice.
2. Enclosure radome spec (material, thickness, standoff) for chosen variant.
3. Pi 5 V budget with radar + Pi under load (measure on eval rig).
4. Cal storage location (EEPROM on board vs Pi config).
5. Whether OPS remains in v1 for ball speed (gate G5).
6. Antenna finish (immersion Ag vs ENIG) for production.
7. Which Pi (4 vs 5 vs CM) is the v1 target — affects UART/SPI pinning and
   power budget.

## 9. Explicitly out of scope (decided against, with reasons — see
`docs/plans/` history and project memory)
- 24 GHz architectures (250 MHz ISM bandwidth → no range gating of floor
  bounce — the dominant documented error)
- Multi-module coherent arrays from sealed transceivers (independent LOs)
- Sound-card / audio-codec ADC capture (solved a non-problem; capped at
  2 coherent elements off-the-shelf)
- AWR2243 (no on-chip DSP → forces LVDS/FPGA capture path, breaks
  Pi-Python architecture)
- Cascade / >12 virtual elements (sim shows 8 vertical elements suffice)
