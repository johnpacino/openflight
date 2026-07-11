# Stage-1c firmware — L3 raw-ADC burst-dump (IWR6843)

Custom firmware that buffers raw ADC in the chip's L3 and streams a burst over
UART when the Pi sends `l3dump` (at the sound-trigger edge). This is the
productizable alternative to the DCA1000 — Pi + TI + sound trigger, no LVDS, no
lab capture card. Full plan: [../docs/plans/stage1c_l3_burst_dump.md](../docs/plans/stage1c_l3_burst_dump.md).

## Where to build vs flash
- **Build: Stacey's 2017 Intel Mac.** Docker runs x86-64 Linux **natively** there
  (full speed) — the TI SDK is x86 Linux/Windows only. (Apple Silicon works under
  `--platform=linux/amd64` but emulates x86 → slow.)
- **Flash: the Pi web flow / UniFlash.** Off-container; the `.bin` is portable.
- **Run: the Pi** drives it via [`../iwr6843_runtime.py`](../iwr6843_runtime.py).

## Phase 0 — prove the toolchain FIRST
1. TI account → download for **Linux x86**: mmWave SDK 3.6.x (IWR6843), the
   `ti-cgt-arm` (R4F) + `ti-cgt-c674x` (DSP) compilers, SYS/BIOS, XDCtools,
   SysConfig, and the mmWave DFP — **exact versions in the SDK release notes**.
   Put the `.bin` installers in `firmware/ti_installers/` (git-ignored; huge +
   license-gated).
2. Uncomment the install `RUN` in the `Dockerfile`, then:
   `docker build -t iwr-sdk firmware/`
3. `docker run --rm -it -v "$PWD:/work" iwr-sdk`, source the SDK env, and build
   the **stock mmw demo unchanged** → get its `.bin`.
4. Flash it (Pi web / UniFlash) → confirm it streams TLV.

   ✅ Toolchain proven → Phase 1 unblocked. (If the stock build fails, fix that
   before writing any custom C — it's the cheapest place to hit toolchain issues.)

## Phase 1 — the L3-dump app
`l3_dump/` is a **skeleton**: the ring-buffer + post-trigger windowing logic and
the wire format are final; the SDK glue is marked `TODO(SDK)`.
- `dump_format.h` — the 20-byte header + payload layout, byte-for-byte matching
  `iwr6843_l3dump.py` (`parse_dump`). **One format, two languages — keep in sync.**
- `l3_dump.c` — per-frame raw-ADC archival into an L3 ring, a `l3dump` CLI command
  that captures `POST_FRAMES` then streams the window. Fill in the `TODO(SDK)`
  calls (EDMA ADCBUF copy, `UART_write`, CLI registration, the frame-done hook).

Build → `.bin` → flash → the Pi's `iwr6843_runtime.py` runs it end-to-end (the Pi
side is already implemented + tested: `iwr6843_l3dump.py`, `iwr6843_runtime.py`).

## Contract with the Pi (already implemented Pi-side)
| | |
|---|---|
| Trigger command | Pi writes `l3dump\n` to the CLI UART on the sound-trigger GPIO edge |
| Dump | `l3_dump_header_t` + int16 I/Q, parsed by `iwr6843_l3dump.parse_dump` |
| Keep in sync | `N_TX` / `N_RX` / `N_SAMPLES` / `LOOPS` across the `.cfg`, `l3_dump.c`, and the Pi meta |

## Sizing (tune in `l3_dump.c`)
`RING_FRAMES` ≈ L3 (~1.5 MB) / raw-frame (~128 KB) ≈ **~12**. `PRE_FRAMES` covers
sound + IRQ + command latency (~1 frame); `POST_FRAMES` covers the ball's flight
window. Fewer range bins / chirps buy more frames if you need a wider window.
