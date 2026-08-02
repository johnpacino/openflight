# IWR6843 Firmware Developer Guide

OpenFlight uses custom firmware on the TI IWR6843LEVM to preserve a short radar
movie around impact in the chip's on-board L3 RAM. The firmware continuously
processes chirps, stores selected complex range bins in a circular frame ring,
and streams that ring to the Raspberry Pi after the shared sound trigger.

Most builders do **not** need to compile firmware. A flashable test image is
checked into the repository. Build the firmware only when changing the hardware
envelope, HWA/EDMA processing, or the binary dump contract; normal capture
tuning now happens in the runtime `.cfg`.

For hardware wiring, mounting, geometry, calibration, and normal OpenFlight
startup, use the [IWR6843 Operator Guide](../docs/iwr6843/README.md).

## Experimental Hybrid-Cadence Release

Use this firmware and runtime configuration together:

| Component | Current value |
|---|---|
| Flash image | `firmware/releases/l3_dump_hybrid_cadence_20260801.bin` |
| Runtime config | `config/iwr6843_l3dump_vTX2_hybrid.cfg` |
| Reference calibration | `config/iwr6843_calibration_reference.json` |
| Build target | `make -C firmware docker-build` (any host) |
| Flash image size | 342,660 bytes |
| Flash SHA-256 | `7187dcbed6328313eeee32ade9eccbaa33451e383fbc8205e81db4d9f431043c` |
| Dump format | Version 7, temperature report plus variable-width timed complex range-FFT snapshots |
| Default complete dump size | 783,532 bytes |

Current release binaries use `<feature_name>_<YYYYMMDD>.bin`. Capabilities such
as temperature reporting and the dump-contract version belong in source and
header metadata, not in the filename. Rollback binaries use the same convention;
renaming an artifact does not alter its published hash.

Verify the image before flashing:

```bash
sha256sum firmware/releases/l3_dump_hybrid_cadence_20260801.bin
```

The v6 image acquires all 12 loops every 2 ms. It retains every pre-trigger
frame in the narrow 32-bin club window, then retains alternate post-trigger
acquisitions in the wider 53-bin ball window. The resulting capture contains
16 dense pre-impact frames at 2 ms spacing and 16 full-quality ball snapshots
at 4 ms spacing. The post frames still contain all 12 loops.

This release has passed the container build, wire-format tests, and synthetic
end-to-end speed/angle processing. It has not yet passed a hardware smoke test
or source-of-truth TrackMan validation. Keep
`l3_dump_configurable_capture_20260728.bin` and its matching config available
as the immediate rollback.

### Club16 Shelf Candidate

The loop-heavy TrackMan A/B candidate is intentionally separate from the
current hybrid configuration:

| Component | Candidate value |
|---|---|
| Flash image | `firmware/releases/l3_dump_club_impact_20260801.bin` |
| Runtime config | `config/iwr6843_l3dump_club16.cfg` |
| Build target | `make -C firmware docker-build RELEASE_NAME=l3_dump_club_impact_20260801.bin` |
| Flash image size | 344,324 bytes |
| Flash SHA-256 | `10338e5fb72ac755ba14d998df950f5e6b6d8c12f8cc7abf4fd69bbb43f61369` |
| RF configuration | 3 TX, 4 RX, 16 loops, 2.5 ms acquisition spacing |
| L3 plan | 8 narrow pre + 6 narrow impact + 14 wide ball frames |
| L3 use | 763,392 of 786,432 bytes |

The narrow 18-bin phases nominally cover approximately 3.4-6.2 feet from the
radar. The six impact frames retain every 2.5 ms acquisition. Ball storage then
uses the established 53-bin middle/late windows and retains alternate
acquisitions at 5 ms spacing. This candidate is built and parser-tested but
must pass a board smoke test before it is used for hitting or TrackMan testing.

Flash it with:

```bash
uv run python firmware/flash_iwr6843.py \
  firmware/releases/l3_dump_club_impact_20260801.bin \
  --port /dev/ttyUSB0
```

Building the firmware needs the TI mmWave SDK and ARM codegen tools, which ship
as **Linux x86_64 installers only**, so it cannot be built directly on an Apple
Silicon Mac or on the Pi. Use `make -C firmware docker-build` (see
[Prerequisites For Building](#prerequisites-for-building)); the target drops the
image into `firmware/releases/` and prints its hash. Other experimental images
and configs remain available through Git history rather than appearing as
supported setup choices.

The linked image is verifiable independently of the hash: the map file must show
`.l3ring` at `0x51000000` with size `0x000c0000`. The firmware reserves all
768 KiB as an arena, then rejects any runtime plan that would exceed it.

```bash
grep -E "l3ring|L3_RAM" firmware/iwr6843/l3_dump_mss.map
```

The loop count and acquisition cadence come from `frameCfg`. `captureCfg`
supplies the standard pre/post windows and retention stride; `phaseCaptureCfg`
explicitly reserves narrow pre/impact phases before a wide decimated ball
phase. The firmware prints the resolved plan during `sensorStart`; invalid or
oversized plans fail before RF capture begins. Immediately before streaming,
the firmware also appends the TI temperature report to the dump.

## What The Current Firmware Captures

### Capture Geometry

| Setting | Value |
|---|---|
| Transmitters | 3 |
| Receivers | 4 |
| Loops per frame | 12 |
| Chirps per frame | 36 (`3 TX x 12 loops`) |
| Frames in the default plan | 32 |
| Acquisition spacing | 2 ms |
| Frames before/around trigger | 16 |
| Retained frames after trigger | 16 |
| Retained post-trigger spacing | 4 ms (`postStride=2`) |
| Acquired ADC samples per chirp | 128 |
| Stored pre-trigger bins per chirp/RX | 32 complex bins |
| Stored post-trigger bins per chirp/RX | 53 complex bins |

The storage command is:

```text
captureCfg 20 32 32 53 47 16 2
```

Its fields are
`preStart preBins postStart postBins lateStart postFrames postStride`.
The first half of the post frames uses `postStart`; the second half uses
`lateStart`. `postStride=2` retains every other post-trigger acquisition. All
windows must remain within the 128-bin range FFT.

The RF portion of each frame occupies approximately:

```text
3 TX x 12 loops x (7 us idle + 38 us ramp) = 1,620 us
```

A 2,000 us frame therefore leaves about 380 us for HWA/EDMA completion and
firmware rearm. That is intentionally an aggressive experimental margin. The
first hardware test must verify `hwa_rearm_err=0`, matching HWA input/output
frame counts, and repeatable dump/rearm cycles before hitting balls.

All three transmitters are retained. The outer transmitter pair is used by the
vertical launch estimator, while the remaining transmitter provides the
experimental horizontal aperture. Each transmitter is fired once per TDM loop;
the four receivers sample every chirp simultaneously.

### On-Chip Data Path

```text
RF chirp
  -> ADCBUF
  -> HWA 128-point range FFT
  -> EDMA copies the configured complex range window
  -> narrow circular pre-trigger ring in L3 RAM
  -> Pi sends "l3dump" at the sound-trigger edge
  -> firmware fills the configured wider post-trigger buffer
  -> freeze at a completed-frame boundary
  -> stream header, per-frame window metadata, and variable-width IQ over UARTA
  -> restart the pre-trigger ring for the next shot
```

HWA performs the range FFT before storage, but the saved bins remain complex
I/Q. That phase information is required for vertical and horizontal direction
of arrival; this is not a magnitude-only detection list.

### Configurable Range Windows

The default plan stores a narrow tee/club corridor in the rolling ring and
switches to the proven 53-bin ball-flight window after the trigger:

| Time region | Frames | Start bin | Stored bins | Purpose |
|---|---:|---:|---:|---|
| Rolling history plus active trigger frame | 16 | 20 | 20-51 | Preserve more approach history without spending ball-flight width |
| Middle post-trigger flight | 8 | 32 | 32-84 | Follow the ball away from the tee |
| Late post-trigger flight | 8 | 47 | 47-99 | Retain farther flight and the net-side region |

Each frame's start bin, valid bin count, and elapsed time from the previous
retained frame are written into the v6 dump. The host therefore knows both the
absolute range and actual sampling time represented by every frame.

### L3 Memory Budget

The IWR6843 provides 768 KiB (786,432 bytes) of L3 RAM for the capture ring. The
default stored payload is:

```text
(16 pre frames x 32 bins + 16 post frames x 53 bins)
  x 3 TX x 12 loops x 4 RX x 4 bytes
= 783,360 bytes
```

The linker reserves the full 786,432-byte region as a flat arena. At
`sensorStart`, the planner reserves the requested post frames and converts all
remaining whole-frame space into a circular pre-trigger ring. The default plan
leaves 3,072 bytes unused because another 18,432-byte pre frame cannot fit.

The transfer adds a 20-byte header and 32 four-byte frame descriptors:

```text
783,360 + 20 + 128 = 783,508 bytes
```

At the saturated 1,041,667-baud link (measured 103,038 bytes/s) one dump takes
about 7.6 seconds. The radar is blind for that whole window, which
is why `--iwr6843-capture-timeout` defaults to 16 seconds.

For comparison, retaining all 128 complex range bins with the same TX, loop,
and 32-frame count would require 2,359,296 bytes. On-chip range FFT plus
selected range windows is what makes the denser capture fit.

## Firmware And Host Contract

The wire format is defined in two places that must stay synchronized:

- Firmware: [`iwr6843/dump_format.h`](iwr6843/dump_format.h)
- Host parser: [`../src/openflight/iwr6843/dump.py`](../src/openflight/iwr6843/dump.py)

The current version 6 transfer contains:

1. A packed 20-byte little-endian `l3_dump_header_t`.
2. One packed `(uint8 start bin, uint8 valid bin count, uint16 delta-us)`
   descriptor for each frame.
3. Only each frame's declared valid complex int16 range bins, ordered by frame,
   chirp, RX, and local range bin.
4. Each complex sample in TI's native imaginary-then-real order.

The header carries:

| Field | Meaning |
|---|---|
| `magic` | `ILD1` synchronization marker |
| `version` | Dump contract version |
| `n_frames` | Number of ring frames |
| `chirps_per_frame` | `n_tx x loops` |
| `n_tx`, `n_rx` | Virtual-array geometry |
| `n_samples` | Raw ADC count or maximum stored frame width |
| `sample_fmt` | Raw ADC, fixed snapshot, fixed-width window, variable-width window, or timed variable-width window |
| `trigger_frame` | Oldest circular slot for legacy formats; zero for chronological v5/v6 dumps |
| `frame_period_us` | Base acquisition spacing; v6 descriptors carry retained-frame deltas |

Changing the header, sample order, frame metadata, or sample format requires a
matching host-parser change and regression tests in the same commit.

## Repository Layout

| Path | Responsibility |
|---|---|
| `firmware/iwr6843/l3_dump.c` | RF control, HWA/EDMA pipeline, circular ring, freeze/rearm, CLI, and dump streaming |
| `firmware/iwr6843/dump_format.h` | Packed firmware-side wire contract |
| `firmware/iwr6843/makefile` | TI mmWave SDK application build and meta-image generation |
| `firmware/iwr6843/mss.cfg` | SYS/BIOS configuration |
| `firmware/iwr6843/mss_linker.cmd` | Places the ring and optional scratch buffers in L3 RAM |
| `firmware/Makefile` | Toolchain setup, installer fetch, container and production build targets |
| `firmware/Dockerfile` | x86_64 build environment, so macOS/arm64 hosts can build |
| `firmware/releases/` | Checked-in test image and validated rollback image |
| `firmware/flash_iwr6843.py` | Pi-compatible IWR6843 ROM bootloader client |
| `config/iwr6843_l3dump_vTX2_hybrid.cfg` | Runtime RF, L3 storage, and post-retention plan |
| `src/openflight/iwr6843/dump.py` | Python decoder and executable format reference |

## Where To Build, Flash, And Run

| Operation | Supported environment |
|---|---|
| Build | Native x86_64 Linux, the Docker container, or an x86_64 Debian VM |
| Build on Apple Silicon | **Docker container (recommended)**, or UTM emulating x86_64 Debian |
| Build on Raspberry Pi 5 | Not currently reliable because TI's x86/i386 installer stubs can fail under QEMU and a 16 KiB host page size |
| Flash | Raspberry Pi using `flash_iwr6843.py`, or TI UniFlash as a fallback |
| Run | Raspberry Pi through OpenFlight |

On an Apple Silicon Mac the container path below is the least work. The UTM VM
remains supported and is the fallback if Docker is unavailable. The Pi can flash
and run the image, but it should not be treated as the canonical compiler host.

## Prerequisites For Building

You only need these to build firmware. Flashing the checked-in image needs none
of them.

| Requirement | Notes |
|---|---|
| **Docker** | Docker Desktop or Engine, with BuildKit (default on 23+). The only host dependency on macOS. |
| **TI installers** (5 files) | Three download automatically; two need a TI login. See [The Installers](#the-installers). |
| **~15 GB free disk** | ~320 MB of automatic installers, ~1.5 GB more manual, plus the container image. |
| **Time** | The one-time `docker-image` step runs TI's installers under x86_64 emulation. Budget tens of minutes. Rebuilds after that are just the compile. |

Alternatively, a native x86_64 Linux host or an x86_64 Debian VM works without
Docker; see [Build On Native x86_64 Linux](#build-on-native-x86_64-linux) and
[Build On Apple Silicon With UTM](#build-on-apple-silicon-with-utm).

**You cannot build on an Apple Silicon Mac directly, or on the Raspberry Pi.**
TI's tools are Linux x86_64 with 32-bit installer stubs; the Pi is aarch64 and
Pi OS uses 16 KiB pages. The container supplies the right environment on both.

## Build In The Docker Container (Recommended On macOS)

```bash
# 1. Fetch what TI serves without a login, and get told exactly what is left.
make -C firmware fetch-installers

# 2. Build the image once (slow: emulated x86_64 + a large SDK install).
make -C firmware docker-image

# 3. Build the firmware. Repeat this after any firmware change.
make -C firmware docker-build
```

### The Installers

Three of the five are unauthenticated and `fetch-installers` downloads them
(verified 2026-07-27, ~318 MB total). Two are behind a TI login plus an
export-control acceptance and cannot be scripted -- a direct GET returns an HTML
consent page, so `fetch-installers` names them, prints the URL, and stops.

| File | Source |
|---|---|
| `bios_6_73_01_01.run` | automatic |
| `sysconfig-1.10.0_2163-setup.run` | automatic |
| `xdctools_3_61_00_16_core_linux.zip` | automatic |
| `mmwave_sdk_03_06_02_00-LTS-Linux-x86-Install.bin` | manual — <https://www.ti.com/tool/MMWAVE-SDK> |
| `ti_cgt_tms470_20.2.7.LTS_linux-x64_installer.bin` | manual — <https://www.ti.com/tool/ARM-CGT> |

Save the two manual downloads into `firmware/ti_installers/` (gitignored) and
re-run `make -C firmware docker-image`.

Every automatic download is magic-byte checked. TI answers a gated URL with
HTTP 200 and an HTML page, so "it downloaded" is not evidence of success; a
response that is not ELF or ZIP is deleted and reported rather than left to fail
later inside `docker build`.

`bios_6_73_01_01.run` is an **ELF 32-bit Intel 80386** executable, which is the
concrete reason the container needs i386 multiarch rather than x86-64 alone.

`docker-build` bind-mounts the repository, so the image and its SHA-256 land on
the host in `firmware/releases/` exactly as a native build would. It runs the
same `build-native` recipe -- there is no second copy of the build definition.
Use `make -C firmware docker-shell` for an interactive shell in the same
environment.

The image self-verifies the three things TI's installers require, and fails at
build time rather than mid-compile if any is missing:

| Requirement | Why | What fails without it |
|---|---|---|
| `x86_64` | SDK and codegen are x86_64-only | Apple Silicon, Raspberry Pi |
| 4096-byte pages | TI installer stubs | Raspberry Pi 5 (16 KiB pages) |
| 32-bit i386 execution | installer stubs are 32-bit | Docker Desktop's Rosetta backend |

If the i386 check fails, turn off **Use Rosetta for x86_64/amd64 emulation** in
Docker Desktop's settings so QEMU handles 32-bit code.

Verify the whole environment without supplying any installers:

```bash
docker build --platform linux/amd64 --target base -t openflight-iwr-sdk-base firmware/
```

## Build On Apple Silicon With UTM

### 1. Create An x86_64 Debian VM

In UTM:

1. Select **Create a New Virtual Machine**.
2. Select **Emulate**, not Virtualize.
3. Select **Linux** and an amd64 Debian netinst ISO.
4. Use `Intel ICH9 based PC (2009, x86_64)`.
5. Allocate at least 4 GB RAM and 30 GB storage.
6. Install `SSH server` and `standard system utilities`; a desktop is optional.
7. Eject the installer ISO before the first reboot into the installed system.

Confirm the guest architecture and page size:

```bash
uname -m
getconf PAGE_SIZE
```

Expected output is `x86_64` and `4096`.

### 2. Put OpenFlight In The VM

Clone the repository inside the VM or copy your existing worktree with `rsync`:

```bash
sudo apt-get update
sudo apt-get install -y git rsync openssh-server
git clone https://github.com/jewbetcha/openflight.git
cd openflight
```

To copy an existing worktree from the Mac instead:

```bash
rsync -av --exclude '.venv' ~/Projects/openflight/ \
  openflight@VM_ADDRESS:~/openflight/
```

Find the VM address with `ip addr` inside Debian.

### 3. Supply The TI Installers

`make -C firmware fetch-installers` works here too and downloads three of the
five; see [The Installers](#the-installers) for which two need a TI login.

TI's installers are large and license-gated, so they are intentionally ignored
by git. Place these exact files under `firmware/ti_installers/` inside the VM:

```text
mmwave_sdk_03_06_02_00-LTS-Linux-x86-Install.bin
ti_cgt_tms470_20.2.7.LTS_linux-x64_installer.bin
bios_6_73_01_01.run
sysconfig-1.10.0_2163-setup.run
xdctools_3_61_00_16_core_linux.zip
```

The application is MSS/R4F-only; it does not require the C674x DSP compiler or
DSP libraries.

Verify the installer set:

```bash
make -C firmware check-installers
```

### 4. Install The Build Environment

Install Debian packages, probe every installer stub, and install the TI tools
under `/opt/ti`:

```bash
make -C firmware install-ti-deps-native
make -C firmware probe-installers-native
make -C firmware install-ti-tools-native
```

The resulting layout is:

```text
/opt/ti/sdk/mmwave_sdk_03_06_02_00-LTS
/opt/ti/cgt-arm/ti-cgt-arm_20.2.7.LTS
/opt/ti/bios/bios_6_73_01_01
/opt/ti/xdc/xdctools_3_61_00_16_core
/opt/ti/sysconfig
```

### 5. Build The Current Firmware

From the repository root inside the VM:

```bash
make -C firmware build-native
```

The target performs the application build, generates the flashable TI
meta-image, and copies the production image into `firmware/releases/`:

```text
firmware/releases/l3_dump_hybrid_cadence_20260801.bin
```

Generated `.xer4f`, `.map`, and intermediate `.bin` files stay under
`firmware/iwr6843/` and are ignored by Git. Current production images and
intentional rollback images live under `releases/`.

### 6. Copy Artifacts Out Of The VM

From the Mac:

```bash
mkdir -p artifacts/firmware_build
rsync -av \
  openflight@VM_ADDRESS:~/openflight/firmware/releases/ \
  artifacts/firmware_build/
```

## Build On Native x86_64 Linux

Use the same installer files and Make targets as the UTM VM. Confirm `uname -m`
reports `x86_64`, then start at **Supply The TI Installers** above.

The tool paths can be overridden when a machine does not use `/opt/ti`:

```bash
make -C firmware build-native \
  TI_ROOT=/custom/ti
```

## Supported Build Target

`build-native` defines the hardware envelope: 3 TX, 4 RX, 128-point range FFT,
complex IQ16 snapshots, a maximum of 16 loops, and the configurable L3 planner.
The `.cfg` chooses loops, cadence, window positions/widths, and post-frame
count. Reach the same build through either path:

| Target | Host | Notes |
|---|---|---|
| `make -C firmware docker-build` | any, incl. Apple Silicon | Runs `build-native` inside the x86_64 container. Recommended. |
| `make -C firmware build-native` | x86_64 Linux only | Direct, no container. Fails fast via `check-tools` if the host or toolchain is wrong. |

There is one build definition, not two: the container invokes the same
`build-native` recipe, so the geometry cannot diverge between paths. Use Git
history for earlier experiments rather than distributing those images or targets
as installation choices.

## Flash From The Raspberry Pi

The checked-in Python flasher uses the IWR6843 ROM UART bootloader and does not
require TI Cloud Agent. Flash over the CP2105 **Enhanced/UARTA** interface,
normally interface `00` and `/dev/ttyUSB0`. Do not use the Standard interface,
normally `/dev/ttyUSB1`.

### 1. Stop Serial Users

Stop OpenFlight and any calibration or test process using the TI port:

```bash
pgrep -af 'openflight|calibrate|shot_test'
sudo fuser -v /dev/ttyUSB0
```

### 2. Enter Flash Mode And Probe

Set the IWR6843LEVM switches to:

```text
S1.1 ON, S1.2 OFF, S1.3 ON, S1.4 ON, S1.5 OFF
```

Start the non-destructive probe:

```bash
uv run python firmware/flash_iwr6843.py \
  --probe \
  --port /dev/ttyUSB0
```

Follow the prompts exactly:

1. Type `READY` so the script opens UART and settles the control lines.
2. Press and release RESET only when requested.
3. Wait one second.
4. Type `PROBE`.

Do not continue until the ROM bootloader handshake passes.

### 3. Flash The Current Image

Leave the board in flash mode and run:

```bash
uv run python firmware/flash_iwr6843.py \
  firmware/releases/l3_dump_hybrid_cadence_20260801.bin \
  --port /dev/ttyUSB0
```

Type `READY`, press RESET when prompted, wait one second, and type `FLASH`. The
default workflow erases SFLASH, writes acknowledged chunks, closes the image,
and verifies the final ROM bootloader status.

Expected completion:

```text
Erasing existing SFLASH...
Opening firmware image...
Writing firmware...
Writing: 100% (342,660/342,660 bytes)
Closing and verifying firmware...

Flash verified by the IWR6843 ROM bootloader.
```

Do not reset, disconnect, or remove power while erase or write is active. A
failed write is recoverable because the ROM bootloader is not stored in SFLASH.
Leave the board in flash mode and rerun the complete command.

### 4. Return To Functional Mode

Set the switches to:

```text
S1.1 OFF, S1.2 OFF, S1.3 ON, S1.4 ON, S1.5 OFF
```

Press and release RESET. The firmware CLI and binary dumps now share the
Enhanced UART at 1,041,667 baud. Flashing itself always uses the ROM
bootloader's 115,200-baud protocol.

The flasher follows TI application note
[SWRA627, IWR6843 Bootloader Flow](https://www.ti.com/lit/an/swra627/swra627.pdf).

## Verify The Installed Firmware

Run OpenFlight with the matching config as described in the
[Operator Guide](../docs/iwr6843/README.md#start-openflight). With `--debug`, a
healthy capture reports:

```text
[IWR6843] Trigger #1: dumping firmware-frozen L3 ring
[IWR6843] Capture #1 complete: 549566 bytes
```

The firmware/config geometry is checked at `sensorStart`. A mismatch in TX
masks, loop count, frame count, or ADC samples is rejected rather than silently
capturing a differently shaped cube.

## Changing Capture Geometry

This is the section to start from when building a new firmware version. The
build itself needs Docker and TI's installers -- see
[Prerequisites For Building](#prerequisites-for-building) -- and the whole loop
is:

```bash
make -C firmware fetch-installers   # once; names the two manual downloads
make -C firmware docker-image       # once; slow
# edit firmware only when changing the hardware envelope or dump contract
uv run pytest tests/test_iwr6843_firmware_rearm.py
make -C firmware docker-build
```

Run the tests *before* the build: they verify the planner contract and default
L3 budget before the longer emulated compile.

The build fixes only the hardware envelope and maximums:

| Firmware define | Matching runtime concept |
|---|---|
| `N_TX` | Number of `chirpCfg` TX masks and TDM chirps per loop |
| `N_SAMPLES` | ADC samples in `profileCfg` |
| `L3_MAX_LOOPS` | Largest loop count accepted from `frameCfg` |
| `L3_MAX_CAPTURE_FRAMES` | Maximum number of frame descriptors |
| `L3_CAPTURE_BYTES` | L3 arena available to the planner |

The checked-in `.cfg` controls:

| Runtime field | Purpose |
|---|---|
| `frameCfg numLoops` | TDM loops per frame |
| `frameCfg framePeriodicity` | Time between frame starts |
| `captureCfg preStart preBins` | Circular pre-trigger range window |
| `captureCfg postStart postBins` | First-half post-trigger range window |
| `captureCfg lateStart` | Second-half post-trigger start; uses `postBins` width |
| `captureCfg postFrames` | Number of retained post-trigger frames |
| `captureCfg postStride` | Retain every Nth post-trigger acquisition |

At `sensorStart`, firmware first reserves the post buffer, then turns every
remaining whole-frame allocation into pre-trigger ring slots:

```text
bytes per bin = TX x loops x RX x 4
post bytes    = postFrames x postBins x bytes per bin
pre frames    = floor((786,432 - post bytes) / (preBins x bytes per bin))
```

The planner rejects odd loops, loops outside 2-16, windows outside the 128-bin
FFT, plans with no pre-trigger frame, and post reservations too large for L3 or
the 64-frame descriptor table. A test asserts the default split still leaves
enough pre-trigger history for club work after trigger latency; see
`docs/superpowers/specs/2026-07-27-iwr6843-preimpact-ring-design.md`.

Changing loops or widths changes the resolved frame count and transfer size.
The UART link is saturated at 1,041,667 baud (103,038 bytes/s measured), so the
default 783,508-byte dump takes about 7.6 seconds, during which the radar is
blind. Keep `--iwr6843-capture-timeout` comfortably above it.

Do not reuse application objects after changing compile-time geometry. The
named targets in `firmware/Makefile` remove application objects before each
build; use those targets rather than invoking the lower-level makefile against
stale objects.

## Validation Before Flashing A New Variant

Run the firmware contract and host-pipeline tests:

```bash
uv run pytest \
  tests/test_iwr6843_firmware_rearm.py \
  tests/test_iwr6843_pipeline.py \
  tests/test_iwr6843_driver.py \
  tests/test_iwr6843_monitor.py \
  tests/test_iwr6843_bootloader.py \
  tests/test_iwr6843_impact_time.py \
  tests/test_iwr6843_club_path.py \
  tests/test_iwr6843_club_phase.py
```

`test_iwr6843_firmware_rearm.py` also guards the build definition itself: the L3
budget, the pre/post frame split, the release name, and that the container does
not duplicate the build recipe. The three club/impact files cover the host-side
consumers of the frame count, which is the thing a ring change moves.

Also check:

1. The `.cfg` uses the expected TX/sample hardware envelope and a legal capture
   plan.
2. The map file keeps `.l3ring` inside L3 at exactly `0x000c0000` bytes.
3. The first static capture has the expected version, dimensions, frame period,
   per-frame window table, and total byte count.
4. Repeated dump/rearm cycles work without resetting the board.
5. Vertical and horizontal estimators can replay the new format offline.
6. Source-of-truth testing is repeated if timing, loops, frame spacing, TX
   schedule, or saved range coverage changed.

## Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| TI installer exits immediately | Build host is ARM, installer lacks execute permission, or i386 compatibility is missing | Use x86_64 Debian, run `install-ti-deps-native`, then `probe-installers-native` |
| VM returns to the Debian installer | ISO remains attached | Eject the ISO from the UTM CD/DVD drive and reboot |
| `check-installers` reports missing files | Installer name or location differs | Use the exact filenames under `firmware/ti_installers/` |
| Build cannot find `/opt/ti/...` | Tool installation did not complete or uses a custom root | Run `install-ti-tools-native` or pass `TI_ROOT=/custom/ti` |
| Link fails with L3 overflow | Ring or scratch allocation exceeds 768 KiB | Reduce frames, loops, TX count, or saved bins and inspect the map file |
| Probe receives no ROM response | Wrong CP2105 interface or RESET timing | Use Enhanced/UARTA, type `READY`, then RESET only when prompted |
| Flash fails after erase | Image transfer was interrupted | Leave flash mode enabled and rerun the full flash command; the ROM bootloader remains available |
| No CLI after flashing | Board remains in flash mode or was not reset | Restore functional switches and press RESET |
| Server rejects the config | Wrong firmware/config pair, invalid loop count/stride, or an L3 plan that does not fit | Use the v6 binary and `iwr6843_l3dump_vTX2_hybrid.cfg`; read the `Capture plan` or `Error` line |
| Default dump is not 783,508 bytes | The pre-ring was not full, the config was changed, the UART transfer was interrupted, or the wrong firmware is flashed | Check the v6 frame descriptors and resolved plan before treating the size difference as corruption |
| First run works but restart hangs | Retired v1 image or incomplete shutdown | Flash the current image and reset in functional mode |
| `docker-build` says the image does not exist | `docker-image` has not been run | `make -C firmware docker-image` (needs all five installers) |
| Container build fails "cannot execute 32-bit i386 binaries" | Docker Desktop is using its Rosetta backend | Turn off **Use Rosetta for x86_64/amd64 emulation** so QEMU handles i386 |
| Container build fails "page size is 16384" | Building on a Raspberry Pi | Build on the Mac or an x86_64 host; Pi OS uses 16 KiB pages |
| Container build fails "must be built for x86_64" | `--platform linux/amd64` was omitted | Use the `docker-image` target, which always passes it |
| `fetch-installers` reports "came back as ..., not ELF or ZIP" | TI served a login or export-control page | Download that file manually; see [The Installers](#the-installers) |

## Historical Context

The [IWR6843 field report](../docs/iwr6843_field_report_2026-07.html) explains
why the project moved capture into on-chip L3 and how the estimator evolved.
The implementation has since advanced from full raw ADC rings to HWA-generated,
dynamically windowed complex range snapshots; this README is the authoritative
description of the current firmware.
