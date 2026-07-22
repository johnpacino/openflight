# Stage-1c firmware — L3 raw-ADC burst-dump (IWR6843)

Custom firmware that buffers raw ADC in the chip's L3 and streams a burst over
UART when the Pi sends `l3dump` (at the sound-trigger edge). This is the
productizable alternative to the DCA1000 — Pi + TI + sound trigger, no LVDS, no
lab capture card. Full plan: [../docs/plans/stage1c_l3_burst_dump.md](../docs/plans/stage1c_l3_burst_dump.md).

## Where to build vs flash
- **Build: x86 Linux.** The TI SDK/toolchain installers are x86 Linux/Windows
  only. On Apple Silicon, the proven local path is an x86_64 Debian VM in UTM.
  On Intel Macs/Linux boxes, build natively. On Raspberry Pi 5, Docker/QEMU can
  be used for smoke tests, but the TI 32-bit installer stubs can fail under the
  Pi 5 16 KB page-size kernel.
- **Flash: directly from the Pi or with UniFlash.** The checked-in Python
  flasher uses TI's ROM UART protocol and does not require TI Cloud Agent.
- **Run: the Pi** drives it via [`../iwr6843_runtime.py`](../iwr6843_runtime.py).

## Apple Silicon Mac: UTM AMD64/x86_64 Debian VM
This is the recommended no-cloud build path for contributors on M1/M2/M3 Macs.
The TI firmware tools are not ARM-native, so the trick is to run an AMD64
Debian VM in UTM. This was validated with Debian 13 x86_64, 4 GB RAM, and a
30 GB virtual disk.

### What you need
- UTM installed on the Mac.
- Debian amd64 ISO, for example the Debian `amd64 netinst` image.
- The OpenFlight repo inside the VM.
- The TI installer files in `firmware/ti_installers/`.

Expected TI installer files:

```text
firmware/ti_installers/mmwave_sdk_03_06_02_00-LTS-Linux-x86-Install.bin
firmware/ti_installers/ti_cgt_tms470_20.2.7.LTS_linux-x64_installer.bin
firmware/ti_installers/bios_6_73_01_01.run
firmware/ti_installers/sysconfig-1.10.0_2163-setup.run
firmware/ti_installers/xdctools_3_61_00_16_core_linux.zip
```

These installers are intentionally not checked into git because they are large
and TI-license-gated.

### Create the VM
In UTM:

1. Choose `Create a New Virtual Machine`.
2. Choose `Emulate`, not virtualize. Apple Silicon cannot virtualize AMD64
   directly.
3. Choose `Linux`.
4. Hardware:
   - Machine: `Intel ICH9 based PC (2009, x86_64)`
   - Memory: `4096 MiB` minimum
   - CPU cores: default is fine
   - Display output: enabled is fine
5. Boot image:
   - Select the Debian amd64 ISO.
6. Storage:
   - `30 GiB` minimum
7. Shared directory:
   - Optional. SSH/rsync is usually more reliable than UTM shared folders.

During Debian install:

- Use the normal text `Install` option.
- Create a regular user.
- Install `SSH server` and `standard system utilities`.
- A desktop environment is not required. If prompted, you can uncheck GNOME.
- After install, remove/eject the ISO from the VM CD/DVD drive before rebooting.

### Prepare the VM
Log into the VM and confirm it is the right architecture:

```bash
uname -m
getconf PAGE_SIZE
```

Expected:

```text
x86_64
4096
```

Install git and clone OpenFlight, or copy the repo into the VM:

```bash
sudo apt-get update
sudo apt-get install -y git rsync openssh-server
git clone https://github.com/jewbetcha/openflight.git
cd openflight
```

If you are copying from your Mac instead of cloning:

```bash
rsync -av ~/Projects/openflight/ openflight@VM_ADDRESS:~/openflight/
```

Replace `VM_ADDRESS` with the VM IP or hostname. You can find it inside the VM
with:

```bash
ip addr
```

### Copy the TI installers
Place the TI installers here inside the VM:

```text
~/openflight/firmware/ti_installers/
```

If the installers are already on another machine, copy them with `rsync`:

```bash
mkdir -p ~/openflight/firmware/ti_installers
rsync -av user@OTHER_MACHINE:/path/to/ti_installers/ ~/openflight/firmware/ti_installers/
```

Then verify the repo can see them:

```bash
make -C firmware check-installers
```

### Install build dependencies
Run this inside the VM from the repo root:

```bash
make -C firmware install-ti-deps-native
```

### Probe the TI installers
Before installing the full toolchain, make sure the installer stubs can run in
the VM:

```bash
make -C firmware probe-installers-native
```

Each installer should print help text and exit successfully. If this works in
the VM, you have avoided the Raspberry Pi 5 page-size/QEMU installer problem.

### Install the TI toolchain
Install the SDK/toolchain into `/opt/ti`:

```bash
make -C firmware install-ti-tools-native
```

This installs:

```text
/opt/ti/sdk/mmwave_sdk_03_06_02_00-LTS
/opt/ti/cgt-arm/ti-cgt-arm_20.2.7.LTS
/opt/ti/bios/bios_6_73_01_01
/opt/ti/xdc/xdctools_3_61_00_16_core
/opt/ti/sysconfig
```

### Build Variant B
Build the current production capture firmware:

```bash
make -C firmware build-vb-native
```

That writes:

```text
firmware/build_artifacts/l3_dump_vB_2tx_16loops_12frames.bin
firmware/build_artifacts/l3_dump_vB_2tx_16loops_12frames_mss.xer4f
```

This is the current LCMF capture firmware:

```text
2 TX x 16 loops x 12 frames
```

### Build the TX2/3TX proof firmware
Build the experimental firmware that adds TX2:

```bash
make -C firmware build-tx2-native
```

That writes:

```text
firmware/build_artifacts/l3_dump_vTX2_3tx_10loops_12frames.bin
firmware/build_artifacts/l3_dump_vTX2_3tx_10loops_12frames_mss.xer4f
```

The TX2 build enables TX1, TX2, and TX3 while reducing the loop count to stay
inside the 768 KiB L3 budget. OpenFlight can decode the three-transmitter dump
and produce an experimental horizontal/aim estimate. That estimate still needs
broader source-of-truth calibration and coverage validation before it becomes
a production-quality metric.

### Copy firmware artifacts back to the Mac
From the Mac:

```bash
mkdir -p artifacts/firmware_build
rsync -av openflight@VM_ADDRESS:~/openflight/firmware/build_artifacts/ artifacts/firmware_build/
```

The `.bin` files are the flashable images. The `.xer4f` files are useful for
debugging/symbols but are not what you flash.

## Flash from the Raspberry Pi

The Pi can flash an IWR6843 `.bin` directly over the CP2105 **Enhanced** UART.
This replaces the Intel Mac, UniFlash, and TI Cloud Agent. The current LEVM's
boot-mode switch and reset button are still manual; a future board can wire
SOP2 and reset to Pi GPIO for a fully unattended update.

This workflow was hardware-validated on a Raspberry Pi 5 and IWR6843LEVM on
July 20, 2026, including full erase, a 339,268-byte image transfer, image close,
and ROM bootloader verification.

Before starting, stop OpenFlight so it releases the TI serial port. Connect the
TI board's USB cable to the Pi and identify its two CP2105 ports:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB*
```

Use the **Enhanced/UARTA** port, normally `/dev/ttyUSB0` and USB interface
number `00`. Do not use the Standard/data port, normally `/dev/ttyUSB1`. For
the first Pi test, use the non-destructive probe. It performs only the
UART-break handshake and `PING`; it does not erase or write flash:

```bash
uv run python firmware/flash_iwr6843.py --probe --port /dev/ttyUSB0
```

After the probe passes, run the actual flash from the repository root:

```bash
uv run python firmware/flash_iwr6843.py \
  firmware/build_artifacts/l3_dump_vTX2_hwa_frame_ring_v1.bin \
  --port /dev/ttyUSB0
```

The script displays the image size and SHA-256, then prompts for the physical
board steps:

1. Flash mode: `S1.1 ON, S1.2 OFF, S1.3 ON, S1.4 ON, S1.5 OFF`.
2. Type `READY` so the script opens UART and settles its control lines.
3. Only when the script asks, press and release `RESET`, then wait one second.
4. Type `FLASH` at the prompt.
5. The default workflow performs a full SFLASH erase, opens the new image,
   writes acknowledged 240-byte chunks, closes the image, and checks the final
   ROM bootloader status. Erasing can take longer than ten seconds; the script
   allows up to two minutes. Do not reset or disconnect the board while it is
   erasing or writing.
6. Functional mode: `S1.1 OFF, S1.2 OFF, S1.3 ON, S1.4 ON, S1.5 OFF`.
7. Press and release `RESET` again.

A successful transfer looks like:

```text
Erasing existing SFLASH...
Opening firmware image...
Writing firmware...
Writing: 100% (.../... bytes)
Closing and verifying firmware...

Flash verified by the IWR6843 ROM bootloader.
```

The flasher uses the protocol documented in TI application note
[SWRA627, IWR6843 Bootloader Flow](https://www.ti.com/lit/an/swra627/swra627.pdf):
115200-baud UARTA, UART-break handshake, `OPEN`, acknowledged `WRITE TO FLASH`
chunks, `CLOSE`, and final status validation. BREAK responses from the CP2105
can contain leading zero bytes; the flasher skips that preamble before
validating the bootloader ACK.

If a transfer fails, leave the board in flash mode and rerun the command using
the normal `READY` → `RESET` → one-second wait → `FLASH` sequence. A failure
after `Erasing existing SFLASH...` may mean the previous application image is
already gone, even if no write percentage appeared. This is recoverable: the
immutable ROM bootloader remains available in flash mode, so retry the complete
flash rather than attempting to boot in functional mode.

Safety notes:

- Do not flash while `scripts/start-kiosk.sh`, `shot_test.py`, or another
  serial process is running.
- Keep the TI board on stable USB power for the complete operation.
- Do not use `--yes` until the interactive workflow has been proven on your
  setup.
- The default intentionally matches UniFlash by erasing SFLASH first.
  `--no-erase` is available for controlled development use, but a complete
  erase is the safer general-purpose workflow.

### Common issues
- If the VM boots back into the Debian installer after installation, eject or
  remove the ISO from the UTM CD/DVD drive and reboot.
- If `sudo` asks for a password, use the password created during Debian install.
- If `make -C firmware probe-installers-native` fails, confirm the VM is
  `x86_64`, not `aarch64`, and that the installer files are executable.
- If `make -C firmware build-vb-native` cannot find `/opt/ti/...`, run
  `make -C firmware install-ti-tools-native` first.
- If building on the Raspberry Pi fails in a Docker AMD64 image, use this UTM
  VM path instead. The Pi 5 can hit 32-bit TI installer crashes under QEMU.

## Raspberry Pi 5 Debian build path
Install Docker and amd64 emulation once:

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-buildx qemu-user-static binfmt-support
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

Copy the TI installers into `firmware/ti_installers/`:

```bash
make -C firmware sync-installers-from-stacey
```

Build the Pi/QEMU-friendly Debian amd64 image:

```bash
make -C firmware build-debian-base
make -C firmware probe-installers
make -C firmware build-debian
```

Open a build shell:

```bash
make -C firmware shell-debian
```

The Debian image intentionally avoids `ca-certificates` and `wget` because Pi 5
16 KB page-size kernels can make some amd64 OpenSSL binaries fail under QEMU.
The image only needs local TI installers, so network tools inside the container
are unnecessary.

Current Pi 5 status: the Debian base image builds and the 64-bit TI installer
stubs run under QEMU. The 32-bit static mmWave SDK and SYS/BIOS installers can
segfault on `--help` on 16 KB page-size Pi kernels. If that happens,
`make -C firmware probe-installers` will show it before the full image build.
Use the UTM x86_64 Debian VM path above, an Intel machine, or an x86 CI runner
for the full toolchain install.

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
`l3_dump/` contains the custom capture firmware and its wire-format contract:
- `dump_format.h` — the 20-byte header + payload layout, byte-for-byte matching
  `iwr6843_l3dump.py` (`parse_dump`). **One format, two languages — keep in sync.**
- `l3_dump.c` — raw-ADC and HWA range-snapshot capture variants, circular L3
  buffering, `l3dump` freeze/stream/rearm behavior, and diagnostics.

Build → `.bin` → flash → the Pi's `iwr6843_runtime.py` runs it end-to-end (the Pi
side is already implemented + tested: `iwr6843_l3dump.py`, `iwr6843_runtime.py`).

## Contract with the Pi (already implemented Pi-side)
| | |
|---|---|
| Trigger command | Pi writes `l3dump\n` to the CLI UART on the sound-trigger GPIO edge |
| Dump | `l3_dump_header_t` + int16 I/Q, parsed by `iwr6843_l3dump.parse_dump` |
| Keep in sync | `N_TX` / `N_RX` / `N_SAMPLES` / `LOOPS` across the `.cfg`, `l3_dump.c`, and the Pi meta |

## Sizing (tune in `l3_dump.c`)
The current runtime budget is the IWR6843 on-chip L3 capture buffer. Variant B
uses:

```text
2 TX x 16 loops x 12 frames x 4 RX x 128 samples x 4 bytes = 786,432 bytes
```

Fewer loops, fewer range samples, or selected range-bin snapshots buy room for
more transmitters or more frames in future firmware variants.

The TX2 proof uses:

```text
3 TX x 10 loops x 12 frames x 4 RX x 128 samples x 4 bytes = 737,280 bytes
```

The HWA frame-ring prototype performs each 128-point range FFT on-chip and
retains only bins 20–99. It keeps the same 3 TX, 10 loops, 12 frames, 4 RX,
and 6 ms frame period while reducing the L3 payload to:

```text
3 TX x 10 loops x 12 frames x 4 RX x 80 complex bins x 4 bytes = 460,800 bytes
```

The ring advances one completed frame at a time. When the Pi sends `l3dump`,
the firmware records the request, retains eight additional frames, freezes at
that completed-frame boundary, rotates the circular ring into chronological
order, and streams it. With a full 12-frame ring this targets approximately
four pre-impact and eight impact/post-impact frames. The production server
always requests the ring immediately at the sound edge; post-trigger frame
placement belongs in firmware, not in a host-side delay.

Build this variant in the x86 Debian environment with:

```bash
make -C firmware build-tx2-hwa-frame-ring-native
```

Artifacts:

```text
firmware/build_artifacts/l3_dump_vTX2_hwa_frame_ring_v1.bin
firmware/build_artifacts/l3_dump_vTX2_hwa_frame_ring_v1_mss.xer4f
```

The validated dynamic-window firmware keeps 53 bins per frame and shifts the
saved range interval from the tee toward the net as the post-trigger frames
arrive. The baseline build retains 10 loops, 12 frames, and 6 ms spacing:

```bash
make -C firmware build-tx2-hwa-window53-native
```

The higher-density test build uses 12 loops, 18 frames, and 4 ms spacing. It
retains six pre-trigger and twelve post-trigger frames while using 549,504 of
the available 786,432 L3 bytes:

```bash
make -C firmware build-tx2-hwa-window53-12l18f-native
```

Use the matching runtime configuration:

```text
config/iwr6843_l3dump_vTX2_window53_12l18f.cfg
```

The flashable artifact is:

```text
firmware/build_artifacts/l3_dump_vTX2_hwa_window53_12loops_18frames_4ms_v2.bin
```

Version 2 keeps the same radar geometry and stored frame format as version 1.
It fixes repeated application startup by stopping the RF front end only after
the active HWA frame reaches a safe boundary, before disabling HWA and EDMA.
