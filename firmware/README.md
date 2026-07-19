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
- **Flash: the Pi web flow / UniFlash.** Off-container; the `.bin` is portable.
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

The TX2 build is a capture proof: it enables TX1, TX2, and TX3 while reducing
the loop count to stay inside the 768 KiB L3 budget. Host-side horizontal/aim
estimation still needs to be implemented and validated before this becomes a
production UI feature.

### Copy firmware artifacts back to the Mac
From the Mac:

```bash
mkdir -p artifacts/firmware_build
rsync -av openflight@VM_ADDRESS:~/openflight/firmware/build_artifacts/ artifacts/firmware_build/
```

The `.bin` files are the flashable images. The `.xer4f` files are useful for
debugging/symbols but are not what you flash.

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
