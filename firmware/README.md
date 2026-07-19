# IWR6843 Firmware Build Environment

OpenFlight uses custom TI IWR6843 firmware to keep a short rolling window of
raw ADC samples in on-chip L3 RAM. When the Pi receives the same sound trigger
used by OPS243, it asks the TI board to freeze and dump that L3 window over the
CLI UART. The Python runtime parses the dump and runs LCMF-v1.

The checked-in release binary is:

```text
firmware/l3_dump/releases/l3_dump_vB-16loops-12frames-20260713.bin
```

You only need this build environment if you want to rebuild or modify the TI
firmware. Normal OpenFlight users can flash the checked-in binary.

## What Is Checked In

- `firmware/l3_dump/l3_dump.c`: custom MSS/R4F L3 rolling-buffer app.
- `firmware/l3_dump/dump_format.h`: binary dump header shared with Python.
- `firmware/l3_dump/makefile`: out-of-tree mmWave SDK build.
- `firmware/l3_dump/mss.cfg`: SYS/BIOS config.
- `firmware/l3_dump/mss_linker.cmd`: MSS linker placement.
- `firmware/l3_dump/releases/*.bin`: known-good flashable release binaries.
- `firmware/Dockerfile`: x86 Linux build environment wrapper.

## What Is Not Checked In

TI SDKs and compiler installers are license-gated and large, so they are not
committed to git. Put them in:

```text
firmware/ti_installers/
```

That directory is ignored by git.

## Required TI Downloads

The current Dockerfile expects the same TI toolchain family used for the checked
in binary:

- mmWave SDK 3.6.x for IWR6843 / xWR68xx.
- TI ARM/R4F code generation tools, `ti-cgt-tms470`, tested with 20.2.7.
- SYS/BIOS, tested with 6.73.01.01.
- XDCtools, tested with 3.61.00.16.
- SysConfig, tested with 1.10.0.

The first L3 dump app is MSS-only, so the Dockerfile does not require the C674x
DSP compiler to build this firmware. If we later add DSS/DSP processing, that
will change.

Expected installer shape inside `firmware/ti_installers/`:

```text
mmwave_sdk_*Install.bin
ti_cgt_tms470_*installer.bin
bios_*.run
sysconfig*.run
xdctools_*.zip
```

## Build The Docker Image

From the repo root:

```bash
docker build -t openflight-iwr6843-fw firmware/
```

Notes:

- The image is `linux/amd64` because TI's mmWave SDK tooling is x86 Linux.
- Intel Macs and x86 Linux machines build fastest.
- Apple Silicon can build via emulation, but it is much slower.
- The mmWave SDK installer may exit non-zero after unpacking because some nested
  DSP library installers fail headless. The Dockerfile tolerates that as long as
  the SDK `packages/` directory exists.

## Build The Firmware

Start the container from the repo root:

```bash
docker run --rm -it -v "$PWD:/work" openflight-iwr6843-fw
```

Inside the container, locate the installed SDK and source its environment:

```bash
find /opt/ti -maxdepth 3 -type d -name packages | sort
export MMWAVE_SDK_INSTALL_PATH=/opt/ti/sdk/mmwave_sdk_03_06_02_00-LTS
source "$MMWAVE_SDK_INSTALL_PATH/packages/scripts/unix/setenv.sh"
```

If your SDK installed to a different path, use the path found by `find`.

Then build:

```bash
cd /work/firmware/l3_dump
make clean
make bin L3_VARIANT_DEFS="--define=LOOPS=16 --define=RING_FRAMES=12"
```

Expected output:

```text
firmware/l3_dump/l3_dump.bin
```

That `.bin` is the flashable meta-image.

## Firmware Variants

The validated release is Variant B:

```bash
make bin L3_VARIANT_DEFS="--define=LOOPS=16 --define=RING_FRAMES=12"
```

Variant B matches:

```text
config/iwr6843_l3dump_vB.cfg
config/iwr6843_l3dump_vBR.cfg
```

The `vBR` config reverses chirp/TX order for experiments. It does not require a
different binary; it requires the matching runtime `--iwr6843-tx-order reversed`
flag.

## Flashing

Flash the generated `.bin` with TI UniFlash or the Pi flashing flow used for the
IWR6843 board. After flashing, run OpenFlight with the matching config:

```bash
scripts/start-kiosk.sh --iwr6843 \
  --iwr6843-config config/iwr6843_l3dump_vB.cfg \
  --iwr6843-tx-order normal \
  --iwr6843-tee-m 1.575 \
  --iwr6843-net-m 4.6 \
  --iwr6843-tilt-deg 10.4 \
  --iwr6843-radar-height-m 0.1524 \
  --iwr6843-ball-height-m 0.040
```

## Runtime Contract

The firmware/Python contract is:

- Python sends the normal TI `.cfg` over the CLI UART.
- On each sound trigger, Python sends `l3dump\n`.
- Firmware returns `l3_dump_header_t` followed by interleaved int16 I/Q payload.
- Python parses the bytes with `openflight.iwr6843.dump.parse_dump`.
- Server/debug mode decides whether those bytes are also written as `.l3dump`
  files for offline replay.

Keep these in sync when changing firmware:

- Number of TX channels.
- Number of RX channels.
- Number of ADC samples.
- Chirps / loops per frame.
- Ring frame count.
- Header fields in `dump_format.h` and `src/openflight/iwr6843/dump.py`.

## Troubleshooting

- `MMWAVE_SDK_INSTALL_PATH` unset: source the SDK `setenv.sh` before running
  `make`.
- Missing installer in Docker build: confirm every required TI installer is in
  `firmware/ti_installers/`.
- Flash succeeds but OpenFlight cannot find the CLI port: power-cycle the board
  and confirm the custom single-port firmware is flashed.
- Runtime says config TX order conflicts: use `vB.cfg` with `normal`, or
  `vBR.cfg` with `reversed`.
