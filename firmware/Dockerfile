# x86-64 build environment for the IWR6843 mmWave-SDK firmware.
#
# Runs NATIVELY on an Intel Mac's Docker (full speed) — build here on Stacey's
# 2017 Intel Mac. On Apple Silicon it works under `--platform=linux/amd64` but
# emulates x86 (slow). Flashing is done OFF-container (UniFlash / Pi web flow),
# so this image only builds the .bin.
FROM --platform=linux/amd64 ubuntu:22.04

# Keep apt fully non-interactive: one of the base deps pulls in tzdata, which
# otherwise blocks the build on a "select your geographic area" debconf prompt.
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# TI SDK installers are 32-bit-ish .bin self-extractors that want mono + i386 libs.
RUN dpkg --add-architecture i386 && apt-get update && apt-get install -y \
    --no-install-recommends \
      build-essential make mono-runtime \
      libc6:i386 libstdc++6:i386 zlib1g:i386 \
      bison flex unzip wget ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# --- TI toolchain (installers are user-provided in ti_installers/, git-ignored) ---
# Present now: mmWave SDK 3.6.02, ARM/R4F CGT (tms470) 20.2.7, SYS/BIOS 6.73.01.01,
# SysConfig 1.10.0, XDCtools 3.61.00.16. The DSP toolchain (C6000 CGT, DSPLIB, MATHLIB)
# is intentionally absent — l3_dump is MSS-only, so the first build targets the R4F.
COPY ti_installers/ /tmp/ti_installers/

# .bin/.run are InstallBuilder self-extractors (run headless via --mode unattended).
# Each tool installs into its OWN dir under /opt/ti so nothing collides regardless of
# how a given installer interprets --prefix. XDCtools ships as a .zip -> just unpack.
# The find at the end prints the REAL install paths -> read them back for products.mak.
#
# The SDK installer exits 1: AFTER its payload is on disk, its post-install step runs
# three nested DSP-lib installers (dsplib c64Px/c674x, mathlib c674x) that abort when
# headless. l3_dump is MSS-only (R4F) and never links those libs, so tolerate the exit
# and instead assert the SDK packages/ tree landed (the grep fails the build if not).
RUN set -eux; \
    cd /tmp/ti_installers; \
    chmod +x *.bin *.run; \
    ./mmwave_sdk_*Install.bin      --mode unattended --unattendedmodeui none --prefix /opt/ti/sdk \
      || echo "SDK installer exited nonzero (known DSP-lib post-install abort) — continuing"; \
    find /opt/ti/sdk -maxdepth 3 -type d -name packages | grep -q .; \
    rm -f /opt/ti/sdk/*_Linux.bin; \
    ./ti_cgt_tms470_*installer.bin --mode unattended --unattendedmodeui none --prefix /opt/ti/cgt-arm; \
    ./bios_*.run                   --mode unattended --unattendedmodeui none --prefix /opt/ti/bios; \
    ./sysconfig*.run               --mode unattended --unattendedmodeui none --prefix /opt/ti/sysconfig; \
    unzip -q xdctools_*.zip -d /opt/ti/xdc; \
    echo "=== installed dirs under /opt/ti ==="; find /opt/ti -maxdepth 2 -type d | sort

ENV TI_ROOT=/opt/ti
WORKDIR /work
CMD ["/bin/bash"]

# Typical build session (after installers are in and the image is built):
#   docker build -t iwr-sdk firmware/
#   docker run --rm -it -v "$PWD:/work" iwr-sdk
#   # inside: source $MMWAVE_SDK/packages/scripts/unix/setenv.sh
#   #         build the STOCK demo first (Phase 0), then the l3_dump app.
