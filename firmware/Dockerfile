# x86-64 build environment for the IWR6843 mmWave-SDK firmware.
#
# Runs NATIVELY on an Intel Mac's Docker (full speed) — build here on Stacey's
# 2017 Intel Mac. On Apple Silicon it works under `--platform=linux/amd64` but
# emulates x86 (slow). Flashing is done OFF-container (UniFlash / Pi web flow),
# so this image only builds the .bin.
FROM --platform=linux/amd64 ubuntu:22.04

# TI SDK installers are 32-bit-ish .bin self-extractors that want mono + i386 libs.
RUN dpkg --add-architecture i386 && apt-get update && apt-get install -y \
    --no-install-recommends \
      build-essential make mono-runtime \
      libc6:i386 libstdc++6:i386 zlib1g:i386 \
      bison flex unzip wget ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# --- YOU PROVIDE the TI installers (login-gated; cannot be baked into git) ---
# Download for LINUX x86 and drop the .bin files in firmware/ti_installers/:
#   * mmwave_sdk_<ver>_Linux-x86_Installer.bin      (SDK 3.6.x, IWR6843)
#   * ti-cgt-arm_<ver>, ti-cgt-c674x_<ver>          (R4F + DSP compilers)
#   * bios_<ver>, xdctools_<ver>, sysconfig_<ver>   (versions per SDK release notes)
#   * mmwave DFP, dsplib/mathlib as the SDK requires
COPY ti_installers/ /tmp/ti_installers/
# Uncomment once the installers are present:
# RUN chmod +x /tmp/ti_installers/*.bin && \
#     for f in /tmp/ti_installers/*.bin; do "$f" --mode unattended --prefix /opt/ti; done

ENV TI_ROOT=/opt/ti
# ENV MMWAVE_SDK=/opt/ti/mmwave_sdk_<ver>
WORKDIR /work
CMD ["/bin/bash"]

# Typical build session (after installers are in and the image is built):
#   docker build -t iwr-sdk firmware/
#   docker run --rm -it -v "$PWD:/work" iwr-sdk
#   # inside: source $MMWAVE_SDK/packages/scripts/unix/setenv.sh
#   #         build the STOCK demo first (Phase 0), then the l3_dump app.
