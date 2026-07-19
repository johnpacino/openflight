IMAGE ?= iwr-sdk-pi
DOCKER ?= docker
PLATFORM ?= linux/amd64
INSTALLER_DIR ?= ti_installers
STACEY_HOST ?= staceypacino@staceys-macbook-pro.local
STACEY_INSTALLER_DIR ?= /Users/staceypacino/Projects/openflight/firmware/ti_installers
TI_ROOT ?= /opt/ti
MMWAVE_SDK_INSTALL_PATH ?= $(TI_ROOT)/sdk/mmwave_sdk_03_06_02_00-LTS/packages
R4F_CODEGEN_INSTALL_PATH ?= $(TI_ROOT)/cgt-arm/ti-cgt-arm_20.2.7.LTS
XDC_INSTALL_PATH ?= $(TI_ROOT)/xdc/xdctools_3_61_00_16_core
BIOS_INSTALL_PATH ?= $(TI_ROOT)/bios/bios_6_73_01_01/packages
SYSCONFIG_INSTALL_PATH ?= $(TI_ROOT)/sysconfig
XWR68XX_RADARSS_IMAGE_BIN ?= $(TI_ROOT)/sdk/mmwave_sdk_03_06_02_00-LTS/firmware/radarss/xwr6xxx_radarss_rprc.bin
L3_DIR ?= l3_dump
BUILD_ARTIFACT_DIR ?= build_artifacts

SDK_INSTALLER := $(INSTALLER_DIR)/mmwave_sdk_03_06_02_00-LTS-Linux-x86-Install.bin
R4F_INSTALLER := $(INSTALLER_DIR)/ti_cgt_tms470_20.2.7.LTS_linux-x64_installer.bin
BIOS_INSTALLER := $(INSTALLER_DIR)/bios_6_73_01_01.run
SYSCONFIG_INSTALLER := $(INSTALLER_DIR)/sysconfig-1.10.0_2163-setup.run
XDC_ZIP := $(INSTALLER_DIR)/xdctools_3_61_00_16_core_linux.zip

BUILD_ENV := \
	MMWAVE_SDK_DEVICE=iwr68xx \
	DOWNLOAD_FROM_CCS=yes \
	MMWAVE_SDK_INSTALL_PATH=$(MMWAVE_SDK_INSTALL_PATH) \
	R4F_CODEGEN_INSTALL_PATH=$(R4F_CODEGEN_INSTALL_PATH) \
	XDC_INSTALL_PATH=$(XDC_INSTALL_PATH) \
	BIOS_INSTALL_PATH=$(BIOS_INSTALL_PATH) \
	XWR68XX_RADARSS_IMAGE_BIN=$(XWR68XX_RADARSS_IMAGE_BIN)

REQUIRED_INSTALLERS := \
	mmwave_sdk_03_06_02_00-LTS-Linux-x86-Install.bin \
	ti_cgt_tms470_20.2.7.LTS_linux-x64_installer.bin \
	bios_6_73_01_01.run \
	sysconfig-1.10.0_2163-setup.run \
	xdctools_3_61_00_16_core_linux.zip

.PHONY: help check-host check-installers sync-installers-from-stacey install-ti-deps-native install-ti-tools-native probe-installers-native build-vb-native build-debian-base build-debian probe-installers shell-debian clean-image

help:
	@echo "IWR6843 firmware build helpers"
	@echo ""
	@echo "Targets:"
	@echo "  check-host                 Show host arch, Docker, and binfmt status"
	@echo "  check-installers           Verify TI installer files exist locally"
	@echo "  sync-installers-from-stacey Copy TI installers from Stacey's Intel Mac"
	@echo "  install-ti-deps-native     apt install packages needed on x86 Debian"
	@echo "  install-ti-tools-native    Install TI SDK/toolchain into /opt/ti"
	@echo "  probe-installers-native    Test TI installer stubs on this host"
	@echo "  build-vb-native            Build current Variant B firmware"
	@echo "  build-debian-base          Build Pi/QEMU-friendly base image only"
	@echo "  build-debian               Build Pi/QEMU-friendly Debian amd64 image"
	@echo "  probe-installers           Test TI installer stubs under the base image"
	@echo "  shell-debian               Open a shell in the built image with repo mounted"
	@echo "  clean-image                Remove the built Docker image"
	@echo ""
	@echo "Examples:"
	@echo "  make -C firmware sync-installers-from-stacey"
	@echo "  make -C firmware install-ti-deps-native"
	@echo "  make -C firmware install-ti-tools-native"
	@echo "  make -C firmware build-vb-native"
	@echo "  make -C firmware build-debian-base"
	@echo "  make -C firmware probe-installers"
	@echo "  make -C firmware build-debian"
	@echo "  make -C firmware shell-debian"

check-host:
	@set -eu; \
	echo "host_arch=$$(uname -m)"; \
	echo "page_size=$$(getconf PAGE_SIZE)"; \
	$(DOCKER) --version; \
	$(DOCKER) buildx version || true; \
	if [ -d /proc/sys/fs/binfmt_misc ]; then \
		echo "binfmt_x86_64=$$(test -e /proc/sys/fs/binfmt_misc/qemu-x86_64 && echo yes || echo no)"; \
	fi

check-installers:
	@set -eu; \
	missing=0; \
	for file in $(REQUIRED_INSTALLERS); do \
		if [ ! -f "$(INSTALLER_DIR)/$$file" ]; then \
			echo "missing: $(INSTALLER_DIR)/$$file"; \
			missing=1; \
		else \
			echo "found:   $(INSTALLER_DIR)/$$file"; \
		fi; \
	done; \
	exit $$missing

sync-installers-from-stacey:
	@mkdir -p "$(INSTALLER_DIR)"
	rsync -av --progress "$(STACEY_HOST):$(STACEY_INSTALLER_DIR)/" "$(INSTALLER_DIR)/"

install-ti-deps-native:
	sudo apt-get update
	sudo apt-get install -y build-essential make mono-runtime libc6:i386 libstdc++6:i386 zlib1g:i386 bison flex unzip rsync file

probe-installers-native: check-installers
	@set -eu; \
	chmod +x "$(SDK_INSTALLER)" "$(R4F_INSTALLER)" "$(BIOS_INSTALLER)" "$(SYSCONFIG_INSTALLER)"; \
	for installer in "$(SDK_INSTALLER)" "$(R4F_INSTALLER)" "$(BIOS_INSTALLER)" "$(SYSCONFIG_INSTALLER)"; do \
		echo "=== $$installer --help ==="; \
		"$$installer" --help >/tmp/installer-help.txt 2>&1; \
		rc=$$?; \
		echo "exit=$$rc"; \
		sed -n '1,24p' /tmp/installer-help.txt; \
	done

install-ti-tools-native: check-installers
	sudo mkdir -p "$(TI_ROOT)/sdk" "$(TI_ROOT)/cgt-arm" "$(TI_ROOT)/bios" "$(TI_ROOT)/xdc" "$(SYSCONFIG_INSTALL_PATH)"
	chmod +x "$(SDK_INSTALLER)" "$(R4F_INSTALLER)" "$(BIOS_INSTALLER)" "$(SYSCONFIG_INSTALLER)"
	sudo "$(SDK_INSTALLER)" --mode unattended --prefix "$(TI_ROOT)/sdk" --disable-components TI_CGT_C6000,TI_CGT_ARM,DSPLIB_C64Px,DSPLIB_C674x,MATHLIB_C674x,XDCtools,SYS_BIOS
	sudo "$(R4F_INSTALLER)" --mode unattended --prefix "$(TI_ROOT)/cgt-arm"
	sudo "$(BIOS_INSTALLER)" --mode unattended --prefix "$(TI_ROOT)/bios"
	sudo "$(SYSCONFIG_INSTALLER)" --mode unattended --prefix "$(SYSCONFIG_INSTALL_PATH)"
	sudo unzip -q -o "$(XDC_ZIP)" -d "$(TI_ROOT)/xdc"
	test -d "$(MMWAVE_SDK_INSTALL_PATH)"
	test -x "$(R4F_CODEGEN_INSTALL_PATH)/bin/armcl"
	test -d "$(BIOS_INSTALL_PATH)"
	test -x "$(XDC_INSTALL_PATH)/xdc"
	test -f "$(XWR68XX_RADARSS_IMAGE_BIN)"

build-vb-native:
	$(BUILD_ENV) $(MAKE) -C "$(L3_DIR)" clean
	$(BUILD_ENV) $(MAKE) -C "$(L3_DIR)" bin L3_VARIANT_DEFS="--define=LOOPS=16 --define=RING_FRAMES=12"
	mkdir -p "$(BUILD_ARTIFACT_DIR)"
	cp "$(L3_DIR)/l3_dump.bin" "$(BUILD_ARTIFACT_DIR)/l3_dump_vB_2tx_16loops_12frames.bin"
	cp "$(L3_DIR)/l3_dump_mss.xer4f" "$(BUILD_ARTIFACT_DIR)/l3_dump_vB_2tx_16loops_12frames_mss.xer4f"

build-debian-base:
	$(DOCKER) build --platform "$(PLATFORM)" --target base -t "$(IMAGE)-base" -f Dockerfile.debian .

build-debian: check-installers
	$(DOCKER) build --platform "$(PLATFORM)" --target ti -t "$(IMAGE)" -f Dockerfile.debian .

probe-installers: check-installers build-debian-base
	$(DOCKER) run --rm --platform "$(PLATFORM)" -v "$$(pwd)/$(INSTALLER_DIR):/ti:ro" "$(IMAGE)-base" /bin/bash -lc '\
		set -u; \
		mkdir -p /tmp/ti; \
		cp /ti/* /tmp/ti/; \
		chmod +x /tmp/ti/*.bin /tmp/ti/*.run; \
		cd /tmp/ti; \
		for installer in \
			mmwave_sdk_03_06_02_00-LTS-Linux-x86-Install.bin \
			ti_cgt_tms470_20.2.7.LTS_linux-x64_installer.bin \
			bios_6_73_01_01.run \
			sysconfig-1.10.0_2163-setup.run; do \
			echo "=== $$installer --help ==="; \
			./$$installer --help >/tmp/installer-help.txt 2>&1; \
			rc=$$?; \
			echo "exit=$$rc"; \
			sed -n "1,24p" /tmp/installer-help.txt; \
		done'

shell-debian:
	$(DOCKER) run --rm -it --platform "$(PLATFORM)" -v "$$(cd .. && pwd):/work" "$(IMAGE)"

clean-image:
	-$(DOCKER) rmi "$(IMAGE)"
	-$(DOCKER) rmi "$(IMAGE)-base"
