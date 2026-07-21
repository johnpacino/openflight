#!/usr/bin/env python3
"""Flash an IWR6843 meta image from Linux using the ROM UART bootloader."""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

import serial

from openflight.iwr6843.bootloader import BootloaderError, IWR6843Bootloader

BAUD = 115_200
FLASH_SWITCH = "S1.1 ON, S1.2 OFF, S1.3 ON, S1.4 ON, S1.5 OFF"
RUN_SWITCH = "S1.1 OFF, S1.2 OFF, S1.3 ON, S1.4 ON, S1.5 OFF"


def open_serial(port: str) -> serial.Serial:
    """Open UARTA without allowing another process to share the device."""
    handle = serial.Serial()
    handle.port = port
    handle.baudrate = BAUD
    handle.timeout = 10.0
    handle.write_timeout = 10.0
    handle.exclusive = True
    handle.dtr = False
    handle.rts = False
    handle.open()
    return handle


def confirm_prepared() -> bool:
    """Confirm the board switch before opening its UART."""
    print("\nPrepare the IWR6843LEVM:")
    print("  1. Stop OpenFlight and any program using the TI serial port.")
    print(f"  2. Set flash mode: {FLASH_SWITCH}.")
    print("  3. Do not press RESET yet; the script must open UART first.")
    return input("\nType READY to open the serial port: ").strip() == "READY"


def confirm_bootloader(action: str) -> bool:
    """Ask for reset after UART is open, then authorize the operation."""
    print("\nSerial port is open and its control lines are settled.")
    print("Press and release RESET now, then wait one second.")
    expected = "PROBE" if action == "probe" else "FLASH"
    return input(f"\nType {expected} to begin: ").strip() == expected


def main() -> int:
    """Run the guarded command-line flashing workflow."""
    parser = argparse.ArgumentParser(
        description="Flash an IWR6843 .bin from the Pi without UniFlash."
    )
    parser.add_argument("image", type=Path, nargs="?", help="TI MSTR meta-image .bin")
    parser.add_argument(
        "--port",
        required=True,
        help="CP2105 Enhanced/UARTA port, normally /dev/ttyUSB0",
    )
    parser.add_argument(
        "--no-erase",
        action="store_true",
        help="skip TI's optional full serial-flash erase",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the interactive FLASH confirmation",
    )
    parser.add_argument(
        "--probe",
        action="store_true",
        help="test the bootloader handshake without reading or writing flash",
    )
    args = parser.parse_args()

    if args.probe:
        print(f"Port: {args.port} at {BAUD:,} baud")
        if not args.yes and not confirm_prepared():
            print("Probe cancelled.")
            return 2
        try:
            with open_serial(args.port) as handle:
                bootloader = IWR6843Bootloader(handle)
                if not args.yes and not confirm_bootloader("probe"):
                    print("Probe cancelled.")
                    return 2
                bootloader.handshake()
        except (BootloaderError, OSError, serial.SerialException) as error:
            print(f"Bootloader probe failed: {error}", file=sys.stderr)
            return 1
        print("IWR6843 ROM bootloader handshake: PASS")
        print(f"Return to functional mode: {RUN_SWITCH}, then press RESET.")
        return 0

    if args.image is None:
        parser.error("image is required unless --probe is used")
    try:
        image = args.image.read_bytes()
    except OSError as error:
        parser.error(str(error))
    if not image.startswith(b"MSTR"):
        parser.error(f"{args.image} is not a TI MSTR meta-image")

    digest = hashlib.sha256(image).hexdigest()
    print(f"Image:  {args.image.resolve()}")
    print(f"Size:   {len(image):,} bytes")
    print(f"SHA256: {digest}")
    print(f"Port:   {args.port} at {BAUD:,} baud")

    if not args.yes and not confirm_prepared():
        print("Flash cancelled.")
        return 2

    last_percent = -1

    def show_progress(sent: int, total: int) -> None:
        nonlocal last_percent
        percent = sent * 100 // total
        if percent != last_percent:
            print(f"\rWriting: {percent:3d}% ({sent:,}/{total:,} bytes)", end="", flush=True)
            last_percent = percent

    def show_stage(message: str) -> None:
        if last_percent >= 0:
            print()
        print(f"{message}...", flush=True)

    try:
        with open_serial(args.port) as handle:
            bootloader = IWR6843Bootloader(handle)
            if not args.yes and not confirm_bootloader("flash"):
                print("Flash cancelled.")
                return 2
            bootloader.handshake()
            bootloader.flash(
                image,
                erase=not args.no_erase,
                progress=show_progress,
                handshake=False,
                stage=show_stage,
            )
    except (BootloaderError, OSError, serial.SerialException) as error:
        if last_percent >= 0:
            print()
        print(f"Flash failed: {error}", file=sys.stderr)
        print("Leave the board in flash mode, press RESET, and retry.", file=sys.stderr)
        return 1

    print("\nFlash verified by the IWR6843 ROM bootloader.")
    print("\nReturn the board to normal operation:")
    print(f"  1. Set functional mode: {RUN_SWITCH}.")
    print("  2. Press and release RESET.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
