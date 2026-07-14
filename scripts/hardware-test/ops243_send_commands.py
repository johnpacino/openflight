#!/usr/bin/env python3
"""Send configuration commands to an OPS243 radar over USB, then save.

Made to be run by a non-technical user:

    1. Plug the OPS243 into the computer with a USB cable.
    2. Install the one dependency (once):   pip3 install pyserial
    3. Run:                                 python3 ops243_send_commands.py

It finds the radar automatically (Mac/Linux/Windows), sends D4 then I4,
saves with A!, and prints everything the radar says. Different commands:

    python3 ops243_send_commands.py --commands D4 I4 --save
"""
from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("The 'pyserial' package is missing. Install it with:")
    print("    pip3 install pyserial")
    sys.exit(1)


def find_radar() -> str | None:
    """Best-guess the OPS243's serial port on any OS."""
    candidates = []
    for port in list_ports.comports():
        text = f"{port.device} {port.description} {port.manufacturer or ''}"
        if any(tag in text for tag in ("OmniPreSense", "OPS", "ACM", "usbmodem")):
            candidates.append(port.device)
    if not candidates:   # fall back to anything that looks like USB serial
        candidates = [p.device for p in list_ports.comports()
                      if "USB" in p.device.upper() or "COM" in p.device.upper()]
    return candidates[0] if candidates else None


def send(ser: serial.Serial, command: str) -> str:
    """Send one command and return whatever the radar replies."""
    ser.reset_input_buffer()
    ser.write(command.encode())
    time.sleep(0.6)
    reply = ser.read(ser.in_waiting or 1).decode(errors="replace").strip()
    return reply


def main() -> int:
    ap = argparse.ArgumentParser(description="Configure an OPS243 over USB")
    ap.add_argument("--commands", nargs="+", default=["D4", "I4"],
                    help="commands to send, in order (default: D4 I4)")
    ap.add_argument("--save", action="store_true", default=True,
                    help="save settings to the radar's memory afterward (A!)")
    ap.add_argument("--port", help="serial port (default: find automatically)")
    args = ap.parse_args()

    port = args.port or find_radar()
    if not port:
        print("Could not find the radar. Is the USB cable plugged in?")
        print("(If it is, unplug it, wait 3 seconds, plug it back in, retry.)")
        return 1
    print(f"Radar found on {port}")

    try:
        ser = serial.Serial(port, 19200, timeout=0.5)
    except (OSError, serial.SerialException) as err:
        print(f"Could not open {port}: {err}")
        print("Close any other program using the radar and try again.")
        return 1

    hello = send(ser, "??")
    if hello:
        print(f"Radar says:\n{hello}\n")

    for command in args.commands:
        reply = send(ser, command)
        print(f"sent {command:<4} -> {reply or '(no reply)'}")

    if args.save:
        reply = send(ser, "A!")
        print(f"sent A!   -> {reply or '(no reply)'}")
        print("\nSettings saved to the radar's permanent memory." )
        print("You can unplug it now — the settings survive power-off.")
    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
