#!/usr/bin/env python3
"""Print LIS3DH X/Y/Z acceleration and enclosure pitch."""

import argparse
import time

from openflight.inclinometer import LIS3DH
from openflight.inclinometer.service import pitch_degrees


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x18)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--count", type=int, default=0, help="Readings to print; 0 runs forever")
    args = parser.parse_args()

    sensor = LIS3DH(bus_number=args.bus, address=args.address)
    sensor.initialize()
    print(f"LIS3DH detected on I2C-{args.bus} at 0x{args.address:02x}")
    try:
        index = 0
        while args.count == 0 or index < args.count:
            sample = sensor.read()
            print(
                f"X={sample.x_g:+.3f}g  Y={sample.y_g:+.3f}g  "
                f"Z={sample.z_g:+.3f}g  Pitch={pitch_degrees(sample):+.2f}deg"
            )
            index += 1
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()


if __name__ == "__main__":
    main()
