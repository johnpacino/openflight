# LIS3DH Inclinometer

The optional LIS3DH measures enclosure pitch so OpenFlight can compensate the
configured IWR6843 mount tilt when the rig is not level.

## Raspberry Pi Setup

Enable I2C with `sudo raspi-config` under **Interface Options > I2C**, then
reboot. Wire the sensor while the Pi is powered off:

| LIS3DH | Raspberry Pi 40-pin header |
|---|---|
| `VIN` / `3V` | 3.3 V, physical pin 1 |
| `GND` | Ground, physical pin 6 |
| `SDA` | GPIO2 / SDA1, physical pin 3 |
| `SCL` | GPIO3 / SCL1, physical pin 5 |

The default I2C address is `0x18`. Verify the sensor before starting OpenFlight:

```bash
sudo apt install i2c-tools
i2cdetect -y 1
uv run python scripts/hardware-test/read_lis3dh.py --count 10
```

## Startup

Use the existing IWR tilt as the fixed radar-to-enclosure angle and enable the
inclinometer separately:

```bash
scripts/start-kiosk.sh \
  --iwr6843 \
  --iwr6843-tilt-deg 11.5 \
  --inclinometer \
  --inclinometer-zero-offset 1.5
```

The correction is:

```text
calibrated enclosure pitch = raw LIS3DH pitch + zero offset
effective IWR tilt = configured IWR tilt + calibrated enclosure pitch
```

Positive LIS3DH Y is treated as positive tilt-back. OpenFlight samples at 10 Hz
and uses the newest stable reading timestamped before impact. A missing, moving,
or stale sensor reading never suppresses a shot; the configured IWR tilt is used
without an enclosure correction and the reason is logged.

## Readout

```bash
uv run python scripts/hardware-test/read_lis3dh.py
```

Useful options include `--count`, `--interval`, `--bus`, and `--address`.

## Calibration

Place the enclosure on a stationary surface. If a phone reports that surface at
`0.1` degrees in the radar tilt direction, run:

```bash
uv run python scripts/hardware-test/calibrate_lis3dh.py --reference-pitch 0.1
```

The script averages 50 samples and prints the recommended
`--inclinometer-zero-offset` value. It does not modify configuration files.

## Session Data

The `session_start` configuration records sensor address, sampling rate, zero
offset, and startup status. Every `shot_detected` entry records the selected raw
and calibrated pitch, reading age, stability status, configured IWR tilt, and
effective IWR tilt.
