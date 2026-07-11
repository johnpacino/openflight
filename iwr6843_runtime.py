"""Pi-side runtime for the Stage-1c L3 burst-dump: sound trigger -> dump -> angle.

Flow (docs/plans/stage1c_l3_burst_dump.md):
  SEN-14262 impact -> Pi GPIO edge -> send DUMP_CMD on the TI CLI UART ->
  firmware freezes its L3 rolling buffer, grabs the post-trigger window, and
  streams one burst on the TI data UART -> parse -> range-FFT -> per-frame
  elevation via FBSS-MUSIC.

The testable core is read_dump() (frame the burst by the header's byte count)
and process_dump() (bytes -> per-frame angles). run_loop() is the thin, Pi-only
GPIO+serial glue and is not unit-tested (smoke-tested on hardware).
"""
from __future__ import annotations

import time

import iwr6843_l3dump as l3

# firmware<->Pi command contract: this CLI command tells the firmware to freeze
# the L3 rolling buffer, capture the post-trigger frames, and stream one dump.
DUMP_CMD = b"l3dump\n"


def read_dump(read_fn, *, timeout_s: float = 8.0) -> bytes:
    """Assemble exactly one dump: fixed header, then payload sized from it.

    read_fn(n) -> up to n bytes (e.g. serial.Serial.read). Blocks until the full
    dump is read or timeout_s elapses. The generous default covers the firmware's
    post-trigger capture wait before it starts streaming.
    """
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while len(buf) < l3.HEADER.size:
        if time.monotonic() > deadline:
            raise TimeoutError(f"dump header timeout ({len(buf)}/{l3.HEADER.size} B)")
        chunk = read_fn(l3.HEADER.size - len(buf))
        if chunk:
            buf += chunk
    need = l3.HEADER.size + l3.payload_nbytes(l3.parse_header(bytes(buf)))
    while len(buf) < need:
        if time.monotonic() > deadline:
            raise TimeoutError(
                f"dump payload timeout ({len(buf) - l3.HEADER.size}"
                f"/{need - l3.HEADER.size} B)")
        chunk = read_fn(need - len(buf))
        if chunk:
            buf += chunk
    return bytes(buf)


def process_dump(raw: bytes, *, noise_var: float = 1.0, lo_bin: int = 1,
                 hi_bin: int | None = None):
    """Dump bytes -> (meta, [ {frame, elev_deg, range_bin} per frame ]).

    Per-frame elevation at the strongest range bin. The launch-angle trajectory
    fit over these per-frame elevations is downstream (two_ray / fit_launch_angle).
    """
    meta, cube = l3.parse_dump(raw)
    rfft = l3.range_fft(cube)
    rows = []
    for f in range(meta["n_frames"]):
        deg, rb = l3.frame_elevation(rfft, f, n_tx=meta["n_tx"],
                                     n_rx=meta["n_rx"], noise_var=noise_var,
                                     lo_bin=lo_bin, hi_bin=hi_bin)
        rows.append(dict(frame=f, elev_deg=deg, range_bin=rb))
    return meta, rows


def capture_once(cli_serial, data_serial, *, noise_var: float = 1.0,
                 timeout_s: float = 8.0, raw_sink: str | None = None):
    """Send the dump command, read the burst, return (meta, rows[, raw])."""
    data_serial.reset_input_buffer()
    cli_serial.write(DUMP_CMD)
    raw = read_dump(data_serial.read, timeout_s=timeout_s)
    if raw_sink:
        with open(raw_sink, "wb") as fh:
            fh.write(raw)
    meta, rows = process_dump(raw, noise_var=noise_var)
    return meta, rows


def run_loop(cli_port: str, data_port: str, trigger_pin: int, *,
             session_dir: str = ".", noise_var: float = 1.0):
    """Pi-only glue: arm the GPIO trigger; on each impact, capture + log.

    Lazy-imports gpiozero + pyserial so importing this module (and its tests)
    needs neither. Wire SEN-14262 GATE -> BCM `trigger_pin`, shared GND.
    """
    import os

    import serial  # lazy: hardware only
    from gpiozero import Button  # lazy: Pi only

    cli = serial.Serial(cli_port, 115200, timeout=0.5)
    data = serial.Serial(data_port, 921600, timeout=0.2)
    shot = [0]

    def on_impact():
        shot[0] += 1
        n = shot[0]
        sink = os.path.join(session_dir, f"shot_{n:03d}.l3dump")
        try:
            meta, rows = capture_once(cli, data, noise_var=noise_var,
                                      raw_sink=sink)
        except TimeoutError as e:
            print(f"[shot {n}] dump timeout: {e}")
            return
        elevs = [r["elev_deg"] for r in rows]
        print(f"[shot {n}] {meta['n_frames']} frames  elev(deg): "
              f"{', '.join(f'{e:+.1f}' for e in elevs)}  -> {sink}")

    button = Button(trigger_pin, pull_up=False, bounce_time=0.05)
    button.when_pressed = on_impact
    print(f"armed on BCM{trigger_pin}; CLI={cli_port} DATA={data_port}. "
          "Ctrl-C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("stopped.")
    finally:
        cli.close()
        data.close()


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cli", required=True, help="TI CLI UART (115200)")
    ap.add_argument("--data", required=True, help="TI data UART (921600)")
    ap.add_argument("--trigger-pin", type=int, required=True,
                    help="BCM pin for SEN-14262 GATE")
    ap.add_argument("--session", default=".", help="dir for shot_*.l3dump")
    ap.add_argument("--noise-var", type=float, default=1.0)
    a = ap.parse_args()
    run_loop(a.cli, a.data, a.trigger_pin, session_dir=a.session,
             noise_var=a.noise_var)
