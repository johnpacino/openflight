#!/usr/bin/env python3
"""Milestone-2a hardware round-trip check for the L3-dump firmware.

Validates ONLY the firmware<->host UART/CLI contract -- no sensor, no GPIO:
send ``l3dump`` on the CLI UART, then confirm the data UART streams a burst that
iwr6843_runtime.read_dump() frames and iwr6843_l3dump.parse_header() accepts.
The 2a payload is a zero stub, so this checks framing + geometry, not real data.

Ports are explicit on purpose: the 2a firmware does not yet answer the stock-demo
CLI probe (``sensorStop``), so iwr6843_uart.find_levm_ports() cannot auto-detect
it -- that arrives with the sensorStop/Start handlers in step 3. Opening reuses
SerialReader's DTR/RTS-safe open so the probe can't reset or wedge the board.

    uv run python firmware/l3_dump/roundtrip_2a.py \
        --cli /dev/tty.SLAB_USBtoUART --data /dev/tty.SLAB_USBtoUART3
"""
from __future__ import annotations

import argparse
import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import iwr6843_l3dump as l3          # noqa: E402
from iwr6843_runtime import DUMP_CMD, read_dump  # noqa: E402
from iwr6843_uart import SerialReader            # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cli", required=True, help="TI CLI UART (115200)")
    ap.add_argument("--data", required=True, help="TI data UART (921600)")
    ap.add_argument("--timeout", type=float, default=8.0,
                    help="seconds to wait for the full burst")
    a = ap.parse_args()

    # SerialReader opens both ports without pulsing DTR/RTS (TI EVMs tie those to
    # NRST/boot-mode -- a plain open can reset the board).
    rdr = SerialReader(cli_port=a.cli, data_port=a.data)
    try:
        rdr.data.reset_input_buffer()
        rdr.cli.write(DUMP_CMD)
        raw = read_dump(rdr.data.read, timeout_s=a.timeout)
    finally:
        rdr.cli.close()
        rdr.data.close()

    meta = l3.parse_header(raw)
    expected = l3.HEADER.size + l3.payload_nbytes(meta)
    ok = (len(raw) == expected)

    print(f"received : {len(raw)} bytes")
    print(f"header   : {meta}")
    print(f"expected : {expected} bytes (header {l3.HEADER.size} + payload "
          f"{l3.payload_nbytes(meta)})")
    print("PASS: firmware<->host contract validated" if ok else
          f"FAIL: byte count {len(raw)} != expected {expected}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
