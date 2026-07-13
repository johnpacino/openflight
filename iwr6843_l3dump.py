"""Compatibility shim — the dump format moved into the package.

Use ``openflight.iwr6843.dump``; this module re-exports it for older
scripts/tests and will be removed once nothing imports it.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src"))

from openflight.iwr6843.dump import *          # noqa: F401,F403,E402
from openflight.iwr6843.dump import (          # noqa: F401,E402
    HEADER, MAGIC, SAMPLE_INT16_IQ, parse_dump, parse_header,
    payload_nbytes, pack_dump, range_fft, ball_range_bin,
    virtual_snapshot, frame_elevation, synth_target)
