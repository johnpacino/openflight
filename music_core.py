"""Compatibility shim — the DSP core moved into the package.

Use ``openflight.iwr6843.music``; this re-export keeps the stage-0 sim and
older analysis scripts working.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src"))

from openflight.iwr6843.music import *         # noqa: F401,F403,E402
from openflight.iwr6843.music import (         # noqa: F401,E402
    C, F_C, LAM, D_EL, RANGE_RES, G, GRID, steer, est_interferometry,
    est_bartlett, est_music_fbss, est_music_fbss_high, _grid_steer)
