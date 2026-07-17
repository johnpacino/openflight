"""IWR6843 60 GHz radar integration — driver, calibration, shot pipeline.

The single-chip successor to the OPS243 + K-LD7 stack: raw-ADC L3 dumps from
custom firmware, all processing host-side. See docs/plans/stage1c_l3_burst_dump.md.
"""

from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.lcmf import LCMFResult, estimate_lcmf_v1
from openflight.iwr6843.shot import ShotMeasurement, process_dump
from openflight.iwr6843.tracking import BallTrack, Geometry
from openflight.iwr6843.trajectory import TrajectoryFit

__all__ = [
    "BallTrack",
    "Calibration",
    "Geometry",
    "IWR6843Radar",
    "LCMFResult",
    "ShotMeasurement",
    "TrajectoryFit",
    "estimate_lcmf_v1",
    "process_dump",
]
