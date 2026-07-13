"""Array + geometry calibration for the IWR6843 elevation array.

Produced by the corner-reflector procedure (3 tape-measured positions,
2026-07-12: range bias +6.6 cm constant, per-element phase/gain including the
~40 deg static TX0<->TX2 block offset, mount tilt 10.40 deg — independently
confirmed by the user's inclinometer). Conventions:

- Element corrections apply AFTER the physical-orientation flip (x8[::-1]).
- ``tilt_rad`` rotates measured angles into ground coordinates.
- ``range_bias_m`` subtracts from measured range (radar-face referenced).

Re-run the calibration whenever the mount, enclosure, or antenna changes.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np

DEFAULT_CAL_PATH = "config/iwr6843_cal_20260712.json"


@dataclass
class Calibration:
    """Loaded calibration constants, ready to apply to snapshots."""

    elem_correction: np.ndarray          # complex, len n_virtual elements
    tilt_rad: float
    range_bias_m: float
    source: str = "unset"
    tee_range_m: float | None = None     # tape-measured launch point (slant)
    tee_height_m: float = 0.04           # ball-on-mat height above radar plane
    meta: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path: str = DEFAULT_CAL_PATH) -> "Calibration":
        """Load a cal JSON written by the corner-reflector solve."""
        with open(path, encoding="utf-8") as fh:
            raw = json.load(fh)
        corr = (np.exp(-1j * np.asarray(raw["elem_phase_rad"]))
                / np.asarray(raw["elem_gain"]))
        return cls(elem_correction=corr,
                   tilt_rad=float(np.radians(raw["tilt_deg"])),
                   range_bias_m=float(raw["range_bias_const_m"]),
                   source=path, meta=raw)

    @classmethod
    def identity(cls, n_elements: int = 8) -> "Calibration":
        """No-op calibration (uncalibrated array, zero tilt/bias)."""
        return cls(elem_correction=np.ones(n_elements, dtype=complex),
                   tilt_rad=0.0, range_bias_m=0.0, source="identity")

    def apply(self, snapshot: np.ndarray) -> np.ndarray:
        """Correct a physical-order snapshot (post-flip) element-wise."""
        return snapshot * self.elem_correction

    def true_range(self, measured_m: float) -> float:
        """Bias-corrected range from a measured (apparent) range."""
        return measured_m - self.range_bias_m
