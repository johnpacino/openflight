"""Ground-multipath geometry and MIMO array signatures.

The production elevation estimator treats the 2-TX x 4-RX virtual array as
an eight-element ULA.  That representation is exact for a single monostatic
path, but ground reflection can occur independently on the transmit and
receive legs.  The resulting DD, DG, GD, and GG signatures are most naturally
represented with the physical TX/RX coordinates kept separate.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GroundPathGeometry:
    """Direct and image-path geometry for one target position."""

    direct_distance_m: float
    image_distance_m: float
    direct_angle_rad: float
    image_angle_rad: float
    reflection_point_m: float

    @property
    def cross_range_excess_m(self) -> float:
        """Apparent radar-range excess for one reflected propagation leg."""
        return 0.5 * (self.image_distance_m - self.direct_distance_m)

    @property
    def double_range_excess_m(self) -> float:
        """Apparent radar-range excess when both propagation legs reflect."""
        return self.image_distance_m - self.direct_distance_m


def ballistic_trajectory_from_range(
    launch_rad: float,
    range_m: np.ndarray,
    *,
    speed_ms: float,
    tee_x_m: float,
    launch_height_m: float,
    radar_height_m: float,
    lateral_offset_m: float = 0.0,
    gravity_ms2: float = 9.81,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Invert direct slant range under a ballistic launch candidate.

    Returns downrange position, floor-referenced target height, direct range
    rate, and floor-image range rate. The inversion is monotonic over the
    short launch-monitor observation window and uses Newton iterations.
    """
    ranges = np.asarray(range_m, dtype=float)
    x_m = np.clip(ranges, 0.5, 6.0)
    cos_launch = np.cos(launch_rad)
    for _ in range(8):
        dx = x_m - tee_x_m
        time_s = dx / (speed_ms * cos_launch + 1e-9)
        height_m = launch_height_m + np.tan(launch_rad) * dx - 0.5 * gravity_ms2 * time_s**2
        vertical = height_m - radar_height_m
        modeled_range = np.sqrt(x_m**2 + lateral_offset_m**2 + vertical**2)
        dz_dx = np.tan(launch_rad) - gravity_ms2 * dx / (speed_ms * cos_launch) ** 2
        dr_dx = (x_m + vertical * dz_dx) / np.maximum(modeled_range, 1e-9)
        x_m = np.clip(x_m - (modeled_range - ranges) / dr_dx, 0.5, 6.0)

    dx = x_m - tee_x_m
    time_s = dx / (speed_ms * cos_launch + 1e-9)
    height_m = launch_height_m + np.tan(launch_rad) * dx - 0.5 * gravity_ms2 * time_s**2
    vx = speed_ms * cos_launch
    vz = speed_ms * np.sin(launch_rad) - gravity_ms2 * time_s
    horizontal_sq = x_m**2 + lateral_offset_m**2
    direct_vertical = height_m - radar_height_m
    image_vertical = height_m + radar_height_m
    direct_range = np.sqrt(horizontal_sq + direct_vertical**2)
    image_range = np.sqrt(horizontal_sq + image_vertical**2)
    direct_vr = (x_m * vx + direct_vertical * vz) / np.maximum(direct_range, 1e-9)
    image_vr = (x_m * vx + image_vertical * vz) / np.maximum(image_range, 1e-9)
    return x_m, height_m, direct_vr, image_vr


def ground_path_geometry(
    x_m: float,
    y_m: float,
    z_m: float,
    radar_height_m: float,
    tilt_rad: float = 0.0,
) -> GroundPathGeometry:
    """Return direct/image geometry using the flat-floor image construction."""
    horizontal_m = float(np.hypot(x_m, y_m))
    direct_vertical_m = z_m - radar_height_m
    image_vertical_m = -(z_m + radar_height_m)
    direct_m = float(np.hypot(horizontal_m, direct_vertical_m))
    image_m = float(np.hypot(horizontal_m, image_vertical_m))
    denominator = radar_height_m + z_m
    reflection_m = horizontal_m * radar_height_m / denominator if denominator > 0 else 0.0
    return GroundPathGeometry(
        direct_distance_m=direct_m,
        image_distance_m=image_m,
        direct_angle_rad=float(np.arctan2(direct_vertical_m, horizontal_m) - tilt_rad),
        image_angle_rad=float(np.arctan2(image_vertical_m, horizontal_m) - tilt_rad),
        reflection_point_m=reflection_m,
    )


def mimo_four_path_dictionary(
    direct_angle_rad: float,
    image_angle_rad: float,
    *,
    n_rx: int = 4,
    tx_offsets_half_lambda: tuple[float, ...] = (0.0, 4.0),
    later_tx_index: int | None = None,
    cross_tdm_phase_rad: float = 0.0,
    double_tdm_phase_rad: float = 0.0,
) -> np.ndarray:
    """Return columns ``[DD, DG, GD, GG]`` for a TDM-MIMO snapshot.

    Coordinates are measured in half wavelengths. ``DG`` uses direct TX and
    ground-image RX angles; ``GD`` uses image TX and direct RX angles.  When a
    snapshot has already been Doppler-corrected for DD, the optional residual
    phases model the different path-rate of reflected components on the later
    TX block.
    """
    tx_offsets = np.asarray(tx_offsets_half_lambda, dtype=float)
    rx_offsets = np.arange(n_rx, dtype=float)
    tx = np.repeat(tx_offsets, n_rx)
    rx = np.tile(rx_offsets, len(tx_offsets))
    sin_direct = np.sin(direct_angle_rad)
    sin_image = np.sin(image_angle_rad)
    dictionary = np.column_stack(
        [
            np.exp(1j * np.pi * (tx * sin_direct + rx * sin_direct)),
            np.exp(1j * np.pi * (tx * sin_direct + rx * sin_image)),
            np.exp(1j * np.pi * (tx * sin_image + rx * sin_direct)),
            np.exp(1j * np.pi * (tx * sin_image + rx * sin_image)),
        ]
    )
    if later_tx_index is not None:
        start = later_tx_index * n_rx
        stop = start + n_rx
        dictionary[start:stop, 1:3] *= np.exp(1j * cross_tdm_phase_rad)
        dictionary[start:stop, 3] *= np.exp(1j * double_tdm_phase_rad)
    return dictionary


def receive_two_path_dictionary(
    direct_angle_rad: float,
    image_angle_rad: float,
    *,
    n_rx: int = 4,
) -> np.ndarray:
    """Direct and image steering columns for one physical TX and its RX row."""
    rx = np.arange(n_rx, dtype=float)
    return np.column_stack(
        [
            np.exp(1j * np.pi * rx * np.sin(direct_angle_rad)),
            np.exp(1j * np.pi * rx * np.sin(image_angle_rad)),
        ]
    )


def collapse_reciprocal_cross_paths(dictionary: np.ndarray) -> np.ndarray:
    """Constrain DG and GD to one shared complex coefficient."""
    if dictionary.ndim < 2 or dictionary.shape[-1] != 4:
        raise ValueError("four-path dictionary must end in four columns")
    return np.stack(
        [dictionary[..., 0], dictionary[..., 1] + dictionary[..., 2], dictionary[..., 3]],
        axis=-1,
    )


def leave_one_channel_out_error(snapshot: np.ndarray, dictionary: np.ndarray) -> np.ndarray:
    """Normalized PRESS error for a complex linear array model.

    The final two dictionary dimensions are ``(channel, coefficient)`` and
    the snapshot's final dimension is ``channel``. Leading dimensions are
    broadcast as independent observations. The PRESS residual predicts each
    channel from coefficients fitted without that channel, which fairly
    compares dictionaries with different numbers of nuisance coefficients.
    """
    dictionary = np.asarray(dictionary, dtype=complex)
    snapshot = np.asarray(snapshot, dtype=complex)
    if dictionary.shape[-2] != snapshot.shape[-1]:
        raise ValueError("dictionary channel dimension must match snapshot")
    pseudo = np.linalg.pinv(dictionary)
    coefficients = np.einsum("...kn,...n->...k", pseudo, snapshot)
    prediction = np.einsum("...nk,...k->...n", dictionary, coefficients)
    residual = snapshot - prediction
    leverage = np.einsum("...nk,...kn->...n", dictionary, pseudo).real
    denominator = np.maximum(1.0 - leverage, 1e-6)
    press = np.sum(np.abs(residual / denominator) ** 2, axis=-1)
    power = np.sum(np.abs(snapshot) ** 2, axis=-1) + 1e-12
    return press / power
