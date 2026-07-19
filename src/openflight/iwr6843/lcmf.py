"""Late-Flight Complex Multipath Fusion (LCMF-v1) launch estimator.

This module is the production form of the frozen 2026-07-16 TrackMan
candidate.  The model consumes an IWR6843 L3 dump plus an independently
measured OPS243 ball speed.  Its constants are intentionally fixed: changing
them requires a new estimator version and independent validation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from openflight.iwr6843 import doa, tracking
from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.dump import parse_dump, project_tx_pair
from openflight.iwr6843.multipath import (
    ballistic_trajectory_from_range,
    leave_one_channel_out_error,
)
from openflight.iwr6843.music import LAM
from openflight.iwr6843.shot import ShotMeasurement, geometry_from_header, process_dump

NAME = "lcmf_v1"
DISPLAY_NAME = "Late-Flight Complex Multipath Fusion v1"
# The July 14 development session used +2.8387689102 degrees. Independent
# validation showed that offset did not transfer across physical bay alignment,
# so production exposes the fused radar estimate without a truth-fitted shift.
ANGLE_CORRECTION_DEG = 0.0
CHANNEL_MODELS = ("two8", "four4_path_tdm")
FAST_MODELS = ("direct1", "two2", "four4")
COMPONENT_WEIGHT = 0.2
MIN_SNR = 8.0
MAX_RANGE_M = 4.7
MAX_PER_FRAME = 4
LATERAL_TEE_OFFSET_M = 0.064
MPH_PER_MS = 2.23694


@dataclass
class LCMFResult:
    """One LCMF-v1 estimate with enough evidence for session replay."""

    status: str
    angle_deg: float | None = None
    raw_angle_deg: float | None = None
    components_deg: dict[str, float] = field(default_factory=dict)
    n_snapshots: int = 0
    n_frames: int = 0
    component_std_deg: float | None = None
    tracker_quality: str | None = None
    track_speed_mph: float | None = None
    track_rms_bins: float | None = None
    track_inliers: int | None = None
    tdm_sign_used: int | None = None

    @property
    def accepted(self) -> bool:
        """Whether the estimator produced a launch angle."""
        return self.angle_deg is not None and self.status.startswith("accepted")

    def to_dict(self) -> dict:
        """Return JSON-safe estimator diagnostics."""
        return {
            "estimator": NAME,
            "status": self.status,
            "launch_angle_deg": self.angle_deg,
            "raw_angle_deg": self.raw_angle_deg,
            "angle_correction_deg": ANGLE_CORRECTION_DEG,
            "components_deg": dict(self.components_deg),
            "n_snapshots": self.n_snapshots,
            "n_frames": self.n_frames,
            "component_std_deg": self.component_std_deg,
            "tracker_quality": self.tracker_quality,
            "track_speed_mph": self.track_speed_mph,
            "track_rms_bins": self.track_rms_bins,
            "track_inliers": self.track_inliers,
            "tdm_sign_used": self.tdm_sign_used,
        }


def _result_from_track(status: str, shot: ShotMeasurement) -> LCMFResult:
    track = shot.track
    return LCMFResult(
        status=status,
        tracker_quality=shot.quality,
        track_speed_mph=track.speed_mph if track is not None else None,
        track_rms_bins=track.rms_bins if track is not None else None,
        track_inliers=track.n_inliers if track is not None else None,
        tdm_sign_used=shot.tdm_sign_used,
    )


def _snapshot_cache(
    raw: bytes,
    shot: ShotMeasurement,
    cal: Calibration,
    tx_order: str,
    tdm_sign: int,
) -> tuple[dict[str, np.ndarray], object, np.ndarray]:
    """Build calibrated per-loop snapshots along the TI range track."""
    meta, cube = parse_dump(raw)
    geometry = geometry_from_header(meta)
    frame_values = geometry.chirps_per_frame * geometry.n_rx * geometry.n_samples
    got_frames = cube.reshape(-1).size // frame_values
    if got_frames < geometry.n_frames:
        geometry.n_frames = got_frames
        cube = cube[:got_frames]
    if meta["n_tx"] != 2:
        raise ValueError(f"LCMF-v1 requires two TX channels, got {meta['n_tx']}")

    scope = "window" if shot.notch_recovered else "burst"
    mti = tracking.mti_filter(cube, scope=scope)
    noise = float(np.median(np.abs(mti) ** 2))
    track = shot.track
    if track is None:
        raise ValueError("ball track unavailable")

    values: dict[str, list] = {
        "t": [],
        "frame": [],
        "loop": [],
        "r": [],
        "snr": [],
        "vr": [],
        "vec": [],
    }
    for frame in range(geometry.n_frames):
        for loop in range(geometry.n_loops):
            time_s = geometry.loop_time(frame, loop)
            if not track.t_first - 2e-3 <= time_s <= track.t_last + 2e-3:
                continue
            range_bin = int(round(track.bin_at(time_s)))
            if not 2 <= range_bin < geometry.n_samples - 1:
                continue
            velocity = track.speed_ms_at(time_s, geometry.range_res_m)
            tdm_phase = tdm_sign * 4.0 * np.pi * velocity * doa.TDM_TAU_S / LAM
            uncalibrated = doa.canonicalize_tx_blocks(
                mti[frame, 0, loop, :, range_bin],
                mti[frame, 1, loop, :, range_bin],
                tdm_phase=tdm_phase,
                tx_order=tx_order,
            )
            values["t"].append(time_s)
            values["frame"].append(frame)
            values["loop"].append(loop)
            values["r"].append(cal.true_range(track.range_at(time_s, geometry.range_res_m)))
            values["snr"].append(float(np.mean(np.abs(uncalibrated) ** 2) / noise))
            values["vr"].append(velocity)
            values["vec"].append(cal.apply(uncalibrated))
    cache = {name: np.asarray(data) for name, data in values.items()}
    return cache, geometry, cube


def _balanced_indices(cache: dict[str, np.ndarray]) -> np.ndarray:
    """Select at most four strong snapshots per frame, as frozen for v1."""
    indices = np.nonzero((cache["snr"] >= MIN_SNR) & (cache["r"] <= MAX_RANGE_M))[0]
    selected: list[int] = []
    for frame in np.unique(cache["frame"][indices]):
        frame_indices = indices[cache["frame"][indices] == frame]
        order = np.argsort(cache["snr"][frame_indices])[::-1]
        selected.extend(frame_indices[order[:MAX_PER_FRAME]])
    return np.asarray(selected, dtype=int)


def _candidate_trajectory(
    launch_rad: float,
    range_m: np.ndarray,
    geometry: dict,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    return ballistic_trajectory_from_range(
        launch_rad,
        range_m,
        speed_ms=geometry["speed_ms"],
        tee_x_m=geometry["tee_x_m"],
        launch_height_m=geometry["ball_height_m"],
        radar_height_m=geometry["radar_height_m"],
        lateral_offset_m=LATERAL_TEE_OFFSET_M,
    )


def _spatial_dictionary(
    model: str,
    launch_rad: float,
    range_m: np.ndarray,
    geometry: dict,
) -> np.ndarray:
    """Build the exact frozen DD/DG/GD/GG moving-ball dictionary."""
    tx_order = doa.validate_tx_order(geometry["tx_order"])
    x_m, height_m, direct_vr, image_vr = _candidate_trajectory(launch_rad, range_m, geometry)
    tilt_rad = geometry["tilt_rad"]
    radar_height_m = geometry["radar_height_m"]
    direct = np.arctan2(height_m - radar_height_m, x_m) - tilt_rad
    image = np.arctan2(-(height_m + radar_height_m), x_m) - tilt_rad

    tx = np.repeat(np.array([0.0, 4.0]), 4)[None, :]
    rx = np.tile(np.arange(4, dtype=float), 2)[None, :]
    sin_direct = np.sin(direct)[:, None]
    sin_image = np.sin(image)[:, None]
    dd = np.exp(1j * np.pi * (tx * sin_direct + rx * sin_direct))
    dg = np.exp(1j * np.pi * (tx * sin_direct + rx * sin_image))
    gd = np.exp(1j * np.pi * (tx * sin_image + rx * sin_direct))
    gg = np.exp(1j * np.pi * (tx * sin_image + rx * sin_image))

    if model == "four4_path_tdm":
        cross_phase = 2.0 * np.pi * (image_vr - direct_vr) * doa.TDM_TAU_S / LAM
        later = doa.later_physical_tx_index(tx_order)
        block = slice(4 * later, 4 * (later + 1))
        dg[:, block] *= np.exp(1j * cross_phase)[:, None]
        gd[:, block] *= np.exp(1j * cross_phase)[:, None]
        gg[:, block] *= np.exp(2j * cross_phase)[:, None]

    if model == "two8":
        return np.stack([dd, gg], axis=-1)
    if model in ("four4", "four4_path_tdm"):
        return np.stack([dd, dg, gd, gg], axis=-1)
    raise ValueError(f"unknown spatial model: {model}")


def _frame_objective(errors: np.ndarray, frames: np.ndarray, ceiling: float) -> float:
    frame_errors = [
        np.median(np.clip(errors[frames == frame], 1e-8, ceiling)) for frame in np.unique(frames)
    ]
    return float(np.mean(np.log(frame_errors)))


def _refine_grid(grid_deg: np.ndarray, objective: np.ndarray) -> float:
    index = int(np.argmin(objective))
    if not 0 < index < len(grid_deg) - 1:
        return float(grid_deg[index])
    y0, y1, y2 = objective[index - 1 : index + 2]
    denominator = y0 - 2.0 * y1 + y2
    offset = 0.5 * (y0 - y2) / denominator if denominator > 0 else 0.0
    return float(grid_deg[index] + np.clip(offset, -1.0, 1.0) * np.diff(grid_deg)[0])


def _channel_estimates(
    cache: dict[str, np.ndarray],
    indices: np.ndarray,
    geometry: dict,
    grid_deg: np.ndarray,
) -> dict[str, float]:
    frames = cache["frame"][indices]
    if len(indices) < 12 or len(np.unique(frames)) < 3:
        raise ValueError("insufficient channel snapshots")
    vectors = cache["vec"][indices]
    range_m = cache["r"][indices]
    estimates: dict[str, float] = {}
    for model in CHANNEL_MODELS:
        objective = []
        for angle_deg in grid_deg:
            dictionary = _spatial_dictionary(model, np.radians(angle_deg), range_m, geometry)
            errors = leave_one_channel_out_error(vectors, dictionary)
            objective.append(_frame_objective(errors, frames, 1e3))
        estimates[f"channel_{model}_deg"] = _refine_grid(grid_deg, np.asarray(objective))
    return estimates


def _prepared_fft(
    cube: np.ndarray,
    radar_geometry,
    indices: np.ndarray,
    cache: dict[str, np.ndarray],
    element_correction: np.ndarray,
    tx_order: str,
    tdm_sign: int,
    *,
    n_fft: int = 512,
) -> tuple[np.ndarray, np.ndarray]:
    tdm = cube.reshape(
        radar_geometry.n_frames,
        radar_geometry.n_loops,
        2,
        radar_geometry.n_rx,
        radar_geometry.n_samples,
    ).transpose(0, 2, 1, 3, 4)
    tdm = tdm - tdm.mean(axis=2, keepdims=True)
    window = np.hanning(radar_geometry.n_samples)
    snapshots = []
    phase_per_ms = 4.0 * np.pi * doa.TDM_TAU_S / LAM
    for index in indices:
        frame = int(cache["frame"][index])
        loop = int(cache["loop"][index])
        snapshot = doa.canonicalize_tx_blocks(
            tdm[frame, 0, loop],
            tdm[frame, 1, loop],
            tdm_phase=tdm_sign * phase_per_ms * cache["vr"][index],
            tx_order=tx_order,
        )
        snapshot *= element_correction[:, None]
        snapshots.append(np.fft.fft(snapshot * window[None, :], n=n_fft, axis=-1))
    return np.asarray(snapshots), window


def _quadratic_peak(profile: np.ndarray, center: float, radius: int) -> float:
    center_bin = int(round(center))
    lo = max(center_bin - radius, 1)
    hi = min(center_bin + radius + 1, len(profile) - 1)
    peak = lo + int(np.argmax(profile[lo:hi]))
    y0, y1, y2 = profile[peak - 1 : peak + 2]
    denominator = y0 - 2.0 * y1 + y2
    offset = 0.5 * (y0 - y2) / denominator if denominator < 0 else 0.0
    return float(peak + np.clip(offset, -1.0, 1.0))


def _fast_design(
    model: str,
    launch_rad: float,
    range_m: np.ndarray,
    center_native_bins: np.ndarray,
    selected_bins: np.ndarray,
    geometry: dict,
    *,
    n_samples: int,
    n_fft: int,
    range_res_m: float,
    window: np.ndarray,
) -> np.ndarray:
    spatial = _spatial_dictionary("four4", launch_rad, range_m, geometry)
    x_m, height_m, _direct_vr, _image_vr = _candidate_trajectory(launch_rad, range_m, geometry)
    radar_height_m = geometry["radar_height_m"]
    direct_distance = np.hypot(x_m, height_m - radar_height_m)
    image_distance = np.hypot(x_m, height_m + radar_height_m)
    delta_bins = (image_distance - direct_distance) / range_res_m
    path_bins = center_native_bins[:, None] + np.stack(
        [
            np.zeros_like(delta_bins),
            0.5 * delta_bins,
            0.5 * delta_bins,
            delta_bins,
        ],
        axis=1,
    )
    sample = np.arange(n_samples)
    tones = np.exp(2j * np.pi * path_bins[:, :, None] * sample / n_samples)
    response = np.fft.fft(tones * window[None, None, :], n=n_fft, axis=-1)
    response = np.take_along_axis(response, selected_bins[:, None, :], axis=2)
    design = spatial[:, :, None, :] * response.transpose(0, 2, 1)[:, None, :, :]
    if model == "direct1":
        return design[..., [0]]
    if model == "two2":
        return design[..., [0, 3]]
    if model == "four4":
        return design
    raise ValueError(f"unknown fast-time model: {model}")


def _fit_error(data: np.ndarray, design: np.ndarray) -> np.ndarray:
    flattened_data = data.reshape(len(data), -1)
    flattened_design = design.reshape(len(data), -1, design.shape[-1])
    coefficients = np.einsum("...kn,...n->...k", np.linalg.pinv(flattened_design), flattened_data)
    prediction = np.einsum("...nk,...k->...n", flattened_design, coefficients)
    residual = np.sum(np.abs(flattened_data - prediction) ** 2, axis=1)
    power = np.sum(np.abs(flattened_data) ** 2, axis=1) + 1e-12
    return residual / power


def _fast_estimates(
    cube: np.ndarray,
    radar_geometry,
    cache: dict[str, np.ndarray],
    indices: np.ndarray,
    geometry: dict,
    cal: Calibration,
    grid_deg: np.ndarray,
    tx_order: str,
    tdm_sign: int,
) -> dict[str, float]:
    ordered = indices[np.argsort(cache["t"][indices])]
    indices = ordered[len(ordered) // 2 :]
    frames = cache["frame"][indices]
    if len(indices) < 6 or len(np.unique(frames)) < 2:
        raise ValueError("insufficient late-flight snapshots")

    n_fft = 512
    data_fft, window = _prepared_fft(
        cube,
        radar_geometry,
        indices,
        cache,
        cal.elem_correction,
        tx_order,
        tdm_sign,
        n_fft=n_fft,
    )
    oversample = n_fft / radar_geometry.n_samples
    range_m = cache["r"][indices]
    track_bins = (range_m + cal.range_bias_m) / radar_geometry.range_res_m * oversample
    power = np.sum(np.abs(data_fft) ** 2, axis=1)
    centers = np.asarray(
        [
            _quadratic_peak(profile, center, int(np.ceil(oversample)))
            for profile, center in zip(power, track_bins, strict=True)
        ]
    )
    offsets = np.arange(-int(np.ceil(2.0 * oversample)), int(np.ceil(6.0 * oversample)) + 1)
    bins = np.rint(centers).astype(int)[:, None] + offsets[None, :]
    if np.any(bins < 0) or np.any(bins >= data_fft.shape[-1]):
        raise ValueError("local range window exceeds FFT bounds")
    local_data = np.take_along_axis(data_fft, bins[:, None, :], axis=2)
    center_native = centers / oversample

    estimates: dict[str, float] = {}
    for model in FAST_MODELS:
        objective = []
        for angle_deg in grid_deg:
            design = _fast_design(
                model,
                np.radians(angle_deg),
                range_m,
                center_native,
                bins,
                geometry,
                n_samples=radar_geometry.n_samples,
                n_fft=n_fft,
                range_res_m=radar_geometry.range_res_m,
                window=window,
            )
            objective.append(_frame_objective(_fit_error(local_data, design), frames, 1.0))
        estimates[f"fast_{model}_deg"] = _refine_grid(grid_deg, np.asarray(objective))
    return estimates


def estimate_lcmf_v1(
    raw: bytes,
    cal: Calibration,
    *,
    ball_speed_mph: float,
    club: str | None = None,
    net_range_m: float | None = None,
    tx_order: str = "normal",
    tdm_sign_policy: str = "positive",
    grid_step_deg: float = 0.5,
) -> LCMFResult:
    """Estimate vertical launch from one TI dump and OPS ball speed."""
    if ball_speed_mph <= 0:
        raise ValueError("ball_speed_mph must be positive")
    if cal.tee_range_m is None:
        raise ValueError("LCMF-v1 requires the measured tee slant range")
    meta, _cube = parse_dump(raw)
    if meta["n_tx"] == 3:
        raw = project_tx_pair(raw, (0, 2))

    shot = process_dump(
        raw,
        cal,
        club=club,
        net_range_m=net_range_m,
        tx_order=tx_order,
        tdm_sign_policy=tdm_sign_policy,
    )
    if shot.track is None:
        return _result_from_track("rejected_by_ball_tracker", shot)
    if shot.tdm_sign_used not in (-1, 1):
        return _result_from_track("rejected_missing_tdm_sign", shot)

    try:
        cache, radar_geometry, cube = _snapshot_cache(raw, shot, cal, tx_order, shot.tdm_sign_used)
        indices = _balanced_indices(cache)
        vertical_delta_m = cal.tee_ball_height_m - cal.radar_height_m
        tee_x_m = math.sqrt(max(cal.tee_range_m**2 - vertical_delta_m**2, 0.25))
        model_geometry = {
            "speed_ms": ball_speed_mph / MPH_PER_MS,
            "tee_x_m": tee_x_m,
            "ball_height_m": cal.tee_ball_height_m,
            "radar_height_m": cal.radar_height_m,
            "tilt_rad": cal.tilt_rad,
            "tx_order": tx_order,
        }
        grid_deg = np.arange(-5.0, 45.0 + grid_step_deg / 2.0, grid_step_deg)
        components = _channel_estimates(cache, indices, model_geometry, grid_deg)
        components.update(
            _fast_estimates(
                cube,
                radar_geometry,
                cache,
                indices,
                model_geometry,
                cal,
                grid_deg,
                tx_order,
                shot.tdm_sign_used,
            )
        )
    except (ValueError, IndexError, np.linalg.LinAlgError) as error:
        return _result_from_track(str(error).replace(" ", "_"), shot)

    component_values = np.asarray(list(components.values()), dtype=float)
    raw_angle_deg = float(np.sum(component_values) * COMPONENT_WEIGHT)
    result = _result_from_track(
        "accepted_track_quality_warning" if shot.quality == "reject" else "accepted",
        shot,
    )
    result.angle_deg = raw_angle_deg + ANGLE_CORRECTION_DEG
    result.raw_angle_deg = raw_angle_deg
    result.components_deg = components
    result.n_snapshots = len(indices)
    result.n_frames = len(np.unique(cache["frame"][indices]))
    result.component_std_deg = float(np.std(component_values))
    return result


__all__ = [
    "ANGLE_CORRECTION_DEG",
    "DISPLAY_NAME",
    "LCMFResult",
    "NAME",
    "estimate_lcmf_v1",
]
