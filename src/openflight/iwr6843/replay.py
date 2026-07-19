"""Offline replay helpers for saved IWR6843 L3 captures."""

from __future__ import annotations

import csv
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.calibration_session import (
    clone_calibration,
    estimated_track_start_range_m,
)
from openflight.iwr6843.lcmf import LCMFResult, estimate_lcmf_v1
from openflight.iwr6843.shot import process_dump


@dataclass(frozen=True)
class ReplayInput:
    """One dump replay request."""

    source: str
    shot_number: int | None
    capture_path: Path
    ball_speed_mph: float
    club: str | None = None


@dataclass(frozen=True)
class ReplayRecord:
    """One offline replay result."""

    source: str
    shot_number: int | None
    capture_path: str
    ball_speed_mph: float | None
    club: str | None
    status: str
    launch_angle_deg: float | None = None
    raw_angle_deg: float | None = None
    component_std_deg: float | None = None
    n_snapshots: int | None = None
    n_frames: int | None = None
    track_speed_mph: float | None = None
    track_rms_bins: float | None = None
    track_inliers: int | None = None
    track_start_range_m: float | None = None
    error: str | None = None

    @property
    def accepted(self) -> bool:
        """Whether replay produced a launch angle."""
        return self.launch_angle_deg is not None

    def to_dict(self) -> dict:
        """Return JSON/CSV-safe output."""
        return {
            "source": self.source,
            "shot_number": self.shot_number,
            "capture_path": self.capture_path,
            "ball_speed_mph": self.ball_speed_mph,
            "club": self.club,
            "status": self.status,
            "launch_angle_deg": self.launch_angle_deg,
            "raw_angle_deg": self.raw_angle_deg,
            "component_std_deg": self.component_std_deg,
            "n_snapshots": self.n_snapshots,
            "n_frames": self.n_frames,
            "track_speed_mph": self.track_speed_mph,
            "track_rms_bins": self.track_rms_bins,
            "track_inliers": self.track_inliers,
            "track_start_range_m": self.track_start_range_m,
            "error": self.error,
        }


def replay_summary(records: Iterable[ReplayRecord]) -> dict:
    """Summarize coverage and quality for replay output."""
    records = list(records)
    accepted = [record for record in records if record.accepted]

    def median(values: Iterable[float | None]) -> float | None:
        clean = [float(value) for value in values if value is not None and math.isfinite(value)]
        return statistics.median(clean) if clean else None

    return {
        "shots": len(records),
        "accepted_shots": len(accepted),
        "coverage": len(accepted) / len(records) if records else 0.0,
        "median_launch_angle_deg": median(record.launch_angle_deg for record in accepted),
        "median_component_std_deg": median(record.component_std_deg for record in accepted),
        "median_track_rms_bins": median(record.track_rms_bins for record in accepted),
        "median_track_start_range_m": median(record.track_start_range_m for record in accepted),
    }


def _coerce_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _resolve_capture_path(path_value: str, *, session_path: Path | None = None) -> Path:
    path = Path(path_value).expanduser()
    if path.is_absolute() or session_path is None:
        return path
    return session_path.parent / path


def inputs_from_session(
    session_path: str | Path,
    *,
    club: str | None = None,
) -> list[ReplayInput]:
    """Extract replayable IWR6843 captures from a session JSONL file."""
    path = Path(session_path).expanduser()
    inputs: list[ReplayInput] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            entry = json.loads(line)
            if entry.get("type") != "iwr6843_capture":
                continue
            capture_path = entry.get("capture_path")
            ball_speed_mph = _coerce_float(entry.get("ball_speed_mph"))
            if not capture_path or ball_speed_mph is None:
                continue
            inputs.append(
                ReplayInput(
                    source=str(path),
                    shot_number=entry.get("shot_number"),
                    capture_path=_resolve_capture_path(capture_path, session_path=path),
                    ball_speed_mph=ball_speed_mph,
                    club=club,
                )
            )
    return inputs


def input_from_dump(
    dump_path: str | Path,
    *,
    ball_speed_mph: float,
    club: str | None = None,
) -> ReplayInput:
    """Create a replay input for one standalone dump."""
    path = Path(dump_path).expanduser()
    return ReplayInput(
        source=str(path),
        shot_number=None,
        capture_path=path,
        ball_speed_mph=float(ball_speed_mph),
        club=club,
    )


def replay_capture(
    replay_input: ReplayInput,
    calibration: Calibration,
    *,
    net_range_m: float | None,
    tx_order: str,
    tdm_sign_policy: str = "positive",
    grid_step_deg: float = 0.5,
    estimator=estimate_lcmf_v1,
) -> ReplayRecord:
    """Run LCMF-v1 on one saved dump."""
    if not replay_input.capture_path.is_file():
        return ReplayRecord(
            source=replay_input.source,
            shot_number=replay_input.shot_number,
            capture_path=str(replay_input.capture_path),
            ball_speed_mph=replay_input.ball_speed_mph,
            club=replay_input.club,
            status="missing_capture_file",
            error=f"capture not found: {replay_input.capture_path}",
        )
    try:
        raw = replay_input.capture_path.read_bytes()
        measurement: LCMFResult = estimator(
            raw,
            calibration,
            ball_speed_mph=replay_input.ball_speed_mph,
            club=replay_input.club,
            net_range_m=net_range_m,
            tx_order=tx_order,
            tdm_sign_policy=tdm_sign_policy,
            grid_step_deg=grid_step_deg,
        )
        track_measurement = process_dump(
            raw,
            calibration,
            club=replay_input.club,
            net_range_m=net_range_m,
            tx_order=tx_order,
            tdm_sign_policy=tdm_sign_policy,
        )
        return ReplayRecord(
            source=replay_input.source,
            shot_number=replay_input.shot_number,
            capture_path=str(replay_input.capture_path),
            ball_speed_mph=replay_input.ball_speed_mph,
            club=replay_input.club,
            status=measurement.status,
            launch_angle_deg=measurement.angle_deg if measurement.accepted else None,
            raw_angle_deg=measurement.raw_angle_deg,
            component_std_deg=measurement.component_std_deg,
            n_snapshots=measurement.n_snapshots,
            n_frames=measurement.n_frames,
            track_speed_mph=measurement.track_speed_mph,
            track_rms_bins=measurement.track_rms_bins,
            track_inliers=measurement.track_inliers,
            track_start_range_m=estimated_track_start_range_m(track_measurement, calibration),
        )
    except Exception as error:  # pylint: disable=broad-exception-caught
        return ReplayRecord(
            source=replay_input.source,
            shot_number=replay_input.shot_number,
            capture_path=str(replay_input.capture_path),
            ball_speed_mph=replay_input.ball_speed_mph,
            club=replay_input.club,
            status="replay_error",
            error=str(error),
        )


def build_replay_calibration(
    calibration_path: str | Path,
    *,
    tee_range_m: float,
    tilt_deg: float | None,
    radar_height_m: float | None,
    ball_height_m: float,
) -> Calibration:
    """Load array calibration and apply replay geometry overrides."""
    return clone_calibration(
        Calibration.load(str(calibration_path)),
        tee_range_m=tee_range_m,
        tilt_deg=tilt_deg,
        radar_height_m=radar_height_m,
        ball_height_m=ball_height_m,
    )


def write_records(records: Iterable[ReplayRecord], path: str | Path, output_format: str) -> None:
    """Write replay results as CSV or JSONL."""
    records = list(records)
    output_path = Path(path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_format == "jsonl":
        with output_path.open("w", encoding="utf-8") as handle:
            for record in records:
                handle.write(json.dumps(record.to_dict(), sort_keys=True) + "\n")
        return
    if output_format == "csv":
        fields = list(ReplayRecord("", None, "", None, None, "").to_dict().keys())
        with output_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for record in records:
                writer.writerow(record.to_dict())
        return
    raise ValueError(f"unsupported output format for file write: {output_format}")


__all__ = [
    "ReplayInput",
    "ReplayRecord",
    "build_replay_calibration",
    "input_from_dump",
    "inputs_from_session",
    "replay_capture",
    "replay_summary",
    "write_records",
]
