"""Runtime boundary joining TI capture to the frozen LCMF estimator."""

from __future__ import annotations

from dataclasses import dataclass

from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.lcmf import LCMFResult, estimate_lcmf_v1
from openflight.iwr6843.monitor import IWR6843Capture, IWR6843CaptureMonitor


@dataclass(frozen=True)
class IWR6843ShotResult:
    """Capture transport result and optional angle measurement."""

    capture: IWR6843Capture | None
    measurement: LCMFResult | None


@dataclass
class IWR6843Runtime:
    """Configured TI hardware and estimator state for the server."""

    capture_monitor: IWR6843CaptureMonitor
    calibration: Calibration
    net_range_m: float | None
    tx_order: str = "normal"
    capture_timeout_s: float = 12.0

    def process_shot(
        self,
        *,
        impact_timestamp: float | None,
        ball_speed_mph: float,
        club: str | None,
    ) -> IWR6843ShotResult:
        """Match one OPS shot to TI data and run LCMF-v1."""
        capture = self.capture_monitor.capture_for_shot(
            impact_timestamp,
            timeout_s=self.capture_timeout_s,
        )
        if capture is None or not capture.valid or capture.raw is None:
            return IWR6843ShotResult(capture=capture, measurement=None)
        measurement = estimate_lcmf_v1(
            capture.raw,
            self.calibration,
            ball_speed_mph=ball_speed_mph,
            club=club,
            net_range_m=self.net_range_m,
            tx_order=self.tx_order,
            tdm_sign_policy="positive",
        )
        return IWR6843ShotResult(capture=capture, measurement=measurement)

    def stop(self) -> None:
        """Release TI hardware."""
        self.capture_monitor.stop()


__all__ = ["IWR6843Runtime", "IWR6843ShotResult"]
