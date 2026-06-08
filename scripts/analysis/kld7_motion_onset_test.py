#!/usr/bin/env python3
"""Correlate OPS sound triggers with K-LD7 motion onset.

This diagnostic is intended for non-golf timing tests such as a corner
reflector on a release sled. It records every OPS hardware sound trigger,
then snapshots the K-LD7 stream around that trigger and reports when target
frames and raw RADC motion energy first appear.

Run on the Pi, for example:

    uv run --extra kld7 python scripts/analysis/kld7_motion_onset_test.py \
      --kld7-port /dev/kld7_vertical --ops243-port /dev/ttyACM0 --trials 10

The output JSONL is written to ~/openflight_sessions by default.
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import math
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from openflight.kld7.radc import (  # noqa: E402
    RADC_PAYLOAD_BYTES,
    bin_to_velocity_kmh,
    compute_spectrum,
    parse_radc_payload,
    to_complex_iq,
)
from openflight.ops243 import OPS243Radar  # noqa: E402
from openflight.rolling_buffer.processor import RollingBufferProcessor  # noqa: E402

RANGE_SETTINGS = {5: 0, 10: 1, 30: 2, 100: 3}
SPEED_SETTINGS = {12: 0, 25: 1, 50: 2, 100: 3}
OPS_SAMPLE_RATE_HZ = 30000.0
OPS_CAPTURE_SAMPLES = 4096
OPS_SEGMENT_SAMPLES = 128


@dataclass(frozen=True)
class RADCMetric:
    """Compact motion metric for one RADC frame."""

    snr: float
    peak_bin: int
    peak_velocity_kmh: float
    peak_magnitude: float
    median_magnitude: float


def target_to_dict(target: Any) -> dict[str, float] | None:
    """Convert a kld7 Target object to JSON-safe values."""
    if target is None:
        return None
    return {
        "distance": float(target.distance),
        "speed": float(target.speed),
        "angle": float(target.angle),
        "magnitude": float(target.magnitude),
    }


def post_trigger_duration_ms(pre_trigger_segments: int) -> float:
    """Return rolling-buffer post-trigger span for the OPS S# split."""
    capture_ms = OPS_CAPTURE_SAMPLES / OPS_SAMPLE_RATE_HZ * 1000.0
    pre_ms = max(0, min(32, pre_trigger_segments)) * OPS_SEGMENT_SAMPLES
    pre_ms = pre_ms / OPS_SAMPLE_RATE_HZ * 1000.0
    return max(capture_ms - pre_ms, 0.0)


def frame_timestamp(frame: dict[str, Any]) -> float:
    """Prefer RADC arrival timestamp when present; otherwise frame timestamp."""
    return float(frame.get("radc_timestamp") or frame["timestamp"])


def relative_ms(timestamp: float | None, trigger_timestamp: float) -> float | None:
    """Convert a host timestamp to milliseconds relative to trigger."""
    if timestamp is None:
        return None
    return (float(timestamp) - float(trigger_timestamp)) * 1000.0


def radc_motion_metric(
    radc_payload: bytes,
    *,
    fft_size: int = 2048,
    dc_mask_bins: int = 5,
    max_speed_kmh: float = 100.0,
) -> RADCMetric | None:
    """Return a broad Doppler motion metric from raw RADC bytes.

    This intentionally does not use the golf OPS speed bin. For the sled test,
    we only need "did motion energy appear?" so a broad peak/median ratio is
    easier to interpret.
    """
    if not isinstance(radc_payload, bytes) or len(radc_payload) != RADC_PAYLOAD_BYTES:
        return None

    channels = parse_radc_payload(radc_payload)
    iq = to_complex_iq(channels["f1a_i"], channels["f1a_q"])
    spectrum = compute_spectrum(iq, fft_size=fft_size, dc_mask_bins=dc_mask_bins)
    positive = spectrum[spectrum > 0]
    if positive.size == 0:
        return None

    peak_bin = int(np.argmax(spectrum))
    peak_magnitude = float(spectrum[peak_bin])
    median_magnitude = float(np.median(positive))
    if median_magnitude <= 0:
        return None

    return RADCMetric(
        snr=peak_magnitude / median_magnitude,
        peak_bin=peak_bin,
        peak_velocity_kmh=float(bin_to_velocity_kmh(peak_bin, fft_size, max_speed_kmh)),
        peak_magnitude=peak_magnitude,
        median_magnitude=median_magnitude,
    )


def summarize_target_onset(
    frames: list[dict[str, Any]],
    *,
    trigger_timestamp: float,
) -> dict[str, Any]:
    """Summarize first PDAT/TDAT target timing around a trigger."""
    first_target_ms = None
    first_tdat_ms = None
    first_pdat_ms = None
    frames_with_targets = 0
    max_pdat_targets = 0

    for frame in sorted(frames, key=frame_timestamp):
        t_ms = relative_ms(frame_timestamp(frame), trigger_timestamp)
        if t_ms is None:
            continue

        has_tdat = frame.get("tdat") is not None
        pdat_count = len(frame.get("pdat") or [])
        has_pdat = pdat_count > 0
        max_pdat_targets = max(max_pdat_targets, pdat_count)

        if has_tdat or has_pdat:
            frames_with_targets += 1
            if first_target_ms is None:
                first_target_ms = t_ms
        if has_tdat and first_tdat_ms is None:
            first_tdat_ms = t_ms
        if has_pdat and first_pdat_ms is None:
            first_pdat_ms = t_ms

    return {
        "first_target_ms": first_target_ms,
        "first_tdat_ms": first_tdat_ms,
        "first_pdat_ms": first_pdat_ms,
        "frames_with_targets": frames_with_targets,
        "max_pdat_targets": max_pdat_targets,
    }


def summarize_radc_onset(
    frames: list[dict[str, Any]],
    *,
    trigger_timestamp: float,
    min_snr: float = 8.0,
    baseline_factor: float = 1.5,
    post_min_ms: float = -25.0,
) -> dict[str, Any]:
    """Summarize first raw-RADC motion-energy timing around a trigger."""
    rows = []
    pre_snrs = []

    for frame in sorted(frames, key=frame_timestamp):
        metric = frame.get("radc_metric")
        if metric is None:
            continue
        t_ms = relative_ms(frame_timestamp(frame), trigger_timestamp)
        if t_ms is None:
            continue
        snr = float(metric["snr"])
        rows.append((t_ms, frame, metric))
        if t_ms < 0:
            pre_snrs.append(snr)

    baseline_median = float(np.median(pre_snrs)) if pre_snrs else None
    threshold = float(min_snr)
    if baseline_median is not None and math.isfinite(baseline_median):
        threshold = max(threshold, baseline_median * baseline_factor)

    first = None
    strongest = None
    for t_ms, frame, metric in rows:
        if strongest is None or float(metric["snr"]) > float(strongest[2]["snr"]):
            strongest = (t_ms, frame, metric)
        if first is None and t_ms >= post_min_ms and float(metric["snr"]) >= threshold:
            first = (t_ms, frame, metric)

    def payload(row: tuple[float, dict[str, Any], dict[str, Any]] | None) -> dict[str, Any] | None:
        if row is None:
            return None
        t_ms, frame, metric = row
        return {
            "t_ms": t_ms,
            "frame_timestamp": frame_timestamp(frame),
            "snr": metric["snr"],
            "peak_bin": metric["peak_bin"],
            "peak_velocity_kmh": metric["peak_velocity_kmh"],
        }

    return {
        "threshold_snr": threshold,
        "baseline_median_snr": baseline_median,
        "frames_with_radc": len(rows),
        "first_motion": payload(first),
        "strongest_motion": payload(strongest),
    }


class KLD7StreamBuffer:
    """Background K-LD7 stream with a small host-side ring buffer."""

    def __init__(
        self,
        *,
        port: str,
        baud: int,
        orientation: str,
        range_m: int,
        speed_kmh: int,
        include_targets: bool,
        buffer_seconds: float,
    ) -> None:
        self.port = port
        self.baud = baud
        self.orientation = orientation
        self.range_m = range_m
        self.speed_kmh = speed_kmh
        self.include_targets = include_targets
        self.max_frames = max(32, int(40 * buffer_seconds))
        self._frames: deque[dict[str, Any]] = deque(maxlen=self.max_frames)
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._radar = None
        self.frame_count = 0
        self.radc_count = 0
        self.target_count = 0

    def connect(self) -> None:
        """Connect and configure the K-LD7."""
        try:
            from kld7 import KLD7
        except ImportError as error:
            raise RuntimeError("kld7 package missing; run with `uv run --extra kld7 ...`") from error

        try:
            from openflight.kld7.serial_io import connect_with_recovery
        except ImportError:
            connect_with_recovery = None

        if connect_with_recovery is not None:
            self._radar = connect_with_recovery(self.port, baudrate=self.baud, log=print)
        else:
            self._radar = KLD7(self.port, baudrate=self.baud)

        params = self._radar.params
        params.RRAI = RANGE_SETTINGS.get(self.range_m, 0)
        params.RSPI = SPEED_SETTINGS.get(self.speed_kmh, 3)
        params.DEDI = 2
        params.THOF = 10
        params.TRFT = 1
        params.MIAN = -90
        params.MAAN = 90
        params.MIRA = 0
        params.MARA = 100
        params.MISP = 0
        params.MASP = 100
        params.VISU = 0

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._radar is not None:
            try:
                self._radar.close()
            except Exception:
                pass
            try:
                self._radar._port = None
            except Exception:
                pass

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self._frames)

    def _append_frame(self, frame: dict[str, Any]) -> None:
        if not frame:
            return
        with self._lock:
            self._frames.append(frame)
        self.frame_count += 1

    def _stream_loop(self) -> None:
        from kld7 import FrameCode, KLD7Exception

        frame_codes = FrameCode.RADC
        if self.include_targets:
            frame_codes = frame_codes | FrameCode.PDAT | FrameCode.TDAT

        current_frame: dict[str, Any] = {"timestamp": time.time()}
        seen_in_frame: set[str] = set()

        while self._running:
            try:
                for code, payload in self._radar.stream_frames(frame_codes, max_count=-1):
                    if not self._running:
                        break

                    now = time.time()
                    if code in seen_in_frame:
                        self._append_frame(current_frame)
                        current_frame = {"timestamp": now}
                        seen_in_frame = set()
                    seen_in_frame.add(code)

                    if code == "RADC":
                        if not isinstance(payload, bytes) or len(payload) != RADC_PAYLOAD_BYTES:
                            continue
                        packet_timing = getattr(self._radar, "_openflight_last_packet_timing", {})
                        arrival_ts = (
                            packet_timing.get("arrival_timestamp")
                            if isinstance(packet_timing, dict)
                            else None
                        )
                        complete_ts = (
                            packet_timing.get("complete_timestamp")
                            if isinstance(packet_timing, dict)
                            else None
                        )
                        if arrival_ts is not None:
                            current_frame["timestamp"] = float(arrival_ts)
                            current_frame["radc_timestamp"] = float(arrival_ts)
                        else:
                            current_frame["radc_timestamp"] = now
                        if complete_ts is not None:
                            current_frame["radc_complete_timestamp"] = float(complete_ts)
                        current_frame["radc"] = payload
                        self.radc_count += 1
                    elif code == "TDAT":
                        current_frame["tdat"] = target_to_dict(payload)
                        if payload is not None:
                            self.target_count += 1
                    elif code == "PDAT":
                        targets = [target_to_dict(t) for t in payload] if payload else []
                        current_frame["pdat"] = [target for target in targets if target is not None]
                        self.target_count += len(current_frame["pdat"])
            except KLD7Exception as error:
                print(f"\n[KLD7] stream error: {error}; retrying")
                time.sleep(0.1)
            except Exception as error:  # pragma: no cover - hardware safety net
                print(f"\n[KLD7] stream crashed: {error}; retrying")
                time.sleep(0.1)


def serialize_frame(
    frame: dict[str, Any],
    *,
    trigger_timestamp: float,
    save_radc: bool,
    radc_metric_kwargs: dict[str, Any],
) -> dict[str, Any]:
    """Return JSON-safe frame details for one trigger window frame."""
    out = {
        "timestamp": frame.get("timestamp"),
        "t_ms": relative_ms(frame_timestamp(frame), trigger_timestamp),
        "tdat": frame.get("tdat"),
        "pdat": frame.get("pdat") or [],
        "pdat_count": len(frame.get("pdat") or []),
    }
    radc = frame.get("radc")
    if isinstance(radc, bytes):
        out["has_radc"] = True
        out["radc_timestamp"] = frame.get("radc_timestamp")
        out["radc_complete_timestamp"] = frame.get("radc_complete_timestamp")
        metric = radc_motion_metric(radc, **radc_metric_kwargs)
        if metric is not None:
            out["radc_metric"] = {
                "snr": round(metric.snr, 3),
                "peak_bin": metric.peak_bin,
                "peak_velocity_kmh": round(metric.peak_velocity_kmh, 3),
                "peak_magnitude": round(metric.peak_magnitude, 3),
                "median_magnitude": round(metric.median_magnitude, 3),
            }
        if save_radc:
            out["radc_b64"] = base64.b64encode(radc).decode("ascii")
            out["radc_payload_bytes"] = len(radc)
    else:
        out["has_radc"] = False
    return out


def configure_ops(args: argparse.Namespace) -> tuple[OPS243Radar, dict[str, Any] | None]:
    """Connect/configure OPS243 rolling-buffer trigger path."""
    radar = OPS243Radar(port=args.ops243_port)
    radar.connect()
    radar.configure_for_rolling_buffer(
        pre_trigger_segments=args.pre_trigger_segments,
        sample_rate_ksps=args.sample_rate_ksps,
    )
    clock_sync = None
    if not args.no_clock_sync:
        clock_sync = radar.read_clock_sync()
    return radar, clock_sync


def write_jsonl(path: Path, payload: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="K-LD7 motion onset diagnostic using OPS sound triggers",
    )
    parser.add_argument("--kld7-port", default="/dev/kld7_vertical")
    parser.add_argument("--ops243-port", default=None)
    parser.add_argument("--baud", type=int, default=3000000)
    parser.add_argument("--orientation", choices=["vertical", "horizontal"], default="vertical")
    parser.add_argument("--range", dest="range_m", type=int, default=5, choices=[5, 10, 30, 100])
    parser.add_argument("--speed", dest="speed_kmh", type=int, default=100, choices=[12, 25, 50, 100])
    parser.add_argument("--radc-only", action="store_true", help="Do not request PDAT/TDAT frames")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--duration", type=float, default=None, help="Optional total run duration")
    parser.add_argument("--trigger-timeout", type=float, default=30.0)
    parser.add_argument("--pre-trigger-segments", type=int, default=16)
    parser.add_argument("--sample-rate-ksps", type=int, default=30)
    parser.add_argument("--window-before", type=float, default=1.0)
    parser.add_argument("--window-after", type=float, default=1.0)
    parser.add_argument("--buffer-seconds", type=float, default=4.0)
    parser.add_argument("--radc-min-snr", type=float, default=8.0)
    parser.add_argument("--radc-baseline-factor", type=float, default=1.5)
    parser.add_argument("--radc-post-min-ms", type=float, default=-25.0)
    parser.add_argument("--radc-dc-mask-bins", type=int, default=5)
    parser.add_argument("--fft-size", type=int, default=2048)
    parser.add_argument("--no-clock-sync", action="store_true")
    parser.add_argument("--no-save-radc", action="store_true", help="Omit base64 RADC payloads")
    parser.add_argument("--output", type=Path, default=None)
    return parser


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    args = build_parser().parse_args()

    out_path = args.output
    if out_path is None:
        out_dir = Path.home() / "openflight_sessions"
        out_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = out_dir / f"kld7_motion_onset_{timestamp}.jsonl"
    else:
        out_path = out_path.expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
    if out_path.exists():
        out_path.unlink()

    include_targets = not args.radc_only
    kld7 = KLD7StreamBuffer(
        port=args.kld7_port,
        baud=args.baud,
        orientation=args.orientation,
        range_m=args.range_m,
        speed_kmh=args.speed_kmh,
        include_targets=include_targets,
        buffer_seconds=args.buffer_seconds,
    )

    print("=" * 72)
    print("  K-LD7 Motion Onset Diagnostic")
    print("=" * 72)
    print(f"  K-LD7:   {args.kld7_port} @ {args.baud} ({args.orientation})")
    print(f"  OPS243:  {args.ops243_port or 'auto-detect'}")
    print(f"  Frames:  {'RADC + PDAT/TDAT' if include_targets else 'RADC only'}")
    print(f"  Output:  {out_path}")
    print("=" * 72)

    write_jsonl(
        out_path,
        {
            "type": "session_start",
            "created_at": datetime.now().isoformat(),
            "args": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        },
    )

    ops = None
    try:
        print("Connecting K-LD7...")
        kld7.connect()
        kld7.start()
        time.sleep(0.5)
        print("Connecting OPS243...")
        ops, clock_sync = configure_ops(args)
        if clock_sync:
            print(
                "OPS clock sync: "
                f"method={clock_sync.get('clock_sync_method')} "
                f"usable={clock_sync.get('usable_for_trigger_timestamps')} "
                f"offset={clock_sync.get('best_offset_s')}"
            )

        processor = RollingBufferProcessor()
        start = time.monotonic()
        trial = 0
        while trial < args.trials:
            if args.duration is not None and time.monotonic() - start >= args.duration:
                break

            print(f"\n[{trial + 1}/{args.trials}] Waiting for sound trigger...")
            response = ops.wait_for_hardware_trigger(timeout=args.trigger_timeout)
            first_byte_ts = ops.last_hardware_trigger_first_byte_timestamp
            if not response:
                print("  timeout")
                continue

            capture = processor.parse_capture(response, first_byte_timestamp=first_byte_ts)
            trigger_timestamp = None
            trigger_source = None
            if capture is not None:
                if clock_sync and clock_sync.get("usable_for_trigger_timestamps"):
                    capture.apply_trigger_timestamp_from_clock_sync(float(clock_sync["best_offset_s"]))
                trigger_timestamp = capture.trigger_timestamp
                trigger_source = capture.trigger_timestamp_source
            elif first_byte_ts is not None:
                trigger_timestamp = first_byte_ts - post_trigger_duration_ms(args.pre_trigger_segments) / 1000.0
                trigger_source = "first_byte_fallback"

            try:
                ops.rearm_rolling_buffer(args.pre_trigger_segments)
            except Exception as error:
                print(f"  OPS re-arm failed: {error}")

            if trigger_timestamp is None:
                print("  trigger received but timestamp could not be inferred")
                continue

            trial += 1
            raw_frames = kld7.snapshot()
            start_ts = trigger_timestamp - args.window_before
            end_ts = trigger_timestamp + args.window_after
            window_raw = [
                frame
                for frame in raw_frames
                if start_ts <= frame_timestamp(frame) <= end_ts
            ]
            radc_metric_kwargs = {
                "fft_size": args.fft_size,
                "dc_mask_bins": args.radc_dc_mask_bins,
                "max_speed_kmh": float(args.speed_kmh),
            }
            frames = [
                serialize_frame(
                    frame,
                    trigger_timestamp=trigger_timestamp,
                    save_radc=not args.no_save_radc,
                    radc_metric_kwargs=radc_metric_kwargs,
                )
                for frame in window_raw
            ]
            target_summary = summarize_target_onset(frames, trigger_timestamp=trigger_timestamp)
            radc_summary = summarize_radc_onset(
                frames,
                trigger_timestamp=trigger_timestamp,
                min_snr=args.radc_min_snr,
                baseline_factor=args.radc_baseline_factor,
                post_min_ms=args.radc_post_min_ms,
            )

            payload = {
                "type": "trigger",
                "trial": trial,
                "trigger_timestamp": trigger_timestamp,
                "trigger_timestamp_source": trigger_source,
                "ops_first_byte_timestamp": first_byte_ts,
                "ops_response_bytes": len(response),
                "ops_sample_time": None if capture is None else capture.sample_time,
                "ops_trigger_time": None if capture is None else capture.trigger_time,
                "ops_post_trigger_duration_ms": (
                    None if capture is None else capture.post_trigger_duration_ms
                ),
                "frame_count": len(frames),
                "target_onset": target_summary,
                "radc_onset": radc_summary,
                "frames": frames,
            }
            write_jsonl(out_path, payload)

            first_target = target_summary["first_target_ms"]
            first_radc = radc_summary["first_motion"]["t_ms"] if radc_summary["first_motion"] else None
            strongest = (
                radc_summary["strongest_motion"]["t_ms"]
                if radc_summary["strongest_motion"]
                else None
            )
            print(
                "  frames=%d target_first=%s ms radc_first=%s ms radc_strongest=%s ms "
                "threshold=%.1f"
                % (
                    len(frames),
                    "n/a" if first_target is None else f"{first_target:.1f}",
                    "n/a" if first_radc is None else f"{first_radc:.1f}",
                    "n/a" if strongest is None else f"{strongest:.1f}",
                    radc_summary["threshold_snr"],
                )
            )

        write_jsonl(
            out_path,
            {
                "type": "session_end",
                "ended_at": datetime.now().isoformat(),
                "trials": trial,
                "kld7_frame_count": kld7.frame_count,
                "kld7_radc_count": kld7.radc_count,
                "kld7_target_count": kld7.target_count,
            },
        )
        print(f"\nSaved: {out_path}")
        return 0
    finally:
        if ops is not None:
            try:
                ops.disconnect()
            except Exception:
                pass
        kld7.stop()


if __name__ == "__main__":
    raise SystemExit(main())
