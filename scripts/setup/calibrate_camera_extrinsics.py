#!/usr/bin/env python3
"""
One-time calibration: solve the camera -> K-LD7 extrinsic transform.

For each calibration sample the user:
  1. Places a golf ball at a measured (L, d) position relative to the K-LD7
     antenna face. L is signed lateral offset (+ right of antenna centerline);
     d is forward distance from the antenna face.
  2. Enters those values at the prompt.
  3. Confirms ball detection in the live preview, then presses SPACE to record.

After at least --min-samples (default 3) good samples, the script solves for
the radar antenna origin in the camera frame and writes the result as JSON.

Translation-only model: we assume the camera and radar are rigidly mounted
in the same orientation (lens axis ~parallel to radar boresight). The
aim-viewer focal-length calibration plus a mechanical jig satisfies this for
the co-located camera + K-LD7 setup described in the camera-aim plan. Rotation
estimation is out of scope for this script — if your camera is rotated
relative to the radar, mount it straight or extend this script.

Output JSON shape:

    {
      "version": 1,
      "calibrated_at": "<iso8601>",
      "focal_px": 800.0,
      "resolution": [640, 480],
      "ball_diameter_in": 1.68,
      "radars": {
        "horizontal_kld7": {
          "radar_origin_in_camera_frame_in": {"x": -3.2, "y": 4.5, "z": 1.5},
          "samples": [ ... ],
          "residuals_in": {"x_rms": 0.12, "z_rms": 0.05}
        }
      }
    }

The radar origin (X, Y, Z) is the location of the radar antenna face in the
camera's coordinate frame (X right+, Y up+, Z forward+). A ball position
in the camera frame is converted to the radar frame by subtracting this
vector:  (L, h, d) = (x_cam - X, y_cam - Y, depth_cam - Z).

Usage:
    uv run python scripts/setup/calibrate_camera_extrinsics.py \\
        --radar horizontal --focal-px 850
    uv run python scripts/setup/calibrate_camera_extrinsics.py \\
        --radar horizontal --usb --config-out /tmp/calib.json
"""

import argparse
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    cv2 = None  # type: ignore
    CV2_AVAILABLE = False

try:
    import numpy as np  # noqa: F401
except ImportError:
    np = None  # type: ignore

try:
    from openflight.camera.capture import CapturedFrame
    from openflight.camera.detector import BallDetector, DetectorConfig
    CAMERA_MODULES_AVAILABLE = True
except ImportError:
    CapturedFrame = None  # type: ignore
    BallDetector = None  # type: ignore
    DetectorConfig = None  # type: ignore
    CAMERA_MODULES_AVAILABLE = False


BALL_DIAMETER_IN = 1.68
DEFAULT_CONFIG_PATH = Path.home() / "openflight_calibration.json"
RADAR_CHOICES = ["horizontal_kld7", "vertical_kld7"]


def parse_resolution(value: str):
    try:
        w_s, h_s = value.lower().split("x")
        return int(w_s), int(h_s)
    except (ValueError, IndexError):
        raise argparse.ArgumentTypeError(
            f"Invalid resolution '{value}'. Use WIDTHxHEIGHT (e.g. 640x480)."
        )


def parse_range(value: str):
    try:
        a_s, b_s = value.split(",")
        a, b = float(a_s), float(b_s)
        if a >= b:
            raise ValueError("min must be < max")
        return (a, b)
    except (ValueError, IndexError) as e:
        raise argparse.ArgumentTypeError(
            f"Invalid range '{value}'. Use MIN,MAX with MIN < MAX (e.g. -16,16). ({e})"
        )


def compute_roi_pixels(width, height, focal_px, x_range, y_range, max_radius):
    if x_range is None and y_range is None:
        return None
    min_depth_in = (BALL_DIAMETER_IN * focal_px) / (2 * max_radius)
    if min_depth_in <= 0:
        return None
    cx_center, cy_center = width / 2.0, height / 2.0
    margin = max_radius + 4
    if x_range is not None:
        x_min_in, x_max_in = x_range
        x1 = int(cx_center + (x_min_in * focal_px / min_depth_in) - margin)
        x2 = int(cx_center + (x_max_in * focal_px / min_depth_in) + margin)
    else:
        x1, x2 = 0, width
    if y_range is not None:
        y_min_in, y_max_in = y_range
        y1 = int(cy_center - (y_max_in * focal_px / min_depth_in) - margin)
        y2 = int(cy_center - (y_min_in * focal_px / min_depth_in) + margin)
    else:
        y1, y2 = 0, height
    x1 = max(0, min(width, x1))
    x2 = max(0, min(width, x2))
    y1 = max(0, min(height, y1))
    y2 = max(0, min(height, y2))
    if x1 >= x2 or y1 >= y2:
        return None
    return (x1, y1, x2, y2)


def apply_roi_mask(frame_data, roi):
    if roi is None:
        return frame_data
    x1, y1, x2, y2 = roi
    masked = np.zeros_like(frame_data)
    masked[y1:y2, x1:x2] = frame_data[y1:y2, x1:x2]
    return masked


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--radar", choices=RADAR_CHOICES, default="horizontal_kld7",
                   help="Which K-LD7 to calibrate against (default: horizontal_kld7)")
    p.add_argument("--focal-px", type=float, required=True,
                   help="Camera focal length in pixels — get this from camera_aim_viewer.py")
    p.add_argument("--resolution", type=parse_resolution, default=(640, 480),
                   help="Camera resolution WIDTHxHEIGHT (default: 640x480) — must match focal-px calibration")
    p.add_argument("--framerate", type=int, default=30,
                   help="Camera framerate (default: 30)")
    p.add_argument("--min-samples", type=int, default=3,
                   help="Minimum samples before solving (default: 3)")
    p.add_argument("--brightness-threshold", type=int, default=180)
    p.add_argument("--hough-param2", type=int, default=20)
    p.add_argument("--min-radius", type=int, default=5)
    p.add_argument("--max-radius", type=int, default=80)
    p.add_argument("--min-confidence", type=float, default=0.5,
                   help="Min detection confidence 0-1 (default: 0.5). Lower for non-IR/bright backgrounds.")
    p.add_argument("--rotate-180", action="store_true",
                   help="Flip image 180° (use if camera is mounted upside-down)")
    p.add_argument("--usb", action="store_true",
                   help="Skip picamera2 and use OpenCV VideoCapture")
    p.add_argument("--config-out", type=Path, default=DEFAULT_CONFIG_PATH,
                   help=f"Output JSON path (default: {DEFAULT_CONFIG_PATH})")
    p.add_argument("--dry-run", action="store_true",
                   help="Solve and print result but do not write the JSON file")
    p.add_argument("--auto-record-sec", type=float, default=0.0,
                   help="Auto-record after detection is stable for N seconds (0 = manual SPACE only). "
                        "Useful when running headless or without a keyboard.")
    p.add_argument("--ball-x-range", type=parse_range, default=None,
                   help="Expected ball X position in inches as MIN,MAX (e.g. -16,16). Restricts detection to this region.")
    p.add_argument("--ball-y-range", type=parse_range, default=None,
                   help="Expected ball Y position in inches as MIN,MAX (e.g. -8,8). Restricts detection to this region.")
    p.add_argument("--display-scale", type=float, default=1.0,
                   help="Scale the live display window by this factor (e.g. 2.0 for 2x zoom). Detection unaffected.")
    return p.parse_args()


def open_camera(width, height, framerate, force_usb=False):
    if not force_usb:
        try:
            from picamera2 import Picamera2  # type: ignore
            cam = Picamera2()
            cfg = cam.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"},
                controls={"FrameRate": framerate},
            )
            cam.configure(cfg)
            cam.start()
            print(f"camera: picamera2 {width}x{height} @ {framerate}fps")
            return cam, "picamera2"
        except (ImportError, RuntimeError) as e:
            print(f"picamera2 unavailable ({e}); falling back to USB webcam")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: no camera available", file=sys.stderr)
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, framerate)
    print(f"camera: USB/OpenCV {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    return cap, "opencv"


def grab_frame(camera, camera_type, frame_number, rotate_180=False):
    if camera_type == "picamera2":
        data = camera.capture_array()
    else:
        ok, data = camera.read()
        if not ok:
            raise RuntimeError("failed to capture frame")
        data = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
    if rotate_180:
        data = cv2.rotate(data, cv2.ROTATE_180)
    return CapturedFrame(data=data, timestamp=time.time(), frame_number=frame_number)


def close_camera(camera, camera_type):
    if camera_type == "picamera2":
        camera.stop()
        camera.close()
    else:
        camera.release()


def detection_to_cam_xyz(detection, frame_w, frame_h, focal_px):
    """Return (x_cam, y_cam, depth_cam) in inches, or None if no detection."""
    if detection is None:
        return None
    pixel_diameter = max(1.0, 2.0 * detection.radius)
    depth_in = (BALL_DIAMETER_IN * focal_px) / pixel_diameter
    dx = detection.x - frame_w / 2.0
    dy = detection.y - frame_h / 2.0
    x_in = (dx / focal_px) * depth_in
    y_in = -(dy / focal_px) * depth_in
    return x_in, y_in, depth_in


def annotate_frame(frame_rgb, detection, frame_w, frame_h, cam_xyz, target_L, target_d, sample_idx, total_needed):
    img = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    cx_c, cy_c = frame_w // 2, frame_h // 2
    cv2.line(img, (cx_c - 12, cy_c), (cx_c + 12, cy_c), (0, 255, 255), 1)
    cv2.line(img, (cx_c, cy_c - 12), (cx_c, cy_c + 12), (0, 255, 255), 1)

    if detection is not None:
        cx, cy, r = int(detection.x), int(detection.y), int(detection.radius)
        cv2.circle(img, (cx, cy), r, (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 2, (0, 0, 255), -1)

    overlay = [
        f"sample {sample_idx + 1}/{total_needed}",
        f"target L={target_L:+.2f}in  d={target_d:.2f}in",
    ]
    if cam_xyz is not None:
        x_in, y_in, depth_in = cam_xyz
        overlay.append(f"meas   x={x_in:+.2f}in  depth={depth_in:.2f}in  y={y_in:+.2f}in")
        overlay.append(f"conf   {detection.confidence:.2f}")
    else:
        overlay.append("NO BALL DETECTED")

    overlay.append("SPACE=record  R=retake  ESC=abort")

    y = 22
    for line in overlay:
        cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (40, 40, 40), 1, cv2.LINE_AA)
        y += 22
    return img


def live_capture_one(camera, camera_type, detector, focal_px, frame_w, frame_h,
                     target_L, target_d, sample_idx, total_needed,
                     rotate_180=False, min_confidence=0.4, auto_record_sec=0.0,
                     roi=None, display_scale=1.0):
    """
    Show live preview until user presses SPACE on a detected frame, OR until
    a stable detection holds for auto_record_sec seconds (if > 0).
    Returns (x_cam, y_cam, depth_cam) or None if aborted.
    """
    frame_no = 0
    last_xyz = None
    last_detection = None
    stable_since = None
    stable_xyz = None
    STABLE_PX_TOLERANCE = 3.0  # pixel center may drift by this much and still be "stable"
    while True:
        frame = grab_frame(camera, camera_type, frame_no, rotate_180=rotate_180)
        frame_no += 1
        if roi is not None:
            masked_data = apply_roi_mask(frame.data, roi)
            det_frame = CapturedFrame(data=masked_data, timestamp=frame.timestamp,
                                      frame_number=frame.frame_number)
        else:
            det_frame = frame
        detection = detector.detect(det_frame)
        xyz = detection_to_cam_xyz(detection, frame_w, frame_h, focal_px)
        last_xyz, last_detection = xyz, detection

        if auto_record_sec > 0 and detection is not None and xyz is not None:
            if stable_xyz is None:
                stable_xyz = (detection.x, detection.y, time.time())
                stable_since = time.time()
            else:
                sx, sy, _ = stable_xyz
                if abs(detection.x - sx) < STABLE_PX_TOLERANCE and abs(detection.y - sy) < STABLE_PX_TOLERANCE:
                    if time.time() - stable_since >= auto_record_sec:
                        print(f"  auto-recorded after {auto_record_sec:.1f}s stable")
                        return xyz
                else:
                    stable_xyz = (detection.x, detection.y, time.time())
                    stable_since = time.time()
        elif auto_record_sec > 0:
            stable_xyz = None
            stable_since = None

        annotated = annotate_frame(frame.data, detection, frame_w, frame_h, xyz,
                                   target_L, target_d, sample_idx, total_needed)
        if roi is not None:
            cv2.rectangle(annotated, (roi[0], roi[1]), (roi[2] - 1, roi[3] - 1),
                          (0, 200, 255), 1)
        if display_scale != 1.0 and display_scale > 0:
            new_w = int(annotated.shape[1] * display_scale)
            new_h = int(annotated.shape[0] * display_scale)
            annotated = cv2.resize(annotated, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("calibrate_extrinsics", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            return None
        if key == ord(" "):
            if last_detection is None or last_xyz is None:
                print("  no detection in current frame — try again")
                continue
            if last_detection.confidence < min_confidence:
                print(f"  low confidence ({last_detection.confidence:.2f}) — recapture or adjust thresholds")
                continue
            return last_xyz
        if key in (ord("r"), ord("R")):
            continue


def solve_extrinsic(samples):
    """
    Estimate radar origin (X, Y, Z) in camera frame.

    From each sample: cam_xyz - radar_origin = target_radar_frame.
    With target = (L, h, d) (h unknown, irrelevant for L/d aim correction):
        X = mean(x_cam_i - L_i)
        Z = mean(depth_cam_i - d_i)
        Y = mean(y_cam_i)    # h_i is unknown, so Y is the mean y of all balls;
                              # this is informational only — the aim correction
                              # does not depend on it.
    Returns (origin_xyz, residuals) where residuals = (x_rms, z_rms).
    """
    xs = [s["x_cam_in"] - s["target_L_in"] for s in samples]
    zs = [s["depth_cam_in"] - s["target_d_in"] for s in samples]
    ys = [s["y_cam_in"] for s in samples]
    X = statistics.fmean(xs)
    Z = statistics.fmean(zs)
    Y = statistics.fmean(ys)
    x_rms = math.sqrt(statistics.fmean([(x - X) ** 2 for x in xs])) if len(xs) > 1 else 0.0
    z_rms = math.sqrt(statistics.fmean([(z - Z) ** 2 for z in zs])) if len(zs) > 1 else 0.0
    return (X, Y, Z), (x_rms, z_rms)


def read_float(prompt):
    while True:
        raw = input(prompt).strip()
        try:
            return float(raw)
        except ValueError:
            print("  not a number, try again")


def merge_existing_config(path: Path):
    if not path.exists():
        return {}
    try:
        with path.open("r") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return {}
        return data
    except (json.JSONDecodeError, OSError) as e:
        print(f"warning: could not read existing {path}: {e}")
        return {}


def atomic_write_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
    tmp.replace(path)


def main():
    if not CV2_AVAILABLE or not CAMERA_MODULES_AVAILABLE:
        print("ERROR: this script requires opencv-python and the openflight.camera package.",
              file=sys.stderr)
        print("Install on the Pi with: uv sync --extra camera", file=sys.stderr)
        sys.exit(1)
    args = parse_args()
    width, height = args.resolution

    det_cfg = DetectorConfig(
        brightness_threshold=args.brightness_threshold,
        hough_param2=args.hough_param2,
        min_radius=args.min_radius,
        max_radius=args.max_radius,
        min_confidence=args.min_confidence,
    )
    detector = BallDetector(det_cfg)
    camera, ctype = open_camera(width, height, args.framerate, force_usb=args.usb)
    roi = compute_roi_pixels(width, height, args.focal_px,
                             args.ball_x_range, args.ball_y_range, args.max_radius)
    if roi is not None:
        print(f"ROI active: pixels ({roi[0]}, {roi[1]}) -> ({roi[2]}, {roi[3]})  "
              f"(x_range={args.ball_x_range}in, y_range={args.ball_y_range}in)")

    cv2.namedWindow("calibrate_extrinsics", cv2.WINDOW_NORMAL)
    disp_w = int(width * args.display_scale)
    disp_h = int(height * args.display_scale)
    cv2.resizeWindow("calibrate_extrinsics", disp_w, disp_h)

    print("")
    print("=" * 64)
    print(f"Camera -> {args.radar} extrinsic calibration")
    print("=" * 64)
    print("For each sample:")
    print("  1. Place a golf ball at a known (L, d) position relative to the")
    print("     radar antenna face.")
    print("       L = signed lateral offset in inches (+ right of centerline)")
    print("       d = forward distance from antenna face in inches")
    print("  2. Enter the (L, d) you measured.")
    print("  3. Confirm the ball is detected (green circle), press SPACE.")
    print("")
    print("Recommended:")
    print("  - At least 1 sample on the centerline at a near distance (L=0, d~48in)")
    print("  - At least 1 sample on the centerline at a far distance  (L=0, d~96in)")
    print("  - 1 off-centerline sample to verify sign convention      (e.g. L=+6in)")
    print("")

    samples = []
    try:
        sample_idx = 0
        while True:
            need_more = sample_idx < args.min_samples
            if not need_more:
                more = input(f"\nHave {sample_idx} samples. Add another? (y/N): ").strip().lower()
                if not more.startswith("y"):
                    break
            print(f"\n--- Sample {sample_idx + 1} ---")
            L_in = read_float("  Ball L (in, signed, + right) = ")
            d_in = read_float("  Ball d (in, forward)         = ")
            print("  Adjust the ball position visually. Press SPACE in the window to record.")
            captured = live_capture_one(camera, ctype, detector, args.focal_px,
                                        width, height, L_in, d_in,
                                        sample_idx, max(args.min_samples, sample_idx + 1),
                                        rotate_180=args.rotate_180,
                                        min_confidence=args.min_confidence,
                                        auto_record_sec=args.auto_record_sec,
                                        roi=roi,
                                        display_scale=args.display_scale)
            if captured is None:
                print("  aborted by user")
                return
            x_cam, y_cam, depth_cam = captured
            samples.append({
                "target_L_in": L_in,
                "target_d_in": d_in,
                "x_cam_in": x_cam,
                "y_cam_in": y_cam,
                "depth_cam_in": depth_cam,
                "captured_at": datetime.now().isoformat(timespec="seconds"),
            })
            print(f"  recorded: cam x={x_cam:+.2f} y={y_cam:+.2f} depth={depth_cam:.2f}")
            sample_idx += 1
    finally:
        close_camera(camera, ctype)
        cv2.destroyAllWindows()

    if len(samples) < 2:
        print("ERROR: need at least 2 samples to solve. Got 0 or 1. Aborting.")
        sys.exit(2)

    depths = [s["target_d_in"] for s in samples]
    if max(depths) - min(depths) < 12.0:
        print(f"WARNING: depth spread is only {max(depths) - min(depths):.1f}in.")
        print("         Z estimate will be ill-conditioned. Recommend re-running with")
        print("         samples spaced 24+ inches apart in depth.")

    (X, Y, Z), (x_rms, z_rms) = solve_extrinsic(samples)

    print("")
    print("=" * 64)
    print("Solve result")
    print("=" * 64)
    print(f"  radar_origin_in_camera_frame:  X={X:+.3f}in  Y={Y:+.3f}in  Z={Z:+.3f}in")
    print(f"  residuals:                     x_rms={x_rms:.3f}in  z_rms={z_rms:.3f}in")
    print("")
    print("  Sanity check: a ball detected at the same camera (x, y, depth)")
    print("  as one of your samples should give (L, d) close to the target you entered.")
    for i, s in enumerate(samples):
        L_pred = s["x_cam_in"] - X
        d_pred = s["depth_cam_in"] - Z
        print(f"    sample {i + 1}: target L={s['target_L_in']:+.2f} d={s['target_d_in']:.2f}"
              f"   predicted L={L_pred:+.2f} d={d_pred:.2f}"
              f"   err L={L_pred - s['target_L_in']:+.2f} d={d_pred - s['target_d_in']:+.2f}")

    if args.dry_run:
        print("\n--dry-run: not writing config file")
        return

    existing = merge_existing_config(args.config_out)
    radars = existing.get("radars", {}) if isinstance(existing.get("radars"), dict) else {}
    radars[args.radar] = {
        "radar_origin_in_camera_frame_in": {"x": X, "y": Y, "z": Z},
        "samples": samples,
        "residuals_in": {"x_rms": x_rms, "z_rms": z_rms},
    }
    payload = {
        "version": 1,
        "calibrated_at": datetime.now().isoformat(timespec="seconds"),
        "focal_px": args.focal_px,
        "resolution": [width, height],
        "ball_diameter_in": BALL_DIAMETER_IN,
        "radars": radars,
    }
    atomic_write_json(args.config_out, payload)
    print(f"\nwrote {args.config_out}")


if __name__ == "__main__":
    main()
