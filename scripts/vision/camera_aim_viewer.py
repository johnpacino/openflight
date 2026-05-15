#!/usr/bin/env python3
"""
Live camera-aim viewer.

Split-screen visualization for camera + ball-position bring-up:
  Left  half : live camera frame with detection overlay (circle, crosshair,
               sign-convention legend).
  Right half : stats panel — pixel coords, world X/Y in inches relative to
               the camera optical centerline, and depth from camera.

Sign convention (matches the K-LD7 aim-correction plan):
  X axis: 0 at center, + to the RIGHT,  - to the LEFT.
  Y axis: 0 at center, + UPWARD,         - DOWNWARD.
  Depth : forward distance from camera, in inches.

Depth recovery uses the known golf-ball diameter (1.68 in) and a calibrated
focal length in pixels. Default --focal-px is a rough starting point for the
HQ Camera + 6mm CS lens at 640x480; for accurate readings, calibrate by
placing a ball at a known distance and tuning until the displayed depth
matches the tape-measured distance.

Usage:
    uv run python scripts/vision/camera_aim_viewer.py
    uv run python scripts/vision/camera_aim_viewer.py --focal-px 850
    uv run python scripts/vision/camera_aim_viewer.py --resolution 1280x720
    uv run python scripts/vision/camera_aim_viewer.py --usb
    uv run python scripts/vision/camera_aim_viewer.py --headless --num-frames 200

Keys (display mode):
    q / ESC      quit
    s            save annotated snapshot to ./aim_viewer_snapshots/
    p / P        decrease / increase hough_param2 (detection sensitivity)
    t / T        decrease / increase brightness threshold
    f / F        decrease / increase focal-px by 10
    r            reset config + focal-px to startup defaults
"""

import argparse
import os
import sys
import time
from pathlib import Path

if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

import cv2
import numpy as np

from openflight.camera.capture import CapturedFrame
from openflight.camera.detector import BallDetector, DetectorConfig


BALL_DIAMETER_IN = 1.68  # USGA spec
SNAPSHOT_DIR = Path("aim_viewer_snapshots")


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
            f"Invalid range '{value}'. Use MIN,MAX with MIN < MAX (e.g. -8,4). ({e})"
        )


def compute_roi_pixels(width, height, focal_px, x_range, y_range, max_radius):
    """
    Convert world-inches ranges to pixel rectangle, conservative across the depth range.

    Returns (x1, y1, x2, y2) clamped to frame, or None if no constraints given.
    """
    if x_range is None and y_range is None:
        return None

    # Worst case (widest pixel projection) is at the smallest possible depth.
    # Smallest depth corresponds to the largest pixel radius we allow.
    min_depth_in = (BALL_DIAMETER_IN * focal_px) / (2 * max_radius)
    if min_depth_in <= 0:
        return None

    cx_center = width / 2.0
    cy_center = height / 2.0
    margin = max_radius + 4  # pixel buffer so a ball at the bound isn't clipped

    if x_range is not None:
        x_min_in, x_max_in = x_range
        x1 = int(cx_center + (x_min_in * focal_px / min_depth_in) - margin)
        x2 = int(cx_center + (x_max_in * focal_px / min_depth_in) + margin)
    else:
        x1, x2 = 0, width

    if y_range is not None:
        y_min_in, y_max_in = y_range
        # World Y+ = up, image y+ = down, so image_y = center - y_in * f / depth.
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
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--resolution", type=parse_resolution, default=(640, 480),
                   help="Camera resolution WIDTHxHEIGHT (default: 640x480)")
    p.add_argument("--framerate", type=int, default=60,
                   help="Target framerate (default: 60)")
    p.add_argument("--focal-px", type=float, default=800.0,
                   help="Camera focal length in pixels — calibrate per lens/resolution (default: 800)")
    p.add_argument("--hough-param2", type=int, default=20,
                   help="Hough circle detection threshold (default: 20)")
    p.add_argument("--brightness-threshold", type=int, default=180,
                   help="Brightness threshold for ball isolation 0-255 (default: 180)")
    p.add_argument("--min-radius", type=int, default=5,
                   help="Min ball radius in pixels (default: 5)")
    p.add_argument("--max-radius", type=int, default=80,
                   help="Max ball radius in pixels (default: 80)")
    p.add_argument("--min-confidence", type=float, default=0.5,
                   help="Min detection confidence 0-1 (default: 0.5). Lower for non-IR/bright backgrounds.")
    p.add_argument("--usb", action="store_true",
                   help="Skip picamera2 and use OpenCV VideoCapture directly")
    p.add_argument("--headless", action="store_true",
                   help="No display window, print stats to stdout")
    p.add_argument("--num-frames", type=int, default=0,
                   help="Stop after N frames (default: 0 = run until 'q')")
    p.add_argument("--rotate-180", action="store_true",
                   help="Flip image 180° (use if camera is mounted upside-down)")
    p.add_argument("--save-every-sec", type=float, default=0.0,
                   help="Auto-save snapshot every N seconds (0 = disabled). Useful when no keyboard is attached.")
    p.add_argument("--ball-x-range", type=parse_range, default=None,
                   help="Expected ball X position in inches as MIN,MAX (e.g. -8,4). Restricts detection to this region.")
    p.add_argument("--ball-y-range", type=parse_range, default=None,
                   help="Expected ball Y position in inches as MIN,MAX (e.g. -3,3). Restricts detection to this region.")
    return p.parse_args()


def open_camera(width, height, framerate, force_usb=False):
    if not force_usb:
        try:
            from picamera2 import Picamera2  # type: ignore
            cam = Picamera2()
            cfg = cam.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"},
                controls={
                    "FrameRate": framerate,
                    "AeEnable": True,
                    "AwbEnable": True,
                },
            )
            cam.configure(cfg)
            cam.start()
            print(f"Opened Pi camera (picamera2) at {width}x{height} @ {framerate}fps")
            return cam, "picamera2"
        except (ImportError, RuntimeError) as e:
            print(f"picamera2 unavailable ({e}); falling back to USB webcam...")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Could not open any camera (picamera2 and OpenCV both failed).")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, framerate)
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Opened USB webcam (OpenCV) at {aw}x{ah} (requested {width}x{height})")
    return cap, "opencv"


def capture_frame(camera, camera_type, frame_number, rotate_180=False):
    if camera_type == "picamera2":
        data = camera.capture_array()
    else:
        ok, data = camera.read()
        if not ok:
            raise RuntimeError("Failed to capture frame from webcam")
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


def compute_geometry(detection, frame_w, frame_h, focal_px):
    """
    Convert detected ball pixel coords to world coords in camera frame.

    Returns dict with x_in (right+), y_in (up+), depth_in, pixel_diameter,
    or None if detection is None or geometry is degenerate.
    """
    if detection is None:
        return None
    pixel_diameter = max(1.0, 2.0 * detection.radius)
    depth_in = (BALL_DIAMETER_IN * focal_px) / pixel_diameter

    cx_off = detection.x - frame_w / 2.0  # + = right in image
    cy_off = detection.y - frame_h / 2.0  # + = down in image
    x_in = (cx_off / focal_px) * depth_in
    y_in = -(cy_off / focal_px) * depth_in  # flip so + = up

    return {
        "x_in": x_in,
        "y_in": y_in,
        "depth_in": depth_in,
        "pixel_diameter": pixel_diameter,
    }


def render_left_panel(frame_rgb, detection, frame_w, frame_h, roi=None):
    """Annotated camera frame (BGR for OpenCV display)."""
    img = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    if roi is not None:
        x1, y1, x2, y2 = roi
        cv2.rectangle(img, (x1, y1), (x2 - 1, y2 - 1), (0, 200, 255), 1)

    cx_center = frame_w // 2
    cy_center = frame_h // 2
    cv2.line(img, (cx_center - 12, cy_center), (cx_center + 12, cy_center), (0, 255, 255), 1)
    cv2.line(img, (cx_center, cy_center - 12), (cx_center, cy_center + 12), (0, 255, 255), 1)
    cv2.circle(img, (cx_center, cy_center), 3, (0, 255, 255), -1)

    if detection is not None:
        cx, cy, r = int(detection.x), int(detection.y), int(detection.radius)
        cv2.circle(img, (cx, cy), r, (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 2, (0, 0, 255), -1)
        cv2.line(img, (cx_center, cy_center), (cx, cy), (255, 0, 0), 1)
    else:
        cv2.putText(img, "no detection", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

    cv2.putText(img, "X+ -> right   Y+ -> up", (8, frame_h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return img


def render_right_panel(width, height, detection, geom, fps, det_cfg, focal_px):
    """Stats text panel as a BGR image of (height x width)."""
    panel = np.full((height, width, 3), 32, dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    line = 28
    y = 30

    def put(text, color=(220, 220, 220), scale=0.55, thick=1):
        nonlocal y
        cv2.putText(panel, text, (14, y), font, scale, color, thick, cv2.LINE_AA)
        y += line

    put("CAMERA AIM VIEWER", (255, 255, 255), 0.7, 2)
    y += 6
    put(f"FPS:   {fps:5.1f}", (180, 220, 255))
    y += 6

    if detection is not None and geom is not None:
        put("PIXELS", (255, 255, 0), 0.55, 2)
        put(f"  cx, cy : ({detection.x:6.1f}, {detection.y:6.1f})")
        put(f"  radius : {detection.radius:5.1f} px")
        put(f"  pix-d  : {geom['pixel_diameter']:5.1f} px")
        put(f"  conf   : {detection.confidence:0.2f}")
        y += 6
        put("WORLD (camera frame)", (0, 255, 0), 0.55, 2)
        x_color = (120, 255, 120) if abs(geom["x_in"]) < 0.5 else (120, 200, 255)
        y_color = (120, 255, 120) if abs(geom["y_in"]) < 0.5 else (120, 200, 255)
        put(f"  X : {geom['x_in']:+7.2f} in", x_color, 0.6, 2)
        put(f"  Y : {geom['y_in']:+7.2f} in", y_color, 0.6, 2)
        put(f"  depth : {geom['depth_in']:6.2f} in", (255, 200, 100), 0.6, 2)
        put(f"        ({geom['depth_in']/12.0:5.2f} ft)", (180, 180, 180))
    else:
        put("NO BALL DETECTED", (60, 60, 255), 0.65, 2)
        y += 6
        put("- ball outside FOV?", (200, 200, 200))
        put("- too small (<min_radius)?", (200, 200, 200))
        put("- low contrast / lighting?", (200, 200, 200))
        put("- adjust thresholds (T/P)", (200, 200, 200))

    y = height - 8 * line
    put("---- config ----", (160, 160, 160), 0.5, 1)
    put(f"focal_px       : {focal_px:6.1f}   (f/F)", (200, 200, 200), 0.5)
    put(f"hough_param2   : {det_cfg.hough_param2}        (p/P)", (200, 200, 200), 0.5)
    put(f"bright_thresh  : {det_cfg.brightness_threshold}        (t/T)", (200, 200, 200), 0.5)
    put(f"min/max radius : {det_cfg.min_radius} / {det_cfg.max_radius}", (200, 200, 200), 0.5)
    put("keys: q quit  s snapshot  r reset", (160, 160, 160), 0.45)

    return panel


def handle_key(key, det_cfg, focal_px, defaults):
    """Returns (should_quit, det_cfg, focal_px, action_label_or_None)."""
    if key in (ord("q"), 27):
        return True, det_cfg, focal_px, None
    if key == ord("p"):
        det_cfg.hough_param2 = max(5, det_cfg.hough_param2 - 1)
        return False, det_cfg, focal_px, f"hough_param2={det_cfg.hough_param2}"
    if key == ord("P"):
        det_cfg.hough_param2 = min(200, det_cfg.hough_param2 + 1)
        return False, det_cfg, focal_px, f"hough_param2={det_cfg.hough_param2}"
    if key == ord("t"):
        det_cfg.brightness_threshold = max(0, det_cfg.brightness_threshold - 5)
        return False, det_cfg, focal_px, f"bright_thresh={det_cfg.brightness_threshold}"
    if key == ord("T"):
        det_cfg.brightness_threshold = min(255, det_cfg.brightness_threshold + 5)
        return False, det_cfg, focal_px, f"bright_thresh={det_cfg.brightness_threshold}"
    if key == ord("f"):
        focal_px = max(50.0, focal_px - 10.0)
        return False, det_cfg, focal_px, f"focal_px={focal_px}"
    if key == ord("F"):
        focal_px = focal_px + 10.0
        return False, det_cfg, focal_px, f"focal_px={focal_px}"
    if key == ord("r"):
        return False, DetectorConfig(**defaults["det"]), defaults["focal_px"], "reset"
    return False, det_cfg, focal_px, "save" if key == ord("s") else None


def save_snapshot(combined_bgr, snapshot_idx):
    SNAPSHOT_DIR.mkdir(exist_ok=True)
    fp = SNAPSHOT_DIR / f"aim_{int(time.time())}_{snapshot_idx:03d}.png"
    cv2.imwrite(str(fp), combined_bgr)
    print(f"saved {fp}")


def main():
    args = parse_args()
    width, height = args.resolution

    det_cfg = DetectorConfig(
        brightness_threshold=args.brightness_threshold,
        hough_param2=args.hough_param2,
        min_radius=args.min_radius,
        max_radius=args.max_radius,
        min_confidence=args.min_confidence,
    )
    defaults = {
        "det": dict(
            brightness_threshold=args.brightness_threshold,
            hough_param2=args.hough_param2,
            min_radius=args.min_radius,
            max_radius=args.max_radius,
            min_confidence=args.min_confidence,
        ),
        "focal_px": args.focal_px,
    }
    detector = BallDetector(det_cfg)
    focal_px = args.focal_px

    camera, camera_type = open_camera(width, height, args.framerate, force_usb=args.usb)

    roi = compute_roi_pixels(width, height, focal_px,
                             args.ball_x_range, args.ball_y_range, args.max_radius)
    if roi is not None:
        print(f"ROI active: pixels ({roi[0]}, {roi[1]}) -> ({roi[2]}, {roi[3]})  "
              f"(x_range={args.ball_x_range}in, y_range={args.ball_y_range}in)")
    elif args.ball_x_range or args.ball_y_range:
        print("WARNING: ROI was requested but computed to be empty/invalid; falling back to full frame.")

    if not args.headless:
        cv2.namedWindow("aim_viewer", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("aim_viewer", width * 2, height)

    print("\nRunning. Press 'q' (or ESC) in the window to quit.")
    print("Stats panel shows world X/Y from camera optical center; depth via known ball diameter.\n")

    frame_count = 0
    snapshot_count = 0
    fps_window = []
    last_print = 0.0
    last_auto_save = 0.0

    try:
        while True:
            t0 = time.time()
            frame = capture_frame(camera, camera_type, frame_count, rotate_180=args.rotate_180)
            detector.config = det_cfg  # in case keys mutated it
            if roi is not None:
                masked_data = apply_roi_mask(frame.data, roi)
                detection_frame = CapturedFrame(data=masked_data, timestamp=frame.timestamp,
                                                frame_number=frame.frame_number)
            else:
                detection_frame = frame
            detection = detector.detect(detection_frame)
            geom = compute_geometry(detection, width, height, focal_px)

            t1 = time.time()
            fps_window.append(t1 - t0)
            if len(fps_window) > 20:
                fps_window.pop(0)
            fps = (1.0 / np.mean(fps_window)) if fps_window else 0.0

            left = render_left_panel(frame.data, detection, width, height, roi=roi)
            right = render_right_panel(width, height, detection, geom, fps, det_cfg, focal_px)
            combined = np.hstack([left, right])

            if args.headless:
                now = time.time()
                if now - last_print > 0.25:
                    last_print = now
                    if geom is not None:
                        print(
                            f"[{frame_count:5d}] X={geom['x_in']:+6.2f}in "
                            f"Y={geom['y_in']:+6.2f}in depth={geom['depth_in']:5.2f}in "
                            f"r={detection.radius:4.1f}px conf={detection.confidence:0.2f} fps={fps:4.1f}"
                        )
                    else:
                        print(f"[{frame_count:5d}] no detection  fps={fps:4.1f}")
            else:
                cv2.imshow("aim_viewer", combined)
                key = cv2.waitKey(1) & 0xFF
                if key != 255:
                    should_quit, det_cfg, focal_px, action = handle_key(key, det_cfg, focal_px, defaults)
                    if should_quit:
                        break
                    if action == "save":
                        save_snapshot(combined, snapshot_count)
                        snapshot_count += 1
                    elif action:
                        print(f"-> {action}")

            if args.save_every_sec > 0:
                now = time.time()
                if now - last_auto_save >= args.save_every_sec:
                    save_snapshot(combined, snapshot_count)
                    snapshot_count += 1
                    last_auto_save = now

            frame_count += 1
            if args.num_frames and frame_count >= args.num_frames:
                break
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        close_camera(camera, camera_type)
        if not args.headless:
            cv2.destroyAllWindows()
        print(f"captured {frame_count} frames")


if __name__ == "__main__":
    main()
