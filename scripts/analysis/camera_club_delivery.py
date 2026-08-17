#!/usr/bin/env python3
"""Camera club-delivery analysis for DTL (down-the-line) OV9281 captures.

Run:
    uv run --extra analysis --with opencv-python-headless \
        scripts/analysis/camera_club_delivery.py ~/openflight_sessions/session_X.jsonl \
        --club 9-iron

Geometry (validated 2026-08-07 on the 55-shot TrackMan-prep session):
  The camera is DOWN THE LINE: the club/ball travel mostly along the optical
  axis. Image x = lateral (in-to-out positive for a right-handed golfer viewed
  from behind), image y = -vertical. Downrange velocity is invisible; it is
  taken from OPS club speed.

  AoA  = atan(v_vertical / v_downrange)
  path = atan(v_lateral  / v_downrange)

Outputs per shot:
  - impact frame (halo-robust ball-core departure)
  - clubhead transverse velocity at impact (quadratic fit over the frames
    where the club line passes near the ball)
  - AoA / path estimates -- NOTE: with ~2 usable pre-impact frames at 288 fps
    the arc-curvature averaging biases both steep by roughly 3x. Treat the
    magnitudes as upper bounds until frame rate / ROI is increased.
  - delivery_direction_deg = atan2(v_vert, v_lat): the AoA:path ratio. This is
    curvature-resistant (both components inflate together) and matched the
    July TrackMan baseline atan2(-4.46, +3.98) = -48.3 deg with sd ~4 deg.

Every value is experimental; validate against TrackMan before promoting.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))

from openflight.camera.club_motion import detect_reference_ball  # noqa: E402

BRIGHT_NOW = 200
DARK_BG = 175
LINE_BALL_DIST = 45.0
MIN_COMP_AREA = 150
HEAD_DISC_RADIUS = 40.0
FRAME_OFFSETS = range(-4, 5)
MPH_TO_MMS = 447.04
BALL_CORE_DELTA = 30.0


def detect_impact_index(frames: np.ndarray, ball) -> int:
    """Last frame the teed ball's core pixels are undisturbed.

    Uses only the inner disk of the detected ball so the approaching club's
    glow (which contaminates a neighbourhood MAD test ~2 frames early) does
    not trigger a false departure.
    """
    r = max(3, int(round(ball.diameter_px * 0.4)))
    yy, xx = np.mgrid[0 : frames.shape[1], 0 : frames.shape[2]]
    disk = (xx - ball.x) ** 2 + (yy - ball.y) ** 2 <= r * r
    ref = float(np.median(frames[:15], axis=0)[disk].mean())
    means = np.array([float(f[disk].mean()) for f in frames])
    present = np.abs(means - ref) < BALL_CORE_DELTA
    idxs = np.nonzero(present)[0]
    for idx in reversed(idxs):
        after = present[idx + 1 : idx + 3]
        if len(after) == 0 or not after.any():
            if idx + 1 < len(frames):
                return int(idx)
    return int(idxs[-1])


def head_centroid(frame: np.ndarray, background: np.ndarray, ball):
    """Clubhead position: bright-shaft line end (hosel) anchored by the dark
    moving head blob around it.

    The moving-bright mask (saturated now, dark background) captures ONLY the
    chrome shaft -- the head is dark metal. The shaft's ball-side line end is
    the hosel, and the absdiff blob around it recovers the head mass. The club
    component is identified by its fitted line passing near the ball, which
    the golfer's body cannot fake.
    """
    mask = ((frame > BRIGHT_NOW) & (background < DARK_BG)).astype(np.uint8)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best = None
    for lab in range(1, n):
        if stats[lab, cv2.CC_STAT_AREA] < MIN_COMP_AREA:
            continue
        ys, xs = np.nonzero(labels == lab)
        pts = np.column_stack([xs, ys]).astype(np.float64)
        vx, vy, x0, y0 = cv2.fitLine(
            pts.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01
        ).ravel()
        axis = np.array([vx, vy], dtype=np.float64)
        origin = np.array([x0, y0], dtype=np.float64)
        rel_ball = np.array([ball.x, ball.y]) - origin
        dist_line = abs(rel_ball[0] * axis[1] - rel_ball[1] * axis[0])
        if dist_line > LINE_BALL_DIST:
            continue
        d_cent = math.hypot(cents[lab][0] - ball.x, cents[lab][1] - ball.y)
        if best is None or d_cent < best[0]:
            best = (d_cent, pts, axis, origin)
    if best is None:
        return None
    _, pts, axis, origin = best
    rel = pts - origin
    along = rel @ axis
    ball_along = float((np.array([ball.x, ball.y]) - origin) @ axis)
    if ball_along >= float(np.median(along)):
        band = along >= along.max() - 6.0
    else:
        band = along <= along.min() + 6.0
    ex, ey = pts[band].mean(axis=0)
    h, w = frame.shape
    yy, xx = np.mgrid[0:h, 0:w]
    disc = (xx - ex) ** 2 + (yy - ey) ** 2 <= HEAD_DISC_RADIUS**2
    mov = cv2.absdiff(frame, background) > 25
    blob = disc & mov
    if int(blob.sum()) >= 60:
        bys, bxs = np.nonzero(blob)
        return float(bxs.mean()), float(bys.mean())
    return float(ex), float(ey)


def analyze_shot(shot: dict, capture_dir: Path, out_dir: Path) -> dict | None:
    archive = np.load(capture_dir / "frames.npz")
    frames = archive["frames"]
    ts = archive["host_timestamp_ns"].astype(np.int64)
    ball = detect_reference_ball(frames)
    impact_idx = detect_impact_index(frames, ball)
    background = np.median(frames[:15], axis=0).astype(np.uint8)

    pts = []
    for off in FRAME_OFFSETS:
        idx = impact_idx + off
        if not 0 <= idx < len(frames):
            continue
        head = head_centroid(frames[idx], background, ball)
        if head is not None:
            pts.append((idx, head[0], head[1]))
    n_pre = sum(1 for p in pts if p[0] <= impact_idx)
    if len(pts) < 4 or n_pre < 2:
        return {
            "shot": shot["shot_number"],
            "impact_idx": impact_idx,
            "status": f"track_too_short ({len(pts)} pts, {n_pre} pre)",
        }

    idxs = np.array([p[0] for p in pts])
    t_impact = (ts[impact_idx] + ts[min(impact_idx + 1, len(ts) - 1)]) / 2
    t = (ts[idxs].astype(np.float64) - t_impact) / 1e9
    x = np.array([p[1] for p in pts])
    y = np.array([p[2] for p in pts])
    mm_per_px = 42.67 / ball.diameter_px

    deg = 2 if len(t) >= 5 else 1
    cx = np.polyfit(t, x, deg)
    cy = np.polyfit(t, y, deg)
    res = np.sqrt((np.polyval(cx, t) - x) ** 2 + (np.polyval(cy, t) - y) ** 2)
    vx_px = float(np.polyval(np.polyder(cx), 0.0))
    vy_px = float(np.polyval(np.polyder(cy), 0.0))
    v_lat = vx_px * mm_per_px / 1000.0
    v_vert = -vy_px * mm_per_px / 1000.0
    v_down = shot["club_speed_mph"] * MPH_TO_MMS / 1000.0
    v_forward = math.sqrt(max(v_down**2 - v_lat**2 - v_vert**2, 1.0))

    overlay = cv2.cvtColor(frames[impact_idx], cv2.COLOR_GRAY2BGR)
    for fx, fy in zip(x, y):
        cv2.circle(overlay, (round(fx), round(fy)), 4, (0, 0, 255), 1)
    cv2.circle(overlay, (round(ball.x), round(ball.y)), 7, (0, 255, 0), 1)
    cv2.imwrite(str(out_dir / f"shot_{shot['shot_number']:02d}_delivery.png"), overlay)

    return {
        "shot": shot["shot_number"],
        "impact_idx": impact_idx,
        "status": "ok",
        "n_pts": len(pts),
        "n_pre": n_pre,
        "v_lateral_ms": round(v_lat, 2),
        "v_vertical_ms": round(v_vert, 2),
        "delivery_direction_deg": round(math.degrees(math.atan2(v_vert, v_lat)), 2),
        "aoa_deg_biased": round(math.degrees(math.atan2(v_vert, v_forward)), 2),
        "path_deg_biased": round(math.degrees(math.atan2(v_lat, v_forward)), 2),
        "ops_club_mph": shot["club_speed_mph"],
        "fit_resid_px": round(float(np.sqrt(np.mean(res**2))), 2),
        "mm_per_px": round(mm_per_px, 3),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="session_*.jsonl with camera_capture entries")
    parser.add_argument("--club", help="only analyze shots with this club label")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--path-map",
        nargs=2,
        metavar=("FROM", "TO"),
        default=("/home/pacinoj", str(Path.home())),
        help="rewrite capture paths recorded on the Pi to local paths",
    )
    args = parser.parse_args()

    shots, cams = {}, {}
    for line in args.session.expanduser().open():
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            continue
        if entry.get("type") == "shot_detected":
            shots[entry["shot_number"]] = entry
        elif entry.get("type") == "camera_capture":
            cams[entry["shot_number"]] = entry

    out_dir = args.output_dir or args.session.expanduser().parent / (
        args.session.stem + "_club_delivery"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for num in sorted(shots):
        shot = shots[num]
        if args.club and shot.get("club") != args.club:
            continue
        cam = cams.get(num)
        if cam is None or cam.get("capture_error"):
            rows.append({"shot": num, "status": "no_capture"})
            continue
        capture_dir = Path(cam["capture_path"].replace(*args.path_map))
        try:
            result = analyze_shot(shot, capture_dir, out_dir)
        except Exception as exc:  # noqa: BLE001 - keep sweeping past bad captures
            result = {"shot": num, "status": f"error: {exc}"}
        if result:
            rows.append(result)
            print(json.dumps(result))

    ok = [r for r in rows if r.get("status") == "ok"]
    if ok:
        dirs = np.array([r["delivery_direction_deg"] for r in ok])
        print(
            f"\ndelivery direction atan2(AoA, path): mean {dirs.mean():.1f} "
            f"sd {dirs.std():.1f} n={len(dirs)} "
            f"(July TrackMan 9i baseline: -48.3)"
        )
    (out_dir / "results.json").write_text(json.dumps(rows, indent=2) + "\n")
    print(f"-> {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
