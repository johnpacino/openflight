"""Build the aligned truth CSV for the 2026-07-17 TrackMan session.

TrackMan strokes are the spine (95 across two exports). Each OpenFlight
session file carries paired entries per shot (shared shot_number):
``iwr6843_capture`` (impact epoch, dump path, LCMF-v1 measurement) and
``shot_detected`` (OPS ball/club speed, spin). Records join on shot_number,
then match to TM strokes on impact time with an ESTIMATED clock offset
(mode of pairwise deltas). OF/TI shots without a TM stroke are skipped.

Usage:
  uv run python scripts/analysis/tm0717/build_aligned.py \
      --tm firmware/l3_dump/trackman_session_data_7_17_40shots.json \
      --tm firmware/l3_dump/trackman_session_data_7_17_55shots.json \
      --sessions artifacts/pi_sync_20260717/openflight_sessions \
      --out ~/openflight_sessions/tm_0717/session_aligned_2026-07-17.csv \
      --no-shanks-out ~/openflight_sessions/tm_0717/session_aligned_2026-07-17_no_shanks.csv
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

MPH = 2.23694
ET_UTC_OFFSET_S = -4 * 3600          # EDT
MATCH_HALF_WINDOW_S = 3.0
OF_TS_DELAY_S = 8.0                  # shot_detected fires after the TI dump
SHANK_RIGHT_DEG = 8.0
SHANK_FACE_OPEN_DEG = 8.0
SKULL_LA_DEG = 8.0


def classify_shot(m: dict) -> tuple[str, str]:
    """Flag obvious TrackMan shot-shape outliers before scoring TI angle."""
    la = m.get("LaunchAngle")
    launch_dir = m.get("LaunchDirection")
    face = m.get("FaceAngle")
    face_to_path = m.get("FaceToPath")
    if launch_dir is not None and launch_dir >= SHANK_RIGHT_DEG:
        return "shank", f"launch_direction>={SHANK_RIGHT_DEG:g}"
    if face is not None and face >= SHANK_FACE_OPEN_DEG:
        return "shank", f"face_angle>={SHANK_FACE_OPEN_DEG:g}"
    if face_to_path is not None and face_to_path >= SHANK_FACE_OPEN_DEG:
        return "shank", f"face_to_path>={SHANK_FACE_OPEN_DEG:g}"
    if la is not None and la < SKULL_LA_DEG:
        return "skull", f"launch_angle<{SKULL_LA_DEG:g}"
    return "good", ""


def tm_strokes(paths: list[Path]) -> list[dict]:
    out = []
    for p in paths:
        data = json.loads(p.read_text())
        for grp in data["StrokeGroups"]:
            club = grp.get("Club", "?")
            for s in grp["Strokes"]:
                m = s["Measurement"]
                shot_type, shot_type_reason = classify_shot(m)
                out.append(dict(
                    t=datetime.fromisoformat(s["Time"]).timestamp(),
                    club=club, file=p.name,
                    ball_mph=(m.get("BallSpeed") or 0) * MPH,
                    club_mph=(m.get("ClubSpeed") or 0) * MPH,
                    la=m.get("LaunchAngle"), dir=m.get("LaunchDirection"),
                    attack=m.get("AttackAngle"), spin=m.get("SpinRate"),
                    carry=(m.get("Carry") or 0) * 1.09361,
                    face=m.get("FaceAngle"),
                    club_path=m.get("ClubPath"),
                    face_to_path=m.get("FaceToPath"),
                    carry_side=(m.get("CarrySide") or 0) * 1.09361,
                    total_side=(m.get("TotalSide") or 0) * 1.09361,
                    shot_type=shot_type,
                    shot_type_reason=shot_type_reason))
    out.sort(key=lambda r: r["t"])
    return out


def session_shots(paths: list[Path]) -> tuple[list[dict], Counter]:
    """Join iwr6843_capture + shot_detected by (file, shot_number)."""
    errors = Counter()
    shots: dict[tuple[str, int], dict] = {}
    for p in paths:
        for line in p.read_text().splitlines():
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            typ = d.get("type")
            if typ == "error":
                errors[p.name] += 1
                continue
            if typ not in ("iwr6843_capture", "shot_detected"):
                continue
            key = (p.name, d.get("shot_number", -1))
            rec = shots.setdefault(key, dict(file=p.name,
                                             n=d.get("shot_number")))
            if typ == "shot_detected":
                rec.update(
                    of_ts=(datetime.fromisoformat(d["ts"])
                           .replace(tzinfo=timezone.utc).timestamp()
                           - ET_UTC_OFFSET_S),
                    of_ball=d.get("ball_speed_mph"),
                    of_club_speed=d.get("club_speed_mph"),
                    of_smash=d.get("smash_factor"),
                    of_carry=d.get("estimated_carry_yards"),
                    of_spin=d.get("spin_rpm"),
                    of_spin_conf=d.get("spin_confidence"),
                    of_club=d.get("club"))
            else:
                m = d.get("measurement") or {}
                rec.update(
                    impact_t=d.get("shot_timestamp"),
                    ti_file=d.get("capture_path"),
                    ti_error=d.get("capture_error"),
                    ti_status=m.get("status"),
                    ti_la=m.get("launch_angle_deg"),
                    ti_la_raw=m.get("raw_angle_deg"),
                    ti_corr=m.get("angle_correction_deg"),
                    ti_estimator=m.get("estimator"),
                    ti_track_mph=m.get("track_speed_mph"),
                    ti_quality=m.get("tracker_quality"),
                    ti_n_snaps=m.get("n_snapshots"),
                    ti_comp_std=m.get("component_std_deg"))
    out = []
    for rec in shots.values():
        t = rec.get("impact_t")
        if t is None and rec.get("of_ts") is not None:
            t = rec["of_ts"] - OF_TS_DELAY_S
        if t is None:
            continue
        rec["t"] = float(t)
        out.append(rec)
    out.sort(key=lambda r: r["t"])
    return out, errors


def estimate_offset(tm: list[dict], src: list[dict], span_s: float = 30.0
                    ) -> float:
    deltas = [s["t"] - r["t"] for s in src for r in tm
              if abs(s["t"] - r["t"]) <= span_s]
    if not deltas:
        return 0.0
    mode, _ = Counter(round(d * 2) / 2 for d in deltas).most_common(1)[0]
    tight = [d for d in deltas if abs(d - mode) <= 1.5]
    return sum(tight) / len(tight)


def match(tm: list[dict], src: list[dict], offset: float
          ) -> tuple[dict[int, int], list[str]]:
    flags = []
    cands = sorted((abs(s["t"] - offset - r["t"]), i, j)
                   for i, r in enumerate(tm) for j, s in enumerate(src)
                   if abs(s["t"] - offset - r["t"]) <= MATCH_HALF_WINDOW_S)
    used_i, used_j, out = set(), set(), {}
    for d, i, j in cands:
        if i in used_i or j in used_j:
            continue
        out[i] = j
        used_i.add(i)
        used_j.add(j)
    per_i = Counter(i for d, i, j in cands if d <= 1.5)
    for i, n in per_i.items():
        if n > 1:
            flags.append(f"TM stroke {i} ({tm[i]['club']}) had {n} "
                         f"candidates within 1.5s")
    return out, flags


def fmt_et(t: float) -> str:
    return datetime.fromtimestamp(t + ET_UTC_OFFSET_S,
                                  tz=timezone.utc).strftime("%H:%M:%S")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tm", action="append", type=Path, required=True)
    ap.add_argument("--sessions", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--no-shanks-out", type=Path)
    a = ap.parse_args()

    tm = tm_strokes(a.tm)
    shots, errors = session_shots(
        sorted(a.sessions.glob("session_20260717_*.jsonl")))
    print(f"TM {len(tm)} strokes | OF/TI joined shots {len(shots)}")
    off = estimate_offset(tm, shots)
    print(f"estimated Pi-vs-TM clock offset: {off:+.2f} s")
    mm, flags = match(tm, shots, off)

    rows = []
    for i, r in enumerate(tm):
        row = dict(time_et=fmt_et(r["t"]), club=r["club"],
                   tm_ball_mph=round(r["ball_mph"], 1),
                   tm_club_mph=round(r["club_mph"], 1),
                   tm_la_deg=r["la"], tm_dir_deg=r["dir"],
                   tm_attack_deg=r["attack"], tm_spin_rpm=r["spin"],
                   tm_carry_yds=round(r["carry"], 1), tm_file=r["file"],
                   tm_face_deg=r["face"],
                   tm_club_path_deg=r["club_path"],
                   tm_face_to_path_deg=r["face_to_path"],
                   tm_carry_side_yds=round(r["carry_side"], 1),
                   tm_total_side_yds=round(r["total_side"], 1),
                   shot_type=r["shot_type"],
                   shot_type_reason=r["shot_type_reason"])
        s = shots[mm[i]] if i in mm else {}

        def rd(v, nd=1):
            return None if v is None else round(v, nd)

        row.update(
            of_ball_mph=rd(s.get("of_ball")),
            of_club_speed_mph=rd(s.get("of_club_speed")),
            of_smash=rd(s.get("of_smash"), 2),
            of_spin_rpm=rd(s.get("of_spin"), 0),
            of_spin_conf=s.get("of_spin_conf"),
            of_carry_yds=rd(s.get("of_carry")),
            of_club=s.get("of_club"), of_file=s.get("file"),
            of_shot_n=s.get("n"),
            ti_la_deg=rd(s.get("ti_la"), 2),
            ti_la_raw_deg=rd(s.get("ti_la_raw"), 2),
            ti_angle_corr_deg=s.get("ti_corr"),
            ti_estimator=s.get("ti_estimator"),
            ti_status=s.get("ti_status"),
            ti_track_mph=rd(s.get("ti_track_mph")),
            ti_tracker_quality=s.get("ti_quality"),
            ti_n_snapshots=s.get("ti_n_snaps"),
            ti_component_std_deg=rd(s.get("ti_comp_std"), 2),
            ti_capture_error=s.get("ti_error"),
            ti_file=s.get("ti_file"),
            dt_s=(None if i not in mm
                  else round(s["t"] - off - r["t"], 2)))
        rows.append(row)

    a.out.parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {a.out} ({len(rows)} rows)")
    if a.no_shanks_out:
        no_shanks = [r for r in rows if r["shot_type"] != "shank"]
        a.no_shanks_out.parent.mkdir(parents=True, exist_ok=True)
        with open(a.no_shanks_out, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(no_shanks)
        print(f"wrote {a.no_shanks_out} ({len(no_shanks)} rows)")

    n_m = len(mm)
    print(f"matched {n_m}/{len(tm)} TM strokes; skipped "
          f"{len(shots)-n_m} OF/TI shots with no TM stroke")
    by_file = Counter(shots[j]["file"] for j in mm.values())
    print("matches by session file:")
    for k, v in sorted(by_file.items()):
        print(f"  {k}: {v}")
    unmatched = [f"{r['club']}@{fmt_et(r['t'])}"
                 for i, r in enumerate(tm) if i not in mm]
    if unmatched:
        print(f"FLAG: {len(unmatched)} TM strokes UNMATCHED: "
              + ", ".join(unmatched))
    for f in flags:
        print("FLAG:", f)
    if errors:
        print("session 'error' entry counts (context, not fatal):")
        for k, v in sorted(errors.items()):
            print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
