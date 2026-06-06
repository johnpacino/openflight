# K-LD7 Timing Drift Debug Notes

## Problem

TrackMan comparison sessions showed that K-LD7 vertical geometry often has the right signal, but the selected frames sometimes need a per-shot timing shift to line up with the known ball start point and TrackMan launch angle. The issue is not a simple constant offset.

This matters because K-LD7 geometry uses frame time relative to impact. A 10-20 ms timing shift can materially change the calculated vertical launch angle.

## Current Evidence

### TrackMan Sessions

In John's TrackMan 7-iron session, the good radar-selected shots were strong:

- After removing shots with vertical launch error greater than 5 degrees, the remaining 11 shots had MAE around `1.18 deg`.
- That showed the K-LD7 vertical radar and geometry model can be accurate when the right frames and timing alignment are used.
- The outlier shots were not random. Several had apparently good frame evidence, but a modest timing shift brought the geometry much closer to TrackMan.
- For multiple two-frame shots, shifting both frames together by roughly `10-40 ms` could reduce error substantially, which points to a shot-level timing alignment issue rather than bad per-frame spacing.

The practical conclusion from that session was:

```text
The geometry is often there, but the impact-time anchor can be wrong for that shot.
```

This is why the problem cannot be solved by only loosening SNR/bin filters. Frame selection matters, but timing alignment is a separate axis.

### Coleman Session

Coleman's `session_20260605_132943_trackman.jsonl` showed the same class of problem on another setup/person.

Running the frame report showed that the frame nearest the shot anchor walked over the session:

- Early shots had the shot anchor around frame `151`.
- By shot 64, the shot anchor was around frame `159`.
- The frame closest to `+50 ms` after impact walked similarly.

That is about an 8-frame movement. At roughly `28.9 ms/frame`, this is a large enough shift to materially affect geometry launch.

Coleman's session also differed from John's normal frame position:

- John's sessions often placed the impact anchor around frame `38-40`.
- Coleman's session placed it around frame `150+`.
- A debug endpoint later confirmed these can both be valid positions inside the same ~6 second K-LD7 rolling buffer depending on where the impact timestamp lands relative to the newest K-LD7 frame.

So `frame 38` versus `frame 158` alone is not proof of dropped frames. It means the shot timestamp is being anchored at a different position in the K-LD7 buffer.

### Idle K-LD7 Buffer Check

The debug endpoint showed that the idle K-LD7 buffer cadence is stable:

- A synthetic `anchor_age_s=4.8` stayed near frames `36/38` over a long idle poll.
- Median K-LD7 frame spacing stayed around `28.9 ms`.
- This argues against idle K-LD7 frame-rate collapse or accumulating dropped frames as the primary cause.

This was useful, but it did not prove the real shot path is stable. It only showed that the K-LD7 stream is not drifting while we invent a fixed anchor age.

### 2026-06-06 Clap Simulation

The 2026-06-06 session was simulated by clapping in front of the radar, so it should not be used as golf-ball truth. Its selected frames, launch values, bin errors, and F1B range values are not meaningful as ball-flight evidence.

It is still useful as timing-chain instrumentation because each clap produced a sound-triggered OPS capture and K-LD7 snapshot.

The per-shot session log now records both sides of the timing chain:

- `shot_detected.kld7_timing_debug.impact_timestamp_kld7`
- `shot_detected.kld7_timing_debug.vertical.snapshot_host_time`
- `shot_detected.kld7_timing_debug.vertical.snapshot_delay_ms`
- `shot_detected.kld7_timing_debug.vertical.anchor_frame_index`
- `shot_detected.kld7_timing_debug.vertical.plus_50ms_frame_index`

From `session_20260606_081638_range.jsonl`:

| Shot | Impact Timestamp | Vertical Snapshot Time | Snapshot Delay | Anchor Frame |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 1780748383.0358565 | 1780748387.8467698 | 4810.91 ms | 35 |
| 8 | 1780748730.8608565 | 1780748735.6161058 | 4755.25 ms | 39 |
| 14 | 1780749076.6308565 | 1780749081.3654137 | 4734.56 ms | 39 |
| 15 | 1780749714.0848565 | 1780749718.7703340 | 4685.48 ms | 40 |
| 16 | 1780749731.6438565 | 1780749736.3246598 | 4680.80 ms | 41 |

Across shots 1-16:

- Impact elapsed: `1348.608 s`
- Snapshot elapsed: `1348.478 s`
- Snapshot delay changed by `-130.11 ms`
- Anchor frame moved from `35` to `41`

This is the first direct evidence showing which relationship is moving:

```text
vertical.snapshot_host_time - impact_timestamp_kld7
```

changed over the session. In other words, both timestamps move forward, but not at exactly the same rate in this run.

## Interpretation

This does not look like K-LD7 frame spacing breaking. The frame spacing is steady in idle polling and in logged frame cadence.

The strongest current signal is that:

```text
K-LD7 snapshot host time - impact timestamp
```

is changing over the session.

That implies relative drift between the timestamp used as impact and the host-time timestamps on K-LD7 frames, or a changing delay in when the K-LD7 snapshot is taken after each shot.

The TrackMan sessions establish the real golf-ball symptom: geometry can be recovered with per-shot timing shifts. Coleman's session establishes the issue is reproducible outside John's setup. The 2026-06-06 clap session adds instrumentation evidence that the moving quantity is the relationship between the K-LD7 snapshot host time and the OPS-derived impact timestamp.

## Current Hypotheses

1. OPS clock sync drift:
   The impact timestamp is derived from OPS `trigger_time + clock_sync_offset_s`. If the OPS clock mapping drifts relative to host time after startup, the K-LD7 anchor will walk even though the K-LD7 stream itself is stable.

2. Snapshot timing drift:
   The server may be snapshotting the K-LD7 buffer slightly earlier relative to impact over time. The logged `snapshot_delay_ms` directly measures this.

3. Log/live timestamp mismatch:
   Older offline analysis could use `kld7_buffer.shot_timestamp` while live extraction uses `impact_timestamp_kld7`. New logs now include both the shot-level timing chain and buffer-level timing debug to avoid guessing.

4. Snapshot timing path:
   If K-LD7 snapshots are taken after variable OPS processing or after any wait/reset behavior that changes over a run, the buffer position relative to impact can move even if both radars are streaming normally.

## What To Check Next

For future simulated shots or TrackMan sessions, compare per shot:

- `trigger_timestamp_source`
- `impact_timestamp_source`
- `ops_clock_sync_age_s`
- `clock_sync_offset_s`
- `impact_timestamp_kld7`
- `vertical.snapshot_host_time`
- `vertical.snapshot_delay_ms`
- `vertical.anchor_frame_index`
- `vertical.plus_50ms_frame_index`

If `snapshot_delay_ms` keeps decreasing as `ops_clock_sync_age_s` increases, the OPS clock sync is a prime suspect.

If `snapshot_delay_ms` changes while `impact_timestamp_kld7` and `snapshot_host_time` deltas disagree shot-to-shot, inspect the shot processing path and K-LD7 snapshot timing.

## Practical Debug Command

Pull the latest session and print the timing chain:

```bash
latest=$(ssh pi-host 'ls -t ~/openflight_sessions/session_*.jsonl | head -1')
scp "pi-host:$latest" /tmp/openflight_latest.jsonl
uv run python - <<'PY'
import json
from pathlib import Path

for line in Path("/tmp/openflight_latest.jsonl").read_text().splitlines():
    entry = json.loads(line)
    if entry.get("type") != "shot_detected":
        continue
    timing = entry.get("kld7_timing_debug") or {}
    vertical = timing.get("vertical") or {}
    print({
        "shot": entry.get("shot_number"),
        "ball": round(entry.get("ball_speed_mph") or 0, 1),
        "impact": timing.get("impact_timestamp_kld7"),
        "snapshot": vertical.get("snapshot_host_time"),
        "delay_ms": vertical.get("snapshot_delay_ms"),
        "anchor": vertical.get("anchor_frame_index"),
        "plus50": vertical.get("plus_50ms_frame_index"),
        "sync_age_s": timing.get("ops_clock_sync_age_s"),
        "trigger_src": timing.get("trigger_timestamp_source"),
        "impact_src": timing.get("impact_timestamp_source"),
    })
PY
```

Replace `pi-host` with the appropriate SSH host for the Pi.
