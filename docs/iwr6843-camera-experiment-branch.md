# IWR6843 + Camera Experiment Branch

This branch is exploring the next step after the IWR6843 proved it can produce useful launch-angle data: adding a high-speed camera capture path so we can line up OPS speed, TI radar frames, and camera frames around the same impact event.

The short version: OPS is still the speed and trigger backbone, the TI IWR6843 board is still the radar angle source, and the OV9281 camera is being added as a high-speed visual witness around impact. The goal is not to replace the radar. The goal is to give us another synchronized view of the club and ball so we can improve club path, attack angle, timing diagnostics, and offline validation.

## Current Branch State

This branch currently includes:

- A new high-speed camera capture runtime for the Raspberry Pi OV9281 camera.
- A GPIO-triggered camera rolling buffer that keeps frames before and after the sound trigger.
- Session logging for camera captures so each shot can reference the saved camera folder and timing metadata.
- CLI/server flags under `--camera-capture` so camera capture can run alongside OPS and IWR6843 without replacing the existing camera UI path.
- Hardware-test scripts for camera clap testing, camera alignment preview, and exposure calibration.
- Early offline camera club-motion analysis code for finding the ball, shaft/club motion, and image-plane movement around impact.

The branch is intentionally still experimental. The camera output is primarily for offline analysis right now. We are not yet claiming production-quality camera-derived club path or attack angle.

## Hardware Roles

### OPS243

OPS remains the trusted speed and trigger device.

It provides:

- Ball speed.
- Club speed candidate data.
- Hardware sound-trigger timing.
- The main shot event used by OpenFlight.

OPS speed has been close enough to TrackMan in prior sessions that we want to keep using it as the anchor for ball speed and as one input into club-search logic.

### TI IWR6843

The TI board provides the short-range radar movie.

It is used for:

- Vertical launch angle.
- Horizontal launch direction.
- Experimental club path and attack-angle radar features.
- Raw L3 dump capture for offline replay.

The current firmware/config direction is built around dense, short-range frame capture near impact. The idea is to spend the limited on-chip L3 RAM on the part of the swing and early ball flight that matters most, rather than recording a large generic radar cube.

### OV9281 Camera

The OV9281 camera is being added as a synchronized high-speed visual channel.

It is used for:

- Capturing a visual movie around impact.
- Checking whether radar impact timing is correct.
- Seeing whether the club/shaft/ball are visible in the expected frames.
- Providing a visual reference for future club path and attack-angle estimation.
- Helping debug radar misses by showing whether the shot was clean, late, early, hosel, skull, etc.

The current capture mode is monochrome raw frames at `640x400`, requested at `300 fps`. In practice we have been seeing about `288 fps` with stable frame intervals and no dropped frames.

## What We Have Built So Far

### Camera Capture Runtime

The new runtime keeps a rolling pre-trigger camera buffer and then saves a short post-trigger tail when the sound trigger fires.

The current target capture is:

- About `150 ms` before the trigger.
- About `50 ms` after the trigger.
- `60` total frames at the current measured frame rate.
- Saved as compact frame data plus preview images and metadata.

This mirrors the same general idea that made OPS and IWR6843 useful: do not start recording after impact. Keep a rolling buffer, then freeze the right window.

### Shared Trigger Strategy

The camera capture is tied to the same sound-trigger event used by the radar stack.

The intended flow is:

1. Sound trigger fires on GPIO17.
2. OPS processes the shot and provides speed/timing.
3. IWR6843 freezes and dumps its radar buffer.
4. Camera freezes and saves its visual buffer.
5. Session logging ties the artifacts together by shot.

This gives us a single shot record that can point to:

- OPS speed/capture data.
- IWR6843 L3 dump.
- Camera frame folder.
- Timing metadata for alignment.

### Exposure Calibration

We added a camera exposure calibration script because outdoor lighting made the first camera frames either too dark or heavily clipped.

Recent outdoor testing compared multiple exposure/gain settings. The best current setting is:

```bash
--camera-capture-exposure-us 500
--camera-capture-gain 2
```

The latest test at this setting produced:

- `60` frames.
- About `287.9 fps`.
- `0` camera frame gaps.
- Mean brightness around `91`.
- p99 brightness around `227`.
- Clipping around `0.75%`.

That was cleaner than `600us / gain 2`, which was brighter but clipped more highlights around the club/ball area.

## Latest Test Goal

The latest test was not about launch-angle MAE. It was specifically about finding a camera exposure that preserves usable detail around impact.

We wanted to answer:

- Can the camera hold frame cadence while OpenFlight is running?
- Do we get the expected pre-impact and post-impact frames?
- Is the club visible through impact?
- Is the ball visible after launch?
- Are highlights clipped so badly that club/ball detail is lost?

The answer so far is encouraging:

- The camera cadence held.
- The impact window was captured.
- The club was visible through impact.
- `500us / gain 2` appears to be the current best outdoor setting.

## What We Are Working On Now

The active work is shifting from "can we capture synchronized camera frames?" to "can those frames help us solve club delivery?"

The immediate focus areas are:

- Better impact-frame identification.
- Club/shaft/clubhead detection in the frames immediately before impact.
- Comparing camera-visible club motion with IWR6843 radar-visible club motion.
- Using OPS club speed to constrain which radar/camera target is likely the clubhead.
- Determining whether camera data can stabilize attack angle and club path estimates.

We are deliberately keeping raw camera frames and radar dumps so we can replay sessions offline as the estimators improve.

## What We Are Trying To Achieve

The larger product goal is a low-cost launch monitor that can approach TrackMan-like ball and club metrics without pretending one sensor can do everything.

The intended division of labor is:

- OPS handles robust speed measurement.
- IWR6843 handles radar-based launch direction and short-window RF tracking.
- Camera handles visual timing, club/ball context, and potentially club delivery refinement.

For ball data, the TI board has already shown strong promise for vertical launch and horizontal direction when the timing/configuration is right.

For club data, the hard problem is that club path and attack angle are defined at impact. That is a very small time window. The camera helps because it can show what the club and ball actually did in the frames around impact, while the radar helps because it can measure motion and range/angle in ways a single down-the-line camera cannot.

## Planned Camera + TI Fusion Approach

The current plan is to keep the fusion simple and evidence-driven:

1. Capture OPS, IWR6843, and camera data from the same sound trigger.
2. Use OPS speed and timing as the shot anchor.
3. Use IWR6843 for vertical and horizontal launch angle when radar quality gates pass.
4. Use camera frames to verify impact timing and visible club/ball motion.
5. Use camera-derived impact context to improve offline selection of the correct IWR6843 club target.
6. Compare camera image-plane club motion against IWR6843 radar club candidates.
7. Only promote camera-derived club path or attack angle into the UI after we can validate it against TrackMan.

The important principle is that the camera should not become another "magic offset" source. It should either help select the correct physical target, improve timing, or provide a measurable visual cue that can be validated.

## Why This Matters

The TI radar gave us a much better short-window radar movie than the older KLD7 approach. The remaining challenge is club delivery.

Club path and attack angle are difficult because:

- They happen at the exact impact window.
- The club may decelerate or change return strength at impact.
- Shaft, head, ball, mat, and multipath can all compete in the radar return.
- A single camera view has perspective limits.
- A single radar view has manifold and target-selection limits.

The combined camera + TI approach gives us two independent views of the same event. That should let us debug which target the radar is selecting and whether the selected motion is physically plausible.

## Open Questions

The main unresolved questions are:

- Can camera timing reliably identify the exact impact frame?
- Can the camera find the clubhead, not just the shaft?
- Can OPS club speed constrain IWR6843 club target selection enough to improve club path and attack angle?
- Does the camera need a calibrated physical pose before it can contribute useful angle estimates?
- Is one down-the-line camera enough, or is it primarily a timing/debug aid?
- How much of the final club delivery estimate should come from camera data versus radar data?

## Near-Term Next Steps

Recommended next steps:

1. Keep testing with `500us / gain 2` outdoors.
2. Capture more real swings with OPS, IWR6843, and camera enabled together.
3. Build a lightweight impact-frame viewer for fast review after each session.
4. Run offline camera club-motion analysis on clean shots first.
5. Use the camera frames to label clean shots, mishits, and timing failures.
6. Compare camera-visible club motion with IWR6843 club path/AoA candidates.
7. Validate any promising estimator against TrackMan before showing it as anything more than experimental.

## Findings From The 2026-08-07 55-Shot Session (9-iron offline analysis)

Offline analysis of the 18 9-iron shots (all 55 shots had clean synchronized
captures; camera 55/55 zero frame gaps) established:

- **The camera view is down-the-line, not face-on.** The club and ball travel
  mostly along the optical axis. Image x measures lateral (in-to-out) motion,
  image y measures vertical motion, and downrange speed is invisible. Any
  image-plane angle is therefore a mix of AoA and club path, never AoA alone.
  The correct decomposition is `AoA = atan(v_vert / v_downrange)` and
  `path = atan(v_lat / v_downrange)` with `v_downrange` taken from OPS club
  speed. This makes a DTL camera a *club path* sensor as much as an AoA sensor.
- **Impact frame is recoverable to ±1 frame** by watching the teed ball's core
  pixels depart (use only the inner disk — the approaching club's glow trips a
  neighborhood-MAD test ~2 frames early). Impact lands ~1–2 frames before the
  GPIO trigger timestamp.
- **The chrome shaft is the only bright mover**; the head is dark metal. The
  robust head locator is: moving-bright mask (>200 now, <175 in background) →
  component whose fitted line passes near the ball (the golfer's body can't
  fake that) → ball-side line end (hosel) → anchored by the absdiff head blob.
  Template/endpoint trackers fail on the aperture problem (they slide along
  the featureless shaft line and report motion perpendicular to it).
- **The delivery-direction ratio is the camera's robust output today.**
  `atan2(v_vert, v_lat) = atan2(tan AoA, tan path)` came out −48.4° ± 3.7°
  across the 18 shots, matching the July TrackMan 9i baseline
  `atan2(−4.46, +3.98) = −48.3°`. Absolute AoA/path magnitudes are biased ~3×
  steep because only ~2 pre-impact frames contain the club at 288 fps and the
  swing arc turns ~0.9°/ms, so the fit averages over curvature.
- **Radar club-path candidates failed 55/55** this session (no
  `candidate_available`), and radar AoA candidates ran ~15° steeper than both
  the camera ratio and TrackMan plausibility — the camera is already useful as
  a plausibility gate on radar target selection.

Why the ratio is robust: the shaft lies in the delivery plane, so all
in-plane motion — head translation and shaft rotation alike — projects along
the plane's image trace. The camera is measuring the delivery-plane
orientation. The same fact poisons magnitude measurements: the shaft sweeps
at ~30 rad/s, so any tracker with shaft pixels in it reads 15–25 m/s of
rotational sweep instead of the ~4 m/s true head transverse velocity at
impact. Low-point extrapolation on those magnitudes gives ~6× steep angles.

Constraints checked on the Pi (2026-08-07): the OV9281 driver's fastest mode
is 640×400 @ 309.8 fps, and that mode is a full-sensor 2×2 bin — there is no
cropped high-fps mode without driver patches. Motion-blur velocimetry is also
out: in DTL geometry only transverse motion blurs, ~0.6 px at 500 µs.

### Camera-assisted radar replay (the actual point of the camera)

Replaying the 18 9-iron L3 dumps through `estimate_club_path` with swept
impact times confirmed the camera's role and produced three results:

1. **The radar's impact anchor is ~2 ms late.** `track_impact_error_m`
   (club-track range vs tee at assumed impact) minimizes at −2 ms, exactly
   the direction and scale the camera measured. A −2 ms correction to
   `impact_t_s` is justified independently by the radar's own geometry.
2. **At −2 ms, radar and camera agree on the delivery plane.** The radar
   candidate (path, AoA) pairs land on the camera's plane trace with median
   error −0.5° (vs +17° at uncorrected timing). Two independent sensors,
   one plane.
3. **The camera plane-trace gates radar candidates effectively.** Candidates
   passing the ±15° trace gate: path sd 9.6°, all in-to-out (matching the
   golfer's TrackMan tendency). Failing candidates: sd 24.9° including
   sign-flipped paths. This is the "help select the correct physical target"
   role working as designed.

**The one remaining blocker for absolute club path / AoA is arc curvature**,
and it is common to both sensors: the radar's constant-velocity candidate
fit over 4 pre-impact frames (~8 ms) averages the club arc exactly like the
camera's chord did (~5× steep: path ~+20° vs TrackMan +4°, AoA ~−20° vs
−4.5°). Shrinking the window fails — `CLUB_MIN_FRAMES = 4` kills the path
candidate and AoA noise explodes (sd 12–25°).

Quadratic (curvature-aware) fits were tested and rejected: medians move
toward truth but per-shot variance explodes (AoA sd 3.8° → 28.7°). More
importantly, arc curvature predicts only ~3.6° of chord bias while ~17° is
observed — the dominant systematic in BOTH sensors is **scattering-center /
glint migration along the rotating club**: the radar's bright spot slides
along the shaft/head just like the camera's shaft glint. Because that
migration is along the club (in the delivery plane), the plane-trace ratio
is immune while all magnitude measurements inflate.

### Working fusion chain (awaiting TrackMan validation)

The radar's linear AoA candidate at the −2 ms-corrected impact time is
*tightly* wrong (sd 3.8° about a biased median) — a stable systematic, and
stable systematics are calibratable (precedent: KLD7 boresight offset,
la_position tiers). The chain:

1. OPS: ball and club speed (unchanged).
2. IWR at −2 ms-corrected impact: linear AoA candidate **+16.0°** offset
   (one calibration constant = TrackMan baseline minus radar median; likely
   club- and setup-dependent — must be validated per club at the next
   TrackMan session).
3. Camera delivery-plane trace converts corrected AoA into club path:
   `tan(path) = tan(AoA) / tan(trace)`.

Result on the 18 9-iron shots: AoA −4.5° sd 3.9 (TrackMan −4.46 ± 1.54),
path **+4.1° sd 3.7 (TrackMan +3.98 ± 1.86)** — the path median emerges
independently from the camera ratio rather than being fit. Estimated
per-shot estimator noise after subtracting real swing variation: ~3.2–3.6°
RMS. Not yet TrackMan-grade, but a working, physically-grounded estimate
with a single validated-offset dependency.

The offset's root cause was isolated with two experiments on the same
captured points. Anchor mismatch is refuted: the club points pass within
1 mm of the tee anchor, and removing the anchor makes the fit far worse
(−31.9° median, sd 12.5). The points themselves descend ~5× too fast
(~11 cm over the final 0.31 m vs the physical ~2.4 cm). Running the
TrackMan-validated two-ray solver on the club snapshots recovers only
~4.4° of the ~16° bias (−20.4° → −16.0°, with slightly worse sd), so
ground multipath is a minor contributor — the dominant term is **vertical
glint migration**: the radar's scattering center genuinely slides down the
club (shaft → hosel/head, ~7–9 cm) during the approach, real motion of the
reflection point that no propagation model can remove. Conclusion: keep
`fit_tee` plus a calibrated offset; the offset is set by club + mount
geometry, so it should transfer across sessions and differ per club class
— both directly testable at the next TrackMan session.

Cross-club replay (7i, 5i, driver) confirmed the glint theory's
predictions: radar AoA stays tight within each club (sd 3.9–5.6°) but the
offset is club-dependent — 9i +16.0°, 7i +10.5°, 5i +9.1° (monotone with
loft), driver +21.6° (quality-suspect) — so the calibration is a per-club
offset table, TrackMan-arbitrated. The camera trace re-validated on the
7-iron (clean shots ≈ −55° vs the TrackMan 7i ratio −59.4°).

Two operational lessons from the same sweep. First, the fixed 500 µs
exposure calibrated at 17:00 was badly underexposed by 17:35: the club
chrome stopped saturating and the club mask locked onto the flying ball
instead (producing ball-launch-direction traces — an accident worth
keeping as a launch cross-check). The capture runtime needs periodic
auto-exposure recalibration and per-capture scene-brightness metadata.
Second, the driver is unsupported end-to-end: all six driver shots failed
radar quality gates (`impact_contact_mismatch` / `club_speed_mismatch`),
the path candidate was null on five of six, and the camera was dark —
driver needs its own capture-config investigation before any calibration
applies.

Known gaps: the per-club offsets need TrackMan arbitration before any UI
exposure; hardware fallbacks remain (wider camera FOV / pull back — the
OV9281 driver tops out at 640×400 @ 309.8 fps, no crop modes — or a second
face-on camera).

Offline tool: `scripts/analysis/camera_club_delivery.py` runs this analysis
per session/club and writes per-shot overlays plus `results.json`.

## Current Caution

This branch should be treated as an instrumentation and research branch, not a finished feature branch.

The useful production pieces are likely:

- The camera capture runtime.
- The session logging hooks.
- The exposure/alignment tools.
- The synchronized artifact layout.

The club path and attack-angle estimators are still experimental and should stay labeled that way until validated against TrackMan.
