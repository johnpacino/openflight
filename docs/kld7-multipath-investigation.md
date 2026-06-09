# K-LD7 Multipath Investigation

This note summarizes the current working theory for why the vertical K-LD7
launch-angle radar can produce high-confidence but wrong launch angles indoors.
It is written for developers who need enough RF context to understand the
problem without needing a radar background.

## Short Version

The K-LD7 often sees the golf ball, but indoors it may also see floor, screen,
club, and mat reflections in the same Doppler neighborhood. When those returns
overlap, the measured phase angle is not "ball angle plus noise." It is a vector
blend of multiple RF returns.

That means a frame can have:

- good SNR
- low OPS speed-bin error
- a plausible timestamp
- and still have the wrong bearing

The geometry model is probably good enough when the selected frames are truly
the ball. The harder problem is candidate identity: deciding which radar return
is the ball and which is reflection or blended multipath.

## Evidence From The 8-Iron TrackMan Session

The June 8, 2026 8-iron TrackMan session was useful because:

- the screen was 12 ft from the ball
- the timing issue was mostly resolved
- TrackMan ball speed was roughly 2.5 mph higher than OPS
- the session included raw K-LD7 RADC frames for replay

Analysis highlights:

- Anchoring candidate bins to TrackMan speed instead of OPS speed did not fix
  the launch angle errors.
- Widening the scan from +/-25 to +/-50 bins found more possible candidates,
  but live-style rules still could not reliably choose the right one.
- Oracle selection using TrackMan truth could find better candidate pairs:
  `OPS +/-50 bins, SNR >= 1.5` reached about 2.46 deg MAE.
- The same data with live-style selection remained around 9 deg MAE.
- Band-weighted averaging around the speed bin only helped slightly:
  best tested case was about 8.42 deg MAE.
- Applying a 12 ft screen cap was physically sensible, but did not solve the
  issue. Most selected frames were already before the estimated screen arrival.

Interpretation: useful ball information exists in the raw data, but simple
rules based on SNR, speed-bin error, and timing are not enough to select it
reliably.

## Datasheet Cross-Check

The K-LD7 datasheet reinforces the multipath theory and adds a few concrete
configuration risks.

### Five-Meter Range Boundary

The datasheet warns that strong targets outside the configured distance or
speed range can create false reflections or wrong outputs. That matters for the
TrackMan bay:

```text
radar-to-ball: 5 ft
ball-to-screen: 12 ft
radar-to-screen: 17 ft = 5.18 m
```

With the K-LD7 configured for a 5 m range, the screen is just outside the
configured maximum range. The 5 m setting has the best range resolution, but it
may be the wrong setting when the screen/net is farther than 5 m from the radar.

Practical test:

- use 5 m range for home setups where the screen is inside 5 m from radar
- test 10 m range when the screen is about 12 ft from the ball and the radar is
  5 ft behind the ball
- compare candidate stability, F1B range progression, and two-frame ambiguity
  between 5 m and 10 m range

The tradeoff is range resolution: 5 m gives about 5 cm resolution; 10 m gives
about 10 cm resolution. The speed-frame duration is controlled by speed range,
so moving from 5 m to 10 m should not by itself cost the 29 ms frame cadence.

If a session is captured at 10 m range, the offline F1B/range analysis must also
be told that the radar was in 10 m mode. Otherwise range unwrapping and
range-consistency checks will be using the wrong ambiguity interval.

### Raw Targets Are Point Clouds

The datasheet says a real object does not necessarily produce one raw target.
It can create a point cloud with different speeds and distances, and the
environment can add reflections. This maps directly to what we see indoors: the
ball, floor, screen, club, and mat can produce multiple candidates in nearby
Doppler bins.

That is why the software should not treat a single high-SNR peak as proof that
the selected target is the ball.

### Speed Is Radial To The Sensor

The datasheet notes that measured speed is only directly correct when the target
motion is radial to the sensor. Tangential movement needs angle compensation.

For OpenFlight, this means OPS/TrackMan speed is still a valuable anchor, but
`bin_error == 0` is not guaranteed to be the only valid ball return. Launch
angle, radar tilt, lateral push/curve, and radar-to-ball geometry can all move
the K-LD7 radial speed slightly away from the full ball speed. Candidate scoring
should treat bin error as a strong feature, not an absolute identity check.

### Internal Threshold And Tracking Are Diagnostics

Current OpenFlight K-LD7 setup uses:

```text
range = 5 m
speed = 100 km/h
THOF = 10
TRFT = 1
VISU = 0
production stream = RADC only
```

The datasheet describes `THOF=10` as the more sensitive end of the internal raw
target threshold behavior: lower threshold offset means more raw targets.
`TRFT=1` is fast detection, which the datasheet says has reduced immunity
against reflections and other interferences. `VISU=0` means no vibration
suppression.

Important caveat: production uses RADC only, so THOF/TRFT/VISU do not remove
the RF energy from the raw ADC samples. They mostly affect K-LD7 internal
RFFT/PDAT/TDAT/DDAT outputs. They are still useful for diagnostic sessions:
PDAT/TDAT can show whether the K-LD7's own processing sees the same candidate
cloud or chooses a different dominant target.

Recommended diagnostic runs:

- capture `RADC + PDAT + TDAT` for a short controlled test, knowing this can
  change timing and data volume compared with production
- compare OpenFlight-selected RADC peaks against K-LD7 PDAT raw targets
- try `THOF` above 10 to see when reflections disappear from PDAT
- compare `TRFT=0` standard and `TRFT=2` long visibility against `TRFT=1` fast
  detection for diagnostic target identity

### DONE Frame Number

The datasheet explicitly recommends checking the DONE frame number to validate
real-time readout when streaming data-heavy RADC/RFFT frames. We already started
logging DONE/frame-number information for timing investigations. It should stay
part of debug output because frame gaps can masquerade as timing drift.

### Antenna Spacing Calibration

The datasheet lists Rx1/Rx2 spacing as 6.223 mm. The OpenFlight RADC code uses
an 8.0 mm effective spacing with a comment that it was calibrated against PDAT
reference data.

That may be the right effective value for our mounted/oriented board and signal
path, but it should be treated as a calibration constant, not a datasheet
constant. If absolute angles remain biased after the multipath problem improves,
this constant is one of the first things to revalidate.

## Why Multipath Creates Confidently Wrong Angles

The K-LD7 estimates angle from phase difference across receive antennas. At
roughly 24 GHz, the RF wavelength is about 12.5 mm, or about 0.49 in. A path
length difference of about a quarter inch can substantially change phase.

Indoor golf creates several paths:

1. Direct return: radar -> ball -> radar
2. Floor-reflected return: radar energy bounces from floor/mat and mixes with
   the ball return
3. Screen/net return: ball or radar energy scatters near the screen
4. Club/mat clutter: transient returns close to impact

The radar receives the vector sum of these returns. If the direct ball return
and a reflected return have similar Doppler speed, the FFT bin can contain both.
The resulting phase can point between them, or toward the stronger reflection.

This is why high SNR is not enough. A strong blended return can be confidently
wrong.

## Why Raising The Radar Can Help

Mounting the vertical K-LD7 close to the floor seemed attractive because the
ball starts near the floor. RF-wise, it may put the radar in a bad two-ray
geometry where the floor-reflected path is strong and phase-coherent with the
direct path.

Raising the radar can help because it:

- changes the direct/reflected path length difference
- changes the reflection angle off the floor
- reduces how much of the floor is inside the strongest antenna lobe
- can make the direct ball return dominate the blended vector

The goal is not to eliminate floor reflection completely. The goal is to make
the direct ball return strong and stable enough that the measured phase follows
the ball rather than the reflection.

## Recommended Mount Test

Current/recent setup:

- radar center roughly 4 in above floor
- vertical K-LD7 tilt around 10 deg
- radar-to-ball distance around 5 ft
- screen around 12 ft from ball in the TrackMan facility

Recommended A/B test:

- raise the enclosure by 6 in
- measure the actual radar phase-center height from the floor
- set vertical K-LD7 tilt to 6 deg
- set OPS tilt to 6 deg for consistency
- keep ball distance at 5 ft
- keep screen distance recorded

If the radar center is about 10 in above the ball/floor, the offline geometry
analysis should use:

```text
ball_above_radar_ft = -0.833
mount_deg = 6
ball_distance_ft = 5
```

This matters. If software assumes the old 4 in height offset while the radar is
physically 10 in high, the geometry fit can look wrong for the wrong reason.

Suggested test set:

- 10 shots with 8-iron
- 10 shots with 7-iron or 4-iron if time allows
- same ball position and screen position as the prior test
- record exact radar center height, mount tilt, ball distance, and screen
  distance

Signals that the mount change helped:

- fewer selected frames with crazy high/low geometry
- F1B range agrees better with speed/time distance
- peak bearing and band-weighted bearing agree more often
- two-frame candidate tracks have lower ambiguity
- fewer high-SNR frames with obviously wrong launch angle

## Software Direction

The selector should not try to force a radar launch angle on every shot. It
should score candidate tracks and emit radar only when the evidence is coherent.

Recommended candidate-track features:

- OPS speed-bin error
- SNR
- phase coherence
- bearing progression
- range progression from F1B
- geometry RMSE
- screen-time cap based on ball speed and measured screen distance
- agreement between peak bearing and band-weighted bearing
- agreement between F1B range and speed/time distance

Proposed behavior:

1. Scan a wider candidate window, possibly +/-50 bins around OPS speed.
2. Allow lower-SNR neighboring frames.
3. Build 2-frame and 3-frame candidate tracks.
4. Reject tracks that violate screen time, range progression, or geometry.
5. Emit radar only when one track clearly wins.
6. Fall back to estimated launch when candidate identity is ambiguous.

This changes the mental model from "find the strongest ball bin" to "find the
track that behaves like a golf ball."

## Practical Mitigations

Physical mitigations to test first:

- raise vertical K-LD7 from about 4 in to about 10 in
- reduce vertical K-LD7 tilt from 10 deg to about 6 deg for 7i/8i testing
- keep exact physical measurements in the session notes
- cap candidate frames at estimated screen arrival time plus a small margin
- test RF absorber or rough/non-reflective material near the floor line only
  after the height/tilt A/B test

Software mitigations to prioritize:

- add radar height as an explicit geometry input if not already configurable
- include screen distance in offline analysis and eventually live scoring
- use F1B range as a consistency check before using it as truth
- distinguish clean two-frame geometry from one-frame diagnostic evidence
- record rejection reasons in a way that separates "no signal" from
  "ambiguous/multipath signal"

## Working Conclusion

The K-LD7 can produce accurate launch angle when it selects clean ball frames.
The main current failure mode is not lack of signal. It is mixed signal.

The next best step is a physical A/B test: raise the radar and reduce tilt, then
replay the same analysis. If that reduces ambiguous candidate tracks, the fix is
mostly RF geometry plus better track scoring. If it does not, the system may need
more sensing diversity, such as a camera constraint, stronger ball marking, or a
different radar placement.
