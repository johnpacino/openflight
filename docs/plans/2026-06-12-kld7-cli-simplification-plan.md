# K-LD7 CLI Simplification Plan

Status: proposed (pre-PR cleanup for `feat/kld7-frame-selection-improvements`)

## Problem

K-LD7 configuration has grown to 23 CLI flags across three eras of
development. Three structural issues:

1. **Estimator selection is now a false choice.** `--kld7-vertical-estimator
   {naive,geometry,two_ray}` implies alternatives, but the production design
   is a cascade — two_ray → geometry fit → geometry single-frame → naive —
   with each stage backing the one before it. Users should not pick an
   estimator any more than they pick an FFT.
2. **Rig geometry is retyped per launch.** Height/tilt/distance/offset/net
   describe the physical rig; they change when the rig moves, not per
   session. Retyping invites the config-vs-physical mismatches that cost us
   debugging sessions (tilt degeneracy, the 1-inch mat). Two defaults are
   also stale (`mount-tilt` 18.0, `ball-distance` 5.5 vs the documented
   4 in / 10° / 5.0 ft rig).
3. **12 `--experimental-kld7-*` flags** map 1:1 onto the internal
   `radc_tuning_params` dict — research knobs occupying top-level CLI space.

## Target shape

```bash
# the common case (after the rig is set up once):
scripts/start-kiosk.sh --kld7

# first run / rig changed (interactive, writes config/rig.json):
scripts/start-kiosk.sh --kld7-setup
```

### Tier 1 — enablement
- `--kld7` enables both radars (vertical + horizontal auto-detect, as today)
- `--kld7-port` / `--kld7-horizontal-port` remain (hardware override)

### Tier 2 — rig geometry: file-first, flags as overrides
- New persisted rig file (`config/rig.json`, git-ignored):
  `{height_in, tilt_deg, ball_distance_ft, net_distance_ft,
  angle_offset_deg, horizontal_offset_deg}`
- Written by `--kld7-setup` (interactive prompts; integrates with the
  upstream `scripts/setup/setup_kld7_devices.sh` flow) and by
  `--kld7-save-rig` after manual overrides
- Existing geometry flags kept as per-launch overrides of the file
- **Definition fix shipped with this**: `height_in` documented as “radar
  center above the surface the ball sits on” (mat ≠ floor; sensitivity is
  ~0.8°/inch)
- Defaults corrected to the standard rig: tilt 10.0, distance 5.0,
  height 4.0, net 10.0

### Tier 3 — estimator selection disappears
- The cascade becomes the only behavior; `--kld7-vertical-estimator` is
  removed from help (kept hidden for one release as a deprecated escape
  hatch, `naive`/`geometry` forcing the legacy stages)
- `--kld7-geometry` (start-kiosk alias) deprecated with a warning
- Session log continues to record the per-shot `selection_path`
  (two_ray / geometry / geometry_single_frame / naive / estimated), which
  is the observable that replaces the flag

### Tier 4 — research knobs consolidated
- The 12 `--experimental-kld7-*` flags collapse into repeatable
  `--kld7-tuning KEY=VALUE` (they already feed one dict); unknown keys
  warn-and-ignore
- `--experimental-kld7-raw-radc-logging` is promoted out of experimental to
  `--kld7-raw-logging` — it has become the project’s replay/validation
  backbone, not an experiment

## What gates two_ray-by-default when `--kld7` is passed

Nothing architectural — the cascade already degrades to exactly today’s
behavior when two_ray refuses. The flip is one default plus these gates:

1. **The 2026-06-16 TrackMan session passes**: fresh-truth MAE ≤ ~3.5° at
   ≥70% coverage on irons/wedges, marginal (one-dot) tier sane, no
   pathological emissions.
2. **Long-club decision** from the session’s driver block: measured driver
   shots currently score worse (~6–7°) than the club estimate (~3.5°);
   decide per-family confidence cap vs accept, then implement (small).
3. **Geometry defaults + setup flow landed** (Tier 2), so default users
   run with a correct rig description — two_ray is more geometry-sensitive
   than naive and must not inherit the stale 18°/5.5 ft defaults.
4. Lead-dev review of the PR; release note that launch-angle source and
   confidence semantics change.

## Phasing

- **Phase 1 (this PR)**: corrected defaults, deprecations (estimator flag
  hidden, `--kld7-geometry` warning), experimental→`--kld7-tuning`
  consolidation, raw-logging promotion. No behavior change for existing
  command lines.
- **Phase 2 (post-TrackMan, small PR)**: default flip to the full cascade;
  rig file + `--kld7-setup`.
- **Phase 3 (when calibration tooling lands)**: reflector-derived
  `angle_offset_deg` written into the rig file by a calibration command;
  per-shot GPIO+range placement solve demotes `ball_distance_ft` to a
  sanity default.

## Out of scope

Horizontal-axis estimator work (still legacy pipeline), ball-speed
correction flag (separate branch), UI changes.
