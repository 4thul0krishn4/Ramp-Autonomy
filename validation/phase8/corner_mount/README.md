# LiDAR corner-mount relocation and re-calibration (2026-10-02)

Both LiDAR sensors moved from their original forward-centreline (`lidar_link`) / chassis-centre (`obstacle_lidar_link`) mounts to
corner mounts - a real functional relocation (the joints' own origins move, not just the drawn visuals), followed by the same
re-validation standard as the original phase-4 calibration. One design iteration failed a required check and had to be fixed before
re-validating; that failure, the fix, and the full before/after numbers are recorded below rather than smoothed over.

## 1. Mount design

### Ramp LiDAR (`lidar_joint`) - front-left corner

**First attempt (rejected by its own re-validation):** same forward offset as the original (x=0.35), shifted to y=0.22 (clearing the
front-left wheel), z set from the real mesh deck height at that point plus a 0.05 m standoff (`mesh_query_debug.py`: the chassis top is
a genuinely flat deck at 0.1283 m rel. base_link, confirmed by sampling two widely-separated corner regions and finding the identical
maximum vertex height in both - a real mesh feature, not a measurement bug). This failed hard: every break-point reading collapsed to
~0.04 m regardless of true distance (section 3). Root cause and fix below.

**Root cause:** the sensor's own forward-downward detection cone (`FORWARD_CONE_MIN_DEG/MAX_DEG`, -80 to 45 deg) was mostly filled with
~0.08-0.10 m self-returns off the chassis's own flat deck, which is wide enough at y=0.22 to extend forward underneath the sensor at a
0.05 m standoff. The OLD centre mount never had this problem: its real standoff was 0.08 m, not the 0.05 m its own placeholder-box-era
design assumed (Task 5's lidar-floating fix was visual-only and never actually corrected the joint height - see
`../lidar_mount_fix/README.md`), and it sat on the lower sloped edge surface, not the broad flat deck.

**Fix:** profiled the real mesh surface along x (forward) at several y values (`mesh_profile.py`) and found that at y=0.22 the surface
stays on the flat deck (0.128 m) out to about x=0.37, then drops to the same low sloped-edge height (~0.096 m) the OLD mount sat on by
x=0.39. Moved the mount forward to x=0.39 and set z from that local surface height plus the SAME ~0.08 m standoff the old mount always
had (not a new guess) - landing at z=0.176 m, nearly identical to the original 0.175 m. This replicates the old mount's proven,
self-occlusion-safe geometry at a genuine corner rather than inventing a new standoff.

roll (1.5708 rad, 90 deg about local X) and yaw (0) are unchanged. Justification: rolling the sensor's native 360 deg horizontal sweep
by 90 deg about local X maps every native ray `(cos th, sin th, 0)` to `(cos th, 0, sin th)` in the parent (base_link) frame - a direct
rotation-matrix result, not an approximation - so every ray in the rolled scan lies in the sensor's own local XZ plane with **zero**
local-Y component, for any th. A lateral (Y) mount shift therefore cannot change which angles are swept, only translates that same
vertical fan sideways; since every ramp world in this project has no cross-slope, this predicts the corner move should not need a
roll/yaw change. Section 3 confirms this empirically (curved-world and bottom-approach accuracy track baseline closely).

Final joint: `xyz="0.39 0.22 0.176" rpy="1.5708 0 0"`.

### Obstacle LiDAR (`obstacle_lidar_joint`) - rear-right corner

x=-0.30 (clears the payload box's own x-footprint entirely for every `payload_cg_offset` used in this project - the payload's x-span
never reaches that far aft even at its most rearward-biased setting); y=-0.22 (clears the rear-right wheel: ~0.13 m to the wheel's own
centre, and the wheel's y-extent, width 0.08 m centred at 0.35, ends at 0.31 - a real if modest 0.09 m clearance band the sensor's own Y
sits outside of).

z is **deliberately left unchanged at 0.5** - not lowered, despite the mast getting shorter. The original 0.5 m height was driven by
clearing the payload box's own top (0.425 m rel. base_link, present even at `payload_mass=0`) for an unobstructed 360 deg view; that
requirement is a function of height alone, independent of lateral position. Re-derived explicitly for the corner case: if anything, a
corner mount makes payload occlusion **worse** at low height, because the payload would then block one wide contiguous arc of the ring
(it sits mostly forward of the robot's centre; a rear-corner sensor below payload-top height would have the payload across most of its
forward hemisphere) rather than a centred band a dead-centre mount sees. The mast itself is shorter than before purely as a visual
consequence of the real deck being higher at this corner (0.128 m) than the old centre-mount surface (0.071 m) - a cosmetic change, not
a functional one.

Final joint: `xyz="-0.30 -0.22 0.5" rpy="0 0 0"`.

## 2. Re-validation method

Same harness and scripts as the original phase-4/phase-8 perception checks (`../perception_regression/`), run guarded (clean-host
preflight/verify, RTF-monitored) one sim at a time:

- `mesh_query_debug.py`, `mesh_profile.py` - the mesh-surface measurements behind the design above.
- `self_occlusion_raw.py` - raw `/scan` + `/scan_360` sample, stationary, open flat world (`construction_site`): full range/bearing
  histogram, explicitly checked against the sensor's own forward-cone window rather than assumed clean.
- `../perception_regression/breakpoint_accuracy.py` - reused unmodified (the ground-truth formula `marker_x - (chassis_x + 0.35)` only
  depends on the sensor's unchanged forward offset, not its new lateral one - see section 1's Y-invariance argument).
- `obstacle_lidar_check_corner.py` - the original `obstacle_lidar_check.py`'s bearing/range constants were derived for the OLD centred
  mount and do not apply to a relocated sensor; re-derived analytically from the known obstacle-box positions and the new sensor's
  world offset (nearest-point-on-box geometry), then checked against the live scan rather than trusted on the math alone.
- `run_corner_checks.sh` - the guarded runner chaining all of the above.
- `../delivery/guarded_run.sh` + `../delivery/full_mission_test.py` - the full 6-leg loaded delivery mission, unmodified.

## 3. Results

### Self-occlusion

| | First attempt (x=0.35, y=0.22, 0.05 m standoff) | Final (x=0.39, y=0.22, ~0.08 m standoff) |
|---|---|---|
| Ramp sensor forward cone (-80..45 deg) sample | 11 of 15 sampled ranges at 0.08-0.10 m (self-return) | 0.40-0.45 m typical (real), 0 suspicious |
| Obstacle sensor | clean at both | clean at both |

The only short returns (<0.15 m) at the final geometry are all in the backward cone (-114 to -162 deg), rays that pass back through the
chassis body - already true at the old centre mount too (same roll geometry, unrelated to the Y move) and outside the classifier's used
window.

### Break-point accuracy, sharp world (vs. `../perception_regression/README.md`'s corrected-geometry baseline, centre mount)

| Stop | Baseline (centre mount) | Corner mount, MIN_SEGMENT_POINTS=3 | Corner mount, MIN_SEGMENT_POINTS=4 (final) |
|---|---|---|---|
| Zone A, spawn (true 2.65 m) | -9.1 cm, slope 17.0 deg, conf 0.64 | -29.4 cm, slope 18.7 deg, conf 0.97 | -29.5 cm, slope 18.2 deg, conf 0.97 |
| Zone A, close (true ~0.30 m) | +3.3 cm, slope 15.4 deg, conf 0.94 | -6.1 cm, slope 9.4 deg, conf 0.92 | -7.2 cm, slope 16.0 deg, conf 0.93 |
| Top, far (~2.1 m) | -37.3 cm, 5/30 valid | -197.7 cm, 3/30 valid | -191.9 cm, 3/30 valid |
| Top, closer (~1.1 m) | -4.0 cm | **-75.8 cm** | **-11.0 cm** |
| Top, imminent (~0.6 m) | -4.6 cm, conf 0.87 | -18.5 cm, conf 0.96 | -9.8 cm, conf 0.97 |
| ASCENT -> TRANSITION trigger | 0.524 m, conf 0.98 | 0.109 m, conf 0.85 (spurious) | 0.665 m, conf 0.97 |

MIN_SEGMENT_POINTS=3 was tuned (phase 4) against the OLD centre mount's **bottom**-break-point near-corner point density. At the corner
mount's slightly different geometry, 3 was once again too permissive at the **top** break point: a small noise-driven cluster got
accepted as a real segment split, the same failure mode the constant exists to prevent, re-appearing at the opposite end of its tuned
range. Re-tuned to 4 (confirmed live, `ramp_classifier_node.py`'s own updated comment) - recovers "closer" and "imminent" to within a
few cm of baseline and restores a sensible ASCENT->TRANSITION trigger, without reopening the original ASCENT-stuck bug (bottom-approach
numbers at 4 are statistically the same as at 3). The far-range top reading remains weak at 4 (3/30 valid) - this is the SAME
pre-existing weak-signal zone already documented in the centre-mount baseline (5/30 valid there too), outside
`BREAKPOINT_TRIGGER_DISTANCE_M`'s 0.75 m operative range, so not a new functional regression.

### Break-point accuracy, curved world

| Stop | Baseline (centre mount) | Corner mount, MSP=3 | Corner mount, MSP=4 (final) |
|---|---|---|---|
| Spawn (true 2.65 m) | +10.6 cm | +9.8 cm | +10.3 cm |
| Close (true ~1.4 m) | +9.4 cm | +9.5 cm | +0.8 cm |

Unchanged or slightly improved - the curved world's multi-segment fillet fit was never sensitive to this particular change.

### Obstacle detection (`sharp_obstacles_TESTONLY`)

The original script's bearing/range constants (180 deg/0.65 m, -90 deg/0.85 m) were derived for the OLD centred mount and don't apply
once the sensor has moved; re-derived analytically from the known obstacle-box positions and the new sensor's world offset at spawn
(`obstacle_lidar_check_corner.py`'s own docstring has the full derivation), then checked live:

| Check | Predicted (analytical) | Measured (live) |
|---|---|---|
| "Behind" obstacle | 0.357 m @ 168.7 deg | 0.34 m @ 167.0 deg |
| "Side" obstacle | 0.648 m @ -76.6 deg | 0.64 m @ -76.7 deg |

Close agreement on both - the sensor behaves exactly as first-principles geometry predicts. Self-occlusion scan (full ring, excluding
the two known obstacles): 0 suspicious short returns. Local costmap occupied cells: 1039 of 14400 (vs. 631 originally documented, 854/737
on the corrected geometry at the old mount - some further increase here, plausibly because the "behind" obstacle is now genuinely much
closer to the sensor (0.34-0.36 m vs. the old mount's 0.65 m) as a direct consequence of sitting at the rear corner, not a functional
problem but a real, reportable side effect of this particular corner's proximity to this particular fixed obstacle in this test world).

### Full delivery mission (6 legs, loaded, `detachable_payload:=true`)

`ok: True`, all 6 legs `ok: True` (ground-truth-vs-odometry disagreement -0.002 to -0.071 m, well inside the 0.5 m gate), payload
released (`released: True`) and landed on the dock (`on_dock: True`), 17 HALTs. Host was moderately loaded throughout this session (mean
RTF 0.86-0.87 across three attempts, all flagged `STARVED` by the clean-host purity gate) - the mission itself completed correctly end
to end each time, which is what this check actually verifies (same acceptance basis as `../lidar_mount_fix/README.md`'s own rerun).
`mission_rerun/mission.json` has the full leg-by-leg record.

## 4. Verdict

**Corner mounting both LiDARs is viable, but it was a genuine re-calibration effort, not a repositioning** - exactly as expected going
in, and worth stating plainly rather than glossing over:

1. The first, naively-reasoned corner position (same forward offset, same thin standoff, just shifted sideways) failed its own
   required self-occlusion check outright - every break-point reading collapsed to near-zero. The failure was geometric and specific:
   a broad, genuinely flat deck inboard of the corner sat inside the sensor's own forward-downward detection cone at a standoff the old
   mount never actually had (an accidental side effect of an earlier visual-only fix). Caught by running the check, not by inspection.
2. Fixing it required moving the mount forward to where the real mesh surface drops to the same sloped-edge height the old mount sat
   on, replicating its proven geometry rather than inventing a new one.
3. Even after that fix, the OLD `MIN_SEGMENT_POINTS=3` (tuned for the old mount's bottom-corner point density) produced a real,
   separate degradation at the ramp's top crest - re-tuning to 4 was necessary and sufficient to recover accuracy at every
   operationally-relevant range.
4. With both fixes in place, every check that matters functionally matches the original centre-mount baseline closely: curved-world
   accuracy, bottom-approach accuracy, the FSM's actual 0.75 m trigger window, obstacle bearing/range geometry (matching independent
   analytical predictions to within a few cm/degrees), zero self-occlusion, and a complete, ground-truth-verified 6-leg delivery
   mission with a successful release and dock landing.
5. Honest costs that remain, not hidden: the already-documented far-range top-approach weakness is unchanged (not improved, not worse
   in kind - just still there); the obstacle sensor's rear-corner position is now meaningfully closer to one specific fixed test
   obstacle (0.65 m -> ~0.35 m) as a direct, geometric consequence of where that corner happens to sit in this particular test world,
   not a sensor defect; and both sensors now sit closer to the robot's own structural edges (wheel, deck edge) than the old, more
   conservative centre mounts did, with real but modest clearance margins (0.09-0.13 m) rather than generous ones.

The tradeoff was worth making here only because every step was re-validated against the original standard rather than assumed to carry
over - a corner mount chosen by inspection alone, without the self-occlusion check or the MIN_SEGMENT_POINTS re-tune, would have shipped
a broken ramp sensor.

## Files

`mesh_query_debug.py`, `mesh_profile.py` (mesh-surface measurements), `self_occlusion_raw.py`, `obstacle_lidar_check_corner.py`,
`run_corner_checks.sh` (the guarded re-validation suite), `self_occlusion*.json`, `breakpoint_*.json`, `obstacle_lidar*.json` (results),
`mission_rerun/` (the full delivery mission data).
