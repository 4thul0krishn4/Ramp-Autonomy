# Crest-freeze diagnosis - pre-registration (rules fixed before the corrected-geometry samples)

Question: is the loaded-robot crest freeze (curved / 0.45 / loaded 0/7 reach the platform; curved / normal / loaded 6/8) the same bug class as the
construction-site freeze (a box-center-vs-top-surface authoring step at a slab join), or a different mechanism?

## Evidence gathered BEFORE any fix (`before_freeze_analysis.txt`, `seam_geometry.py`)
* Exact world-frame corner math over every seam of every ramp world: all seams match within 1.4 mm EXCEPT two, in every ramp world file:
  the crest (`fillet_top_3` top face z 1.050 -> `zone_c_exit` top face z 1.100: **+0.0500 m step**, sharp world +0.0502) and the ramp foot
  (`zone_a_approach` top 0.100 -> ramp start 0.050: **-0.0502 m**, sharp -0.0537). The already-fixed `construction_site.sdf` is within 1.1 mm everywhere.
  The breakpoint marker heights in the sharp world (0.15, 1.15) confirm the design surfaces are 0.10 and 1.10: the RAMP slabs were authored 0.05 m too low.
* Two instrumented freezes (curved / 0.45 / loaded, recorder = per-wheel joint velocity, /wheel_slip, detector signals, full-orientation ground truth):
  chassis frozen at the identical pose (gz x 8.018, z 1.157, pitch -3.6 deg) for 40-50 s; all four wheels turn (surface speed 0.09 m/s) while the chassis moves 0.0003 m/s;
  front wheel centre (8.297, 1.175) is 0.1233 m from the step's top corner (8.395, 1.100) against a 0.12 m wheel radius (contact within 3 mm, contact normal 38.7 deg from vertical);
  /wheel_slip mean 0.012 (blind: it compares wheel to command, not chassis to wheel); EKF x runs +4.5 m ahead of ground truth; /verified_progress_m grew 4.3 m (96% of the raw wheel distance).
=> same bug class (a 0.05 m step at a slab join), in different geometry (`ramp_world_*.sdf`, never fixed when the construction site was).

## Fix (decided before the samples)
Raise the ramp slabs (fillets, ramp, rails) so both seams close: curved worlds +0.050 m; sharp worlds +0.052 m (balances the two tilted-slab seams to -1.7/-1.8 mm). zone_a and zone_c untouched
(the robot spawn height and every earlier platform-z reference assume 0.10 / 1.10). All 7 `ramp_world_*.sdf` files; originals in `data/*.ORIGINAL`. Post-fix seams: curved <= 1.4 mm, sharp <= 1.8 mm.

## Ascent gate (step 4)
Finding: neither the validation test (ascent "PASS" = /ramp_state reached EXIT) nor the round-trip driver's ascent legs (end when EKF x crosses a target) had any physical-reality check; the descent's
/verified_progress_m gate exists only for leg 3. The descent-style gate would NOT have caught this freeze (verified progress kept growing: each stall/resume cycle re-accumulates ~5 s of spinning wheels).
Applied fix pattern = client refuses to accept completion unless independent evidence agrees, using the strongest independent evidence available to the harness: Gazebo ground-truth chassis displacement vs
the EKF/odometry displacement over each drive leg, tolerance 0.5 m (24 healthy leg ends <= 0.09 m); verified progress recorded, not gated. Validation test: `FAIL: ASCENT_POSITION_UNVERIFIED`, plus the existing
ground-truth platform-drive check (`DID_NOT_REACH_PLATFORM`). Round trip: legs 1, 2 (and 4) checked; an unverified ascent stops the run before leg 3.
The stack itself still has no on-robot physical ascent check; the on-robot options are limited by the same physics as the stall detector (no exteroceptive floor-motion sensor) - documented, not solved.

## Reverification (corrected turn procedure defaults, unscoped fixed detector 48625b76, guard-safe, one sim at a time)
12 runs each of the four loaded cells (curved 0.45, curved normal, sharp normal, sharp 0.45), 6 each of curved/normal/unloaded and sharp/normal/unloaded (regression), then the full 7-cell matrix and both round trips.
Completion = ascent PASS (ground-truth verified) AND full descent to the ramp foot AND max |y| <= 1.5 m. Wilson 95% intervals reported.
The fix "works" for a curved loaded cell if >= 11/12 complete (Wilson lower bound >= 65%). Loaded sharp cells: whatever they do is reported; if they still fail, the mechanism is diagnosed, not assumed to be the same.
Regression rule: the unloaded cells must keep 0 failures in 6.
