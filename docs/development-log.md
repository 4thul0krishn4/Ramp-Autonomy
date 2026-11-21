# Development log

This is the project's original README, kept intact as the chronological, phase-by-phase engineering record - the detail behind the summary
layer in `docs/`. Start at the repository root [README.md](../README.md) for the current entry point; `architecture.md`, `results.md`,
`retrospective.md`, and `known-limitations.md` in this directory distill what's below into a polished, standalone form. This file is preserved
for anyone who wants the full session-by-session trail, written as it happened, including findings that were later superseded - each
supersession is itself noted inline, not deleted.

---

# Ramp Autonomy

A ROS2/Gazebo simulation project solving **ramp ascent and descent
autonomy for a loaded wheeled AMR** — the class of problem that
flat-floor Nav2/MPPI navigation doesn't handle well: IMU pitch bias
corrupting odometry, shifting payload CG changing the effective
tip-over margin in real time, and wheel slip becoming a distinct
failure mode from anything seen on flat ground.

Built in simulation first, with an explicit path to hardware later.

## Why this exists

Construction-site AMRs (the kind that move material between floors)
have to cross temporary ramps constantly, and most published
navigation work — including standard Nav2/MPPI tuning — assumes flat
ground. This project treats the ramp itself as a distinct control
and safety problem rather than something the flat-floor planner
should just muscle through.

## Architecture

```
ramp_autonomy_ws/
├── ramp_autonomy_description/   robot xacro, payload link (mass + CG offset are launch args)
├── ramp_autonomy_gazebo/        ramp_world_sharp.sdf, ramp_world_curved.sdf, spawn launch
├── ramp_autonomy_nav/           5-state FSM (approach/ascent/transition/descent/exit/halt)
├── ramp_autonomy_perception/    LiDAR-based ramp slope + break-point classifier
├── ramp_autonomy_msgs/          RampPercept.msg (phase 4 perception output)
├── ramp_autonomy_control/       stability margin monitor, wheel slip, descent braking
├── ramp_autonomy_bringup/       single launch file tying it all together
└── validation/phase8/           world x friction x payload test matrix (see phase 8 below)
```

(`ramp_autonomy_*` are ROS2 packages under `src/`; `validation/` is a
plain top-level directory, not a package - it holds the phase 8 test
harness and its logs, not code any package depends on.)

State flow:

```
APPROACH -> ASCENT -> TRANSITION -> DESCENT -> EXIT
    ^                                              |
    +---------------- HALT (any state) <-----------+
              triggered by /stability_margin
```

## Status

| Phase | Package(s) | Status |
|---|---|---|
| 1. Environment & repo scaffolding | — | done |
| 2. World & robot model | `ramp_autonomy_gazebo`, `ramp_autonomy_description` | done (initial ramp geometry + payload link) |
| 3. Baseline navigation integration | `ramp_autonomy_nav` | done (Nav2 `controller_server` running `nav2_mppi_controller`, live-switched by `/ramp_state` - see below) |
| 4. Ramp perception & classification | `ramp_autonomy_perception`, `ramp_autonomy_msgs` | done (LiDAR two-segment break-point/slope fit, feeding a pre-emptive state-machine trigger - see below) |
| 5. Slope-aware planning & control | `ramp_autonomy_nav/config` | not started |
| 6. Stability margin & safety halt | `ramp_autonomy_control`, `ramp_autonomy_nav` | done (tip-over margin + per-wheel slip both live and wired into HALT; cmd_vel watchdog + param-push failure handling added - see below) |
| 7. Controlled descent braking | `ramp_autonomy_control`, `ramp_autonomy_nav` | done, across three passes (see below) - first pass (`vx_max`-capping) shipped with a hidden dependency on two FSM bugs and an unclimbable test world; a second pass root-caused and fixed both FSM bugs (`ASCENT`'s missing exit, then HALT-recovery erasing `EXIT`), corrected the test world's friction, and replaced `vx_max`-capping with `ax_max`/`az_max`-capping, which matches the unbraked baseline instead of causing 146 HALTs |
| 8. Validation suite & docs | `ramp_autonomy_gazebo`, all packages | done (world x friction x payload test matrix, pre-declared pass/fail criteria, results and final project status below; two perception bugs the matrix surfaced were root-caused and fixed in a follow-up pass, matrix rerun corrected); **the matrix's own historical results were then found to only check wheel odometry, never Gazebo ground truth - rerun with the EKF live and ground-truth polling added, 3/7 cells changed verdict from MARGINAL to FAIL, all real problems ground truth caught that odometry alone missed - see Known Gaps and "Full matrix, corrected" below** |
| 8, session A/B/C/D/E/F/G. `construction_site.sdf` (realistic-site blockout) + EKF localization fix + crosstrack fix + live stall detector + goal-verification gate + freeze root-caused and fixed + populated with real Fuel props + arc-turn crosstrack activation gap root-caused and fixed + leg-4 drift characterized and closed | `ramp_autonomy_gazebo`, `ramp_autonomy_nav`, `ramp_autonomy_control` | **unloaded**: full round trip verified live, no safety abort, correct -16° ramp pitch, healthy margins (structural blockout cited against real-world standards - `docs/construction_site_design.md`; turnaround localization fixed with EKF, cross-track tracking fixed via a real `PathAlignCritic` misconfiguration - see Known Gaps). **loaded (100 kg/+0.1 m CG): completes the full round trip, root cause found and fixed, not just detected and refused.** Sessions B/C built a live stall detector and an independent goal-verification gate that correctly forced HALT and refused false success while the underlying cause stayed unknown. Session D instrumented per-wheel commanded-vs-actual velocity (`/joint_states`, real physics) during a fresh repro and found the freeze was wheels genuinely spinning at commanded rate (`/wheel_slip`~0) while the chassis stayed completely frozen - confirmed by cross-referencing the exact frozen position against `construction_site.sdf`'s own collision geometry: a real, isolated 0.05 m vertical step at the `elevated_level`/`ramp_fillet_top_3` seam (an SDF pose-vs-box-center authoring bug, not a design constraint - six other seams in the same fillet chain match to within 1.1 mm). Fixed in the world file and reverified live end to end. **Session E populated the site with 22 real Gazebo Fuel props** (barriers/pallets/crates/a rack/a shipping-container office/a dumpster/cones/fencing - real assets, real dimensions, zoning per the original design doc), verified clear of the robot's actual proven path (exact-footprint math + a buffer check against 1197 real logged trajectory points, not eyeballed), and reverified the loaded round trip still completes (`ok=True`) with the site populated. **Session F root-caused why leg-4 drift kept growing past its own worst-recorded case**: `flat_mode` (the profile live for the entire arc-turn, since it happens on a flat platform before `DESCENT` ever fires) never got the `path_align_offset` fix `descent_mode` got two sessions earlier - `PathAlignCritic` was fully disabled for the whole 16-point arc, confirmed via an external proxy reconstructed from the real critic source and live data (not guessed), identically regardless of payload. Also confirmed, side by side: this is a genuinely different mechanism from the curved-world seam-stumble finding and from the arc-turn's own wheel-slip/motion-stall-driven HALT-thrash - three distinct causes, not one shared weakness. Fixed (`flat_mode.path_align_offset` 20->3) and reverified: arc-phase cross-track deviation down 59%, full round-trip leg-4 drift down to 0.76 m (the smallest logged for this maneuver to date). The smaller drift did newly reach one of session E's gate-area props (0.58 m clearance, reproducing the elevated-level freeze's own signature) - those props were removed rather than re-margined off too little data (2 points). **Session G closed this with a real distribution**: 8 independent fresh-launch trials per payload config. Loaded drift (the binding case) converged by n=6-8 (min=0.526 m, max=2.535 m, mean=1.263 m, sample stdev=0.695 m, n=8) - a real, stable distribution, not still swinging, but inherently ~5x wide trial to trial. Unloaded drift was tight (n=4 valid, min=-0.203 m, max=0.005 m). (The 4-of-8 unloaded arc-turn non-completions first reported here were an artifact of the harness leaking stale processes and did not reproduce on a clean host - see the Correction in Known Gaps; the drift numbers carry the same caveat.) Applying this project's own margining convention (max drift + 1.5 m buffer = 4.035 m required clearance) against `access_road`'s own 3.05 m half-width: no position on the road can satisfy it, so the removed gate-area props stay removed **permanently**, not pending further work - see Known Gaps and `validation/phase8/construction_site_descent/README.md`'s "Drift characterization" section for every trial and the full derivation. |

## Quick start

```bash
# from ramp_autonomy_ws/
colcon build --symlink-install
source install/setup.bash

# unloaded run, sharp-transition world
ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=sharp

# loaded run, payload offset forward 0.1m, curved-transition world
ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \
    world:=curved payload_mass:=100.0 payload_cg_offset:=0.1
```

Watch `/ramp_state`, `/stability_margin`, and `/cmd_vel_scale` in
`ros2 topic echo` or RViz to see the state machine react as the robot
crosses the ramp.

If the Gazebo GUI client crashes on your setup (seen here as a
`libpthread.so.0: undefined symbol: __libc_pthread_init` symbol clash
coming from a snap-packaged GTK lib on the `LD_LIBRARY_PATH`), add
`headless:=true` to run Gazebo server-only - physics, sensors, and
every topic/bridge work identically either way, you just don't get a
3D view.

## Nav2 MPPI integration (phase 3)

`ramp_autonomy_bringup` now brings up a real Nav2 `controller_server`
running the `nav2_mppi_controller` plugin (`FollowPath`), plus its
internal `local_costmap` (LiDAR-fed, `odom`-frame rolling window) and a
`nav2_lifecycle_manager` that auto-activates it - see
`ramp_autonomy_nav/config/nav2_params.yaml`. This is deliberately
scoped to the controller layer only: no `planner_server`/`bt_navigator`/
AMCL/map yet, since there's no waypoint-following goal source until
phase 5 ("slope-aware planning & control"). What phase 3 needed was a
*live* MPPI controller whose velocity/heading-tolerance bounds actually
respond to ramp state, and that's what's here.

`nav2_param_switcher_node.py` (new node, kept separate from
`state_machine_node.py` - see its own docstring for why) subscribes to
`/ramp_state` and pushes the matching profile from
`ramp_autonomy_nav/config/ramp_mppi_params.yaml` into
`controller_server`'s live parameters via its standard
`/controller_server/set_parameters` service on every state change:

| `/ramp_state` | profile | `FollowPath.vx_max` | `FollowPath.wz_max` | heading tolerance |
|---|---|---|---|---|
| APPROACH, EXIT | `flat_mode` | 0.8 m/s | 1.2 rad/s | 15° |
| ASCENT, DESCENT | `ramp_mode` | 0.35 m/s | 0.4 rad/s | 5° |
| TRANSITION | `transition_mode` | 0.2 m/s | 0.2 rad/s | 3° |
| HALT | `halt_mode` (new) | 0.0 m/s | 0.0 rad/s | 3° |

**Verified against a live run** (`world:=sharp`, driven manually via
`ros2 topic pub /model/ramp_robot/cmd_vel ...` since there's no
waypoint source yet to exercise `FollowPath` end-to-end): `ros2 param
get /controller_server FollowPath.vx_max` read back `0.8` in APPROACH,
`0.35` immediately after `/ramp_state` flipped to ASCENT crossing onto
the ramp, and `0.0` when a HALT was triggered - all three matching the
table above and confirming the parameter push actually lands on the
running controller, not just that the switcher node believes it sent
something.

`halt_mode`, `robot.urdf.xacro`'s Gazebo plugins, and the spawn-height
fix below were **not in the original placeholder set** - added because
the robot could not physically move or produce sensor data without
them (see next section). Everything else in `ramp_mppi_params.yaml`
(`ramp_mode`/`transition_mode`'s exact numbers) is still an untuned
placeholder per its own TODO comments - phase 3 only needed the
switching *mechanism* proven live, not final tuning, which phase 5's
test matrix is the right place for.

### Bugs found and fixed while wiring this up

None of these were introduced by phase 3's changes, but the wiring
work is what exposed them - each was silently making the existing
bootstrap FSM/perception stack a dead end even before Nav2 entered the
picture:

- **No wheel actuation.** `robot.urdf.xacro` had wheel joints but no
  Gazebo drive plugin - `/cmd_vel` (from anything, MPPI included) had
  nothing listening on the Gazebo side. Added the `gz-sim-diff-drive-
  system` plugin, paired left/right across all 4 wheels for skid
  steer. **Its cmd_vel subscription topic has no SDF override tag** (
  confirmed empirically with `gz topic -l` after spawning - a bare
  `/cmd_vel` publish went nowhere) and is hardcoded to
  `/model/ramp_robot/cmd_vel`; `ramp_world.launch.py`'s bridge and
  `controller_server`'s cmd_vel remap were both updated to target that
  exact topic instead.
- **No IMU or LiDAR sensors.** The URDF had `imu_link`/`lidar_link`
  frames but no `<sensor>` blocks, so Gazebo never produced `/imu` or
  `/scan` - `state_machine_node.py`, `stability_monitor_node.py`, and
  `ramp_classifier_node.py` were all subscribed to topics that could
  never exist. Added both sensor definitions (the two system plugins
  they need, `gz-sim-imu-system` and `gz-sim-sensors-system`, were
  already declared in both world files, just unused).
- **Robot spawned 7.5 cm into the ground.** See the fix already applied
  in `ramp_world.launch.py` (`-z 0.05` -> `0.23`) - tracing the URDF's
  joint offsets showed the old value put the wheel bottoms at
  z = -0.075 against Zone A's actual driving surface at z = 0.1.

## Ramp perception (phase 4)

### Sensor fix this depended on

`robot.urdf.xacro`'s LiDAR was mounted level (`rpy="0 0 0"`). A single-
-plane 2D LiDAR mounted level physically cannot produce ground-return
points on flat approach ground - every forward beam at mount height
sails straight over flat terrain and only ever hits something once
that something rises into the beam's path (at which point you're
seeing a rising face as a wall, not sampling "the ground ahead" the
way a ground-plane fit needs). Rolled the mount 90° about local X
(`rpy="1.5708 0 0"`) instead, turning the sensor's native horizontal
(yaw) sweep into a vertical fore-aft fan: angle=0 stays pointed
straight ahead, negative angles tip down toward the ground, positive
tip up. That makes a single scan's ordinary polar-to-Cartesian
conversion (`x = r*cos(angle)`, `y = r*sin(angle)`) directly yield
(forward distance, height-relative-to-sensor) ground-profile points -
flat ground is a zero-slope line in that space, an incline is a line
whose slope is `tan(incline angle)`.

**Trade-off, not free:** `/scan` also feeds Nav2's `local_costmap`
`obstacle_layer` (phase 3). Rolling the LiDAR into a fore-aft profiling
sensor means it no longer sees a 360° horizontal ring - side/rear
obstacle detection is lost. A single 2D LiDAR can't serve both roles at
once; a real build would want a second, dedicated sensor. Not fixed
here since it's outside phase 4's scope - flagged below.

### Algorithm: piecewise (two-segment) least squares, not RANSAC

Take the forward cone (-80° to +20°, mostly downward), convert every
valid return to (x, y), sort by x, and try every plausible split into a
near/far pair: fit ordinary least squares to each side independently,
keep the split with the lowest combined RMS residual. That directly
produces both outputs needed - where the slope changes (the break
point) and what the two slopes are - because it's fundamentally a
change-point problem, not an outlier-rejection problem.

That's why this isn't RANSAC: RANSAC is built to robustly fit ONE model
while discarding a minority of outlier points, but here the "second
population" (points past the break point) is the signal being looked
for, not noise to reject. A single global RANSAC or plain-OLS fit would
average across the break point and blur exactly what needs localizing.
Segmented OLS costs more (O(n²) over ~100 points in the cone, trivial
at 10 Hz) and directly optimizes for the actual required output.

If the two-segment fit isn't meaningfully better than one global line
(not enough points on both sides, or the slope difference is below
`MIN_SLOPE_DIFF_DEG`), that means there's no break point in view right
now: reports `breakpoint_distance_m = -1.0` and `slope_deg` from the
single-line fit instead (e.g. "on a uniform incline/flat, nothing
changing ahead"), rather than a split driven by noise. Confidence is
`1 - rms_residual / RESIDUAL_CONFIDENCE_SCALE`, clamped to [0, 1].

### Measured accuracy (live `world:=sharp` run, headless)

Robot driven manually via `ros2 topic pub /model/ramp_robot/cmd_vel`
(no goal source exists yet - see phase 3's known gaps); ground truth
computed from `/odom` + the LiDAR's known 0.35 m forward mount offset,
compared against the sharp world's `breakpoint_bottom_marker` (x=4.0)
and `breakpoint_top_marker` (x=7.5):

| Approach | true distance to break point | measured `breakpoint_distance_m` | error | measured `slope_deg` | confidence |
|---|---|---|---|---|---|
| Stationary in Zone A, bottom break point ahead | 2.65 m | 2.61-2.63 m | ~-3 cm | 17.7-17.8° (true 16°) | 0.61 |
| Closer to bottom break point | 0.31 m | 0.35 m | +4.3 cm | 16.9° | 0.74 |
| On the ramp, top break point ahead, far | ~2.16 m | 2.25 m | +9 cm | 75° (see below) | 0.92 |
| On the ramp, top break point ahead, closer | ~1.16 m | 1.25 m | +9 cm | 70° (see below) | 0.85 |
| On the ramp, top break point ahead, imminent | ~0.60 m | 0.64 m | ~+4 cm | 71° (see below) | 0.80 |

Break-point **distance** is accurate to a few cm across every case
tested (and tracked 1:1 with the robot's own odometry advance between
readings, confirming it's locking onto the real fixed geometric
feature, not drifting). Slope accuracy for the bottom approach is good
(≤2° error). The **top-approach slope readings are a real, informative
artifact, not a bug**: `ramp_world_sharp.sdf`'s break points are
literally sharp - two separate box models touching at a seam - and a
downward ray grazing that seam at a shallow angle can catch a sliver of
the ramp box's end face, which is a near-vertical surface. The
two-segment fit correctly identifies this as "a different surface
starts here" (hence the still-accurate distance) but reports the end
face's near-vertical angle rather than a meaningful terrain slope. The
state machine's pre-emptive trigger only consumes
`breakpoint_distance_m`, not `slope_deg`, so this doesn't affect the
one thing phase 4 actually needed - but `slope_deg` shouldn't be
trusted at very close range to a sharp corner.

**Pre-emptive TRANSITION trigger, confirmed live:** with the robot
climbing at a steady -16° pitch (IMU quaternion identical between two
consecutive checks - pitch had not moved at all), `/ramp_state` flipped
ASCENT -> TRANSITION the moment `breakpoint_distance_m` crossed
`BREAKPOINT_TRIGGER_DISTANCE_M` (0.75 m) with confidence 0.80. The
IMU-based fallback condition (`pitch decreasing AND below threshold`)
was provably false at that instant (pitch was steady, not decreasing) -
proof the perception path, not the IMU fallback, is what fired it.

### `world:=curved` behavior (fillet transitions, no single sharp corner)

Same overall bottom break point (world x=4.0) as the sharp world, but
approached via three short increasing-angle segments (5.3° -> 10.7° ->
16°) instead of one discontinuity. Logged, not assumed identical:

- **Farther out (spawn, ~2.65 m to the fillet start): distance
  estimate was noticeably worse** (3.09 m measured vs. 2.65 m true, a
  44 cm error - an order of magnitude worse than the sharp world's few
  cm at similar range). A gradual onset is a weaker change-point signal
  than a sharp discontinuity, and it's more exposed to the "too few
  far-side points at long range" limitation below.
- **Closer in (~1.42 m to the fillet start): accuracy recovered** to
  ~5 cm error (1.37 m measured), comparable to the sharp world.
- **Slope reading does not track the fillet's local onset angle.** At
  both distances the far-segment slope came back close to the *main
  ramp's peak angle* (15.6° and 16.5°) rather than the first fillet
  segment's true local angle (5.3°). The far segment's individual
  fillet sub-segments are only ~0.3 m long, so once a split lands
  anywhere near the fillet's start, the far side's point population is
  dominated by the long uniform 16° section beyond it, not the brief
  gentle onset immediately after the split. A two-segment fit
  fundamentally can't resolve a *multi-segment* curve into its
  individual pieces - it was never expected to, and this confirms that
  rather than assuming it.

### Known algorithm limitation: far-range accuracy needs enough points

A synthetic (non-Gazebo) test - see the reasoning that motivated it -
showed that when fewer than `MIN_SEGMENT_POINTS` (5) rays actually land
past the true break point, the detector is forced to pad the far
segment with some genuinely-flat points to reach the minimum, which
pulls the reported break point closer and the slope shallower than
truth. Because consecutive 1° angle steps cover a rapidly growing
forward distance at shallow angles (`x = h_lidar / tan(angle)`), this
only bites at long range (a clean synthetic case at 5.0 m true distance
returned 2.8 m / 4.2° - badly wrong) - it did *not* show up in any of
the live tests above, all of which were within ~3 m of their target
break point, comfortably inside the well-supported range. Since
`BREAKPOINT_TRIGGER_DISTANCE_M` is only 0.75 m, this doesn't affect the
state machine's actual use of the estimate; it would matter for a
longer-range "start planning your approach now" use case, which isn't
what phase 4 needed.

## Wheel slip & safety hardening (phase 6)

### Architecture decision: stayed on `gz-sim-diff-drive-system`, no `ros2_control` migration

The question going in: per-wheel slip needs commanded-vs-actual
velocity for each of the 4 wheels, but DiffDrive only takes 2 commanded
speeds (left/right side). Does slip estimation require migrating the
robot's actuation to `ros2_control` with 4 independent wheel interfaces?

**No - verified, not assumed.** DiffDrive groups the 4 wheel joints into
2 commanded sides, but it still actuates all 4 joints independently
under the hood, and Gazebo's physics engine simulates each of those 4
joints' actual velocity separately - two wheels sharing a commanded
value doesn't mean they're simulated as one wheel. `gz-sim-joint-state
-publisher-system` is a second, completely independent Gazebo system
plugin that reports real per-joint position/velocity from physics
ground truth - it doesn't know or care what plugin is doing the
actuating. Confirmed this exists and does what's needed before using it
(`strings` on the installed `.so` showed `JointPosition`/`JointVelocity`
components; `turtlebot3_gazebo`'s own model.sdf uses the exact same
plugin+tags pattern for the same purpose). Added it to
`robot.urdf.xacro` alongside DiffDrive, bridged `/joint_states`
(`gz.msgs.Model` <-> `sensor_msgs/msg/JointState` - ros_gz_bridge's
documented mapping for this exact plugin's output), and that's the
entire "actual" half of the slip comparison. "Commanded" per wheel is
derived in `stability_monitor_node.py` from the same
`/model/ramp_robot/cmd_vel` Twist DiffDrive itself consumes, run
through identical diff-drive kinematics (`wheel_radius`,
`wheel_separation` - same values as the DiffDrive plugin).

Migrating to `ros2_control` would replace a working, tested actuation
plugin with a new controller-manager subsystem just to get read-only
telemetry a second, orthogonal plugin already provides for free. That
trade only makes sense if something later needs independent per-wheel
**commands** (true 4-wheel torque/velocity control) - slip estimation
never needed that, only per-wheel **readback**.

### `/wheel_slip` (`stability_monitor_node.py`)

`slip_ratio = |commanded - actual| / |commanded|` per wheel (skipped
below `MIN_COMMANDED_SPEED_RAD_S` so a near-stationary robot doesn't
report a meaningless huge ratio from sensor noise), max across the 4
wheels published as `/wheel_slip`. Left unbounded above 1.0 rather than
clamped - a wheel spinning at more than double its commanded rate is
real, useful information, not an error to hide.

**Live test:** copied `ramp_world_sharp.sdf` to
`ramp_world_sharp_lowfriction_TESTONLY.sdf` (test-only, not referenced
by any launch default - see its own header comment) and dropped Zone
B's ramp-surface friction from 0.55 to 0.12. First tried 0.02
(near-ice) - too extreme, made the robot's physics unstable (it drove
off the end of the modeled world entirely) rather than producing a
clean, observable slip signal; 0.12 was low enough for real, measurable
slip while the robot stayed controllable. Driving up the low-friction
ramp at 0.3 m/s:

```
max wheel_slip observed: 1.0 (readings up to 2.08 - see below)
```

and in `state_machine_node.py`'s log, repeatedly and correctly:

```
[ramp_state_machine]: Wheel slip 0.94 above threshold 0.5 -> HALT
[ramp_state_machine]: Wheel slip 1.92 above threshold 0.5 -> HALT
[ramp_state_machine]: Wheel slip 2.08 above threshold 0.5 -> HALT
[nav2_param_switcher]: confirmed MPPI profile "halt_mode" applied
```

(Slip ratios above 1.0 are real: with `|commanded - actual| / |commanded|`
unbounded, a wheel spinning backward relative to its commanded direction,
or more than 2x the commanded rate, scores above 1.0 - informative, not
a bug.) This confirms the full chain end to end: real low-friction
physics -> elevated `/wheel_slip` -> `state_machine_node.py`'s
`WHEEL_SLIP_HALT_THRESHOLD` (0.5, a placeholder like
`STABILITY_HALT_THRESHOLD` was before its own tuning pass - phase 8's
test matrix is the right place to tune both properly) -> HALT ->
`nav2_param_switcher_node.py` confirms `halt_mode` (vx_max=0.0) applied
on the live controller. HALT now fires from either tipping margin or
wheel slip independently (see `state_machine_node.py`'s docstring) -
different physical failure modes, checked and logged separately, either
alone sufficient.

Also worth noting as a knock-on effect, not a new bug: with real slip
happening, `/odom` (wheel-rotation dead reckoning, no contact feedback)
visibly ran ahead of the robot's true physics position during this
test - direct, independent confirmation that the wheels really were
spinning faster than the body was translating.

### Safety gap 1: `nav2_param_switcher_node.py`'s failure handling

Flagged last session, not yet fixed: what happens if `set_parameters`
fails, or `controller_server` isn't active yet? The original code
optimistically marked a profile "applied" the moment it was *sent*,
only reverting on an explicit failure *response* - a request that never
got a response at all (service not ready, so nothing was ever sent)
left the node believing whatever the previous profile was was still
correctly applied, with no retry beyond hoping the next `/ramp_state`
tick happened to re-trigger it. Concretely: a `controller_server`
crash+respawn that reset it to its params-file defaults while
`/ramp_state` stayed unchanged (no transition to re-trigger a push)
would never be noticed or corrected.

Fixed by separating two previously-conflated beliefs:
`self._target_profile` (what `/ramp_state` currently says should be
applied) and `self._confirmed_profile` (what has an explicit *success*
response). Retried on a fast timer (0.2 s) any time these disagree, not
just reactively on a state change, and even once they agree, a slow
keepalive re-push (~every 3 s) guards against exactly the silent-reset
scenario above. Every gap is logged at ERROR, and the message explicitly
says whether the controller might still be running something **faster**
than intended (the actually unsafe direction) or has merely not caught
up to a newly-safer target yet (latency, not a safety risk) -
`_gap_description()` compares the two profiles' `vx_max`.

**Live-observed:** at startup, before `controller_server` finishes
activating, this correctly logs
`MPPI profile UNCONFIRMED: target is "flat_mode" but nothing has been
confirmed applied ... do not assume it is running a safe speed limit`
every retry tick until the service comes up - loud, honest, and
retried, rather than the previous silent optimistic assumption. Once
confirmed, `confirmed MPPI profile "flat_mode" applied` repeats on the
~3 s keepalive cadence for the rest of the run, visible in every test
log in this README.

**Honest limitation, not fully closed:** this is retry + loud logging +
accurate state tracking, not a redundant independent enforcement path.
If `set_parameters` kept failing indefinitely (not tested - would need
`controller_server` to be up but its parameter service specifically
broken), the robot could still be running a stale, faster-than-intended
profile for as long as that failure persists, just never *silently*.
A fully bulletproof version would need this node (or another) to
independently zero `/cmd_vel` directly as a last resort, bypassing
Nav2 entirely - not implemented here, to avoid a second node racing the
controller/smoother for the same topic without a clear precedence rule.
Flagged below as a residual gap.

### Safety gap 2: no `/cmd_vel` watchdog

Confirmed the failure mode first: `gz-sim-diff-drive-system` has no
command-timeout parameter (checked in the phase 3 session - not in the
plugin's supported SDF tags). If whatever is publishing velocity
commands stops (crashes, or simply stops publishing), DiffDrive keeps
driving at the last received command forever.

Rather than hand-roll a custom watchdog node, used `nav2_velocity_
smoother` - already part of the Nav2 stack this project uses, and it
already has a documented `velocity_timeout` behavior built in
(publish zero once no new command has arrived for that long). Inserted
it into the pipeline: `controller_server` -> `cmd_vel_nav` ->
`velocity_smoother` -> `/model/ramp_robot/cmd_vel` (the robot's real
actuation topic). `max_velocity`/`max_accel`/etc. mirror the DiffDrive
plugin's own hard caps exactly rather than adding a third tier of
limits - this stage is a pass-through/safety-net, not another
behavior-mode limiter (that's the MPPI profiles' job).
`velocity_timeout: 0.5` (~10x the FSM's 0.1 s state-publish period and
controller's 20 Hz control loop - generous enough that normal jitter
never trips it, tight enough to bound worst-case coast distance).

**Live test:** published a sustained 0.4 m/s command on `cmd_vel_nav`,
then killed the publishing process outright (simulating commands
actually stopping, which is the real failure mode - see note below on
why this test targets that rather than literally killing
`state_machine_node.py`), while subscribed directly to
`/model/ramp_robot/cmd_vel` with per-message timestamps:

```
t=2.97s  linear.x=0.4000   <- still driving
(publisher killed at t=3.02s)
t=3.47s  linear.x=0.4000   <- held at last command for exactly ~0.45s
t=3.52s  linear.x=0.3000   <- timeout fired, decel-limited ramp down begins
t=3.57s  linear.x=0.2000
t=3.62s  linear.x=0.1000
t=3.67s  linear.x=0.0000   <- fully stopped, ~0.65s after commands stopped
```

Held the last command for ~0.45-0.5 s (matching `velocity_timeout`),
then ramped to zero respecting `max_decel` rather than a discontinuous
jolt - both correct. Without this fix (confirmed in the phase 3
session), `linear.x` would have stayed at `0.4` indefinitely.

*Note on the test target:* `state_machine_node.py` publishes
`/ramp_state` and `/cmd_vel_scale`, not velocity commands - killing it
wouldn't by itself stop `/cmd_vel` traffic (that comes from whatever is
driving `controller_server`, e.g. a Nav2 goal or a manual test
publish). The task description's framing and the real risk are the
same thing either way: *something upstream stops sending commands* -
tested that directly (killed the actual command source) rather than a
node that was never in that data path to begin with.

## Controlled descent braking (phase 7)

> **Superseded by a later pass within this same phase - see
> ["Phase 7, continued"](#phase-7-continued-fsm-stuck-state-bug-physics-correction-and-a-corrected-descent-braking-comparison)
> below.** The live side-by-side numbers in this section were measured
> against a test world where the ramp's friction (mu=0.12) was
> physically below `tan(16°)≈0.287`, the minimum needed to climb the
> slope at all - the robot could not have climbed this ramp under its
> own power, and a separate FSM bug (also fixed in that later pass)
> meant `DESCENT` may never have been genuinely reached either. Both are
> fixed below, which reruns this exact comparison (plus the
> designed-but-unbuilt `ax_max` approach) honestly. Left in place below
> as the original record, not because the numbers are still trusted.

### Control approach: adjust MPPI's vx_max, not a separate brake output

`descent_brake_node.py` runs a closed-loop controller while
`/ramp_state == DESCENT`, comparing `/wheel_slip` against a target band
and adjusting `FollowPath.vx_max` via the exact same
`/controller_server/set_parameters` mechanism `nav2_param_switcher_node.py`
already uses - not a separate cmd_vel publisher. Publishing cmd_vel
directly would make this a THIRD node able to command the robot,
racing `controller_server` and `velocity_smoother` for the same topic
with no arbitration, and would silently override or interleave with
MPPI's own path-following/obstacle-avoidance output rather than
incorporating it. Adjusting `vx_max` instead keeps MPPI as the one
actual driver - this node constrains the achievable speed envelope MPPI
operates within, the way a traction-control system constrains a car's
engine/brake authority without taking over steering.

**Coordination with `nav2_param_switcher_node.py`:** that node still
pushes DESCENT's initial `ramp_mode` baseline the instant `/ramp_state`
becomes DESCENT (a safety net if this node isn't running) and still
retries on failure, but now skips its periodic keepalive re-push
specifically while `/ramp_state == DESCENT`, so it can't periodically
clobber this node's live-adjusted `vx_max` back to the static ceiling.
Residual gap, documented in both nodes: if `descent_brake_node.py`
crashes mid-descent, nothing re-asserts a safe `vx_max` for the rest of
that DESCENT - a liveness check would close this properly and isn't
implemented here.

### Target slip band - and a real bug the live test caught

Starting point, from phase 6's actual data: its low-friction (mu=0.12)
test showed slip jumping from ~0.0 straight to 0.5-2.08, not a gradual
climb - this simulation's wheel-ground contact is plain Coulomb
friction (fixed mu, binary-ish stick/slip), not a slip-ratio-dependent
curve real tire models (and real ABS/traction-control target bands)
are tuned against. So the band here is a threshold/hysteresis pair,
not an attempt to hold a physically-optimal slip ratio that may not
exist as a stable equilibrium in this sim:
`SLIP_BRAKE_THRESHOLD = 0.25` (half of `WHEEL_SLIP_HALT_THRESHOLD`,
leaving real room to react before HALT's hard backstop),
`SLIP_RELEASE_THRESHOLD = 0.08`.

**A first implementation, live-tested, never once adjusted `vx_max`
despite slip confirmed (via a separate live probe) reaching 1.0 during
the same run.** Cause, found by testing rather than assumed: this
sim's slip events are brief spikes tied to acceleration/correction
transients, not a sustained condition - a smooth, constant-velocity
raw `cmd_vel` probe across the same low-friction surface produced
**zero** slip at all, while MPPI-driven motion (continuous small
corrections) readily produced spikes up to 2.0+. A control loop
sampling "what is slip right now" once per tick can miss a spike
entirely if it lands between two ticks, regardless of how the
threshold/step constants are tuned - the problem was never the band,
it was the sampling. Fixed by tracking the **peak** slip seen since the
last tick (updated on every `/wheel_slip` message, not just at tick
time) and resetting that peak after each tick reads it, so a spike is
caught if it happened at all during the window, independent of
control-loop rate. Confirmed live: after the fix, the same scenario
produced real, repeated `vx_max` reductions in direct response to
logged slip readings up to 1.16.

### Live side-by-side test (`world:=sharp`, ramp friction dropped to
0.12 in a test-only world copy - same one built for phase 6)

Both runs: ascend to zone C in gentle raw-`cmd_vel` bursts, rotate 180°
in place on normal-friction ground, then hand off to Nav2's
`FollowPath` action for the actual descent back through the
low-friction ramp - the measured leg has to go through MPPI (not a raw
`cmd_vel` publish) for `vx_max` to constrain anything at all. (Also
found live: `state_machine_node.py`'s `ASCENT -> TRANSITION` edge needs
a sharp pitch-**rate** spike to fire, which this gentle driving style
never produces, so the FSM never naturally reached DESCENT in this test
- a real, pre-existing, out-of-scope-for-phase-7 limitation, flagged
below. Worked around for the comparison by asserting `/ramp_state` =
DESCENT directly for the measured leg, identically in both runs, with
`state_machine_node.py`'s process killed first so there's no race - its
real HALT logic was replicated against the same live `/stability_margin`
and `/wheel_slip` topics, so HALT behavior in the test is authentic.)

| Run | distance covered | time | max slip | HALT fired |
|---|---|---|---|---|
| Baseline (`descent_brake_node.py` killed) | 5.41 m | 15.91 s | 1.00 | no |
| Brake active (floor 0.05 m/s) | 0.72 m | 11.9 s (FollowPath gave up) | 2.06 | no |
| Brake active (floor 0.12 m/s, different start point - see below) | 0.89 m | 24.43 s | 1.50 | yes, briefly |

**Honest result: this is not a clean win.** The brake controller
demonstrably *works* in the narrow sense of reacting correctly to real
slip (confirmed via direct log evidence: `slip=1.16 -> vx_max 0.13 ->
0.05 m/s`, etc.) - but in both floor configurations tested, capping
`vx_max` in response to slip made the robot cover **dramatically less
distance** than doing nothing at all, without correspondingly reducing
peak slip (2.06 and 1.50 vs. baseline's 1.00 - both *higher*). The
likely mechanism, consistent with the "peak vs. instantaneous" finding
above: since slip spikes here are tied to brief acceleration/correction
transients rather than sustained cruising speed, lowering the *steady-
-state speed ceiling* doesn't reliably prevent the next transient spike
(MPPI can still command a sharp correction within a lower ceiling), but
it reliably costs the controller speed budget it needs to make
progress - especially once a slip event throttles the ceiling down
near its floor, at which point `FollowPath` itself struggled to make
progress and gave up early. Raising `MIN_DESCENT_VX_MAX_MPS` from 0.05
to 0.12 (see the code comment) was one evidence-based attempt at a
fix; the third run above didn't cleanly re-validate it as solving the
progress problem (also starting from a different, mid-ramp position
after an ascent leg that fell short of zone C on that attempt - not a
fully apples-to-apples rerun, noted honestly rather than presented as
equivalent).

**What this suggests as a better fix, not implemented here:** capping
`ax_max`/`ay_max` (MPPI's acceleration limits) rather than `vx_max`
(the steady-state speed ceiling) would target what the evidence says
actually causes the spikes - aggressive correction transients - without
taxing the robot's ability to sustain a reasonable cruising speed
between them. That's a meaningfully different control design (limiting
*how hard* the controller can push, not *how fast* it's allowed to
settle) worth a follow-up pass rather than a same-session pivot given
how much of this session's budget the diagnosis itself took.

Two transient sim crashes were hit during this testing (a
`tf2::NoDataForExtrapolationException` in `controller_server` and
unrelated ascent-phase physics instability, both before any driving
had occurred in the affected runs) and resolved by restarting - neither
reproduced consistently and neither is attributed to this session's
code changes; noted for completeness, not as a new gap.

### HALT recovery for DESCENT (task 4) - thought through, not assumed

`state_machine_node.py`'s existing HALT recovery dropped back to
APPROACH regardless of which state HALT interrupted. That's fine for
every other state (cheaply re-detected within an IMU callback or two)
but is a real, asymmetric risk specifically for DESCENT: gravity
*assists* the failure mode on the way down instead of opposing it the
way it does climbing, so a recovery window - even a brief one - where
`flat_mode`'s much higher `vx_max` (0.8 vs. `ramp_mode`'s 0.35) is live
while the robot is still physically on the downhill slope that just
triggered the HALT is a real risk of immediately re-triggering the same
failure, not just a latency inconvenience. Fixed: HALT recovery now
goes straight back to DESCENT (not APPROACH) when that's what it
interrupted, and `descent_brake_node.py` starts that re-entry at 40% of
the `ramp_mode` ceiling rather than the full value, specifically because
the previous state was HALT (see `POST_HALT_RESTART_FRACTION`) - the
slip-band control loop is back in charge immediately, cautiously,
instead of momentarily exposed to a faster, wrong-for-the-terrain
profile.

## Phase 7, continued: FSM stuck-state bug, physics correction, and a corrected descent-braking comparison

This is a direct continuation of phase 7 above, not phase 8 - phase 7's
first pass shipped a descent-braking controller whose validation turned
out to be resting on two undiscovered FSM bugs and an unclimbable test
ramp. This section root-causes and fixes both bugs, corrects the test
world, and reruns phase 7's comparison honestly, including the
designed-but-not-yet-built `ax_max` approach. Phase 8 (the real
validation-suite phase from the original plan) comes after this
section, once descent braking and the FSM it depends on are both
actually solid.

### The bug: `ASCENT` had no guaranteed way out

Flagged going into this pass: `state_machine_node.py`'s `ASCENT ->
TRANSITION` edge fires on either perception's `breakpoint_imminent`
(phase 4) or a sharp IMU pitch-**rate** spike, ORed together - but
smooth, well-controlled driving produces neither reliably, so `ASCENT`
could get stuck indefinitely with no forward progress through the FSM.
The ask was to find the *real* cause rather than just loosening the
rate threshold. Investigated and ruled out, in order:

1. **Is the OR logic actually wired to this edge?** Yes - confirmed by
   direct logging that the elif branch containing both conditions is
   reached and evaluated on every IMU callback. Not a wiring bug.
2. **Is perception confidence too low to pass `breakpoint_imminent`'s
   gate?** No - live-logged confidence during the exact scenario that
   got stuck was 0.85-0.93, comfortably above `PERCEPTION_MIN_CONFIDENCE`
   (0.5). Not a confidence-threshold problem.
3. **Is `breakpoint_distance_m` just not decreasing?** Yes, this is the
   real cause: even with confidence well above threshold, perception's
   `breakpoint_distance_m` estimate stayed stuck near a stale ~2.55 m
   value instead of counting down as the robot actually approached the
   break point, so `breakpoint_imminent`'s distance gate never passed
   either - both halves of the OR were failing simultaneously, for
   unrelated reasons.
4. **Is the stuck estimate caused by slip-induced chassis vibration
   feeding bad geometry into the fit?** Tested directly rather than
   assumed - probed raw, high-precision `/imu` pitch during an active
   low-friction climb with real slip nearby. Pitch was **exactly**
   constant (-16.0027°, zero variance to four decimal places) while the
   perception estimate was stuck at the same time. This conclusively
   rules out sensor/attitude noise; the fault is in
   `ramp_classifier_node.py`'s geometric estimate itself (a real,
   separate bug, root-causing which is future work - not attempted
   here, see Known gaps).

**The fix, not a `TRANSITION_WINDOW_DEG` band-aid:** added a
completeness fallback edge, `ASCENT and not on_ramp -> EXIT`, mirroring
`DESCENT`'s own existing unconditional `not on_ramp -> EXIT` edge. This
guarantees `ASCENT` always has a way out once the robot has genuinely
left the ramp, independent of whether either upstream detector ever
fires - it doesn't matter *why* the OR failed, only that the FSM can no
longer get permanently stranded on the way up. Verified live, multiple
times: the FSM printed `ASCENT -> EXIT via fallback` and continued
normally after the perception-based and rate-based edges both silently
failed to fire, exactly as designed. See `state_machine_node.py`'s
`_update_state` for the full comment.

### A second, physically unrelated bug this uncovered: the old test ramp was unclimbable

Verifying the fix meant actually driving up the test ramp, which kept
appearing "stuck" (pitch never changing) even with the fix in place.
Two separate confounds, neither of which was the fix being wrong:

- **Odometry drift** (a known, already-documented phase-6 limitation)
  made position-based test logic misleading - switched the verification
  script to watch IMU pitch (ground truth) instead of `/odom.x`.
- **A real physics impossibility**: a rigid body on an incline needs
  `mu >= tan(incline angle)` to climb at all - below that, wheel torque
  produces only slip in place, never net forward motion, the same
  reason a car can't climb an icy hill. `tan(16°) ≈ 0.287`. Phase 6/7's
  test world used `mu=0.12` for the ramp surface, **below** that floor.
  This means **phase 6/7's live tests almost certainly never climbed
  this ramp under their own power at all** - whatever position they
  reported reaching zone C from was very likely odometry drift, not
  genuine progress, compounding rather than replacing the FSM bug above.
  Corrected to `mu=0.45` in `ramp_world_sharp_lowfriction_TESTONLY.sdf`
  (a first attempt at `mu=0.35`, technically above the 0.287 floor, was
  live-tested and still got the robot genuinely stuck mid-ramp for 70+
  seconds - pure static-friction math ignores acceleration, weight
  transfer, and other real dynamic effects that eat into the thin
  margin just above the theoretical minimum, so 0.45 was used instead
  for a comfortable, reliable climbing margin while still being well
  below the normal 0.55 "dusty" friction and still low enough to
  produce real, measurable slip).

### A third finding, live during the corrected re-test: `DESCENT` itself can be silently skipped

Even after both fixes above, the first corrected comparison runs showed
`/ramp_state` never actually reaching `DESCENT` during the measured
descent leg (state log showed only `ASCENT`/`TRANSITION` cycling),
which would have made `descent_brake_node.py` never activate and the
whole comparison meaningless. Cause: `DESCENT` requires being in `EXIT`
first, but HALT recovery (existing phase-7 logic, unchanged here) sends
*any* non-DESCENT pre-halt state - `EXIT` included - straight back to
`APPROACH`. A brief HALT blip right at the ramp crest (common in this
friction regime, from settling dynamics right as the robot levels out)
can land the FSM in `APPROACH` instead of `EXIT`; heading back downhill
from there gets relabeled `ASCENT` via the normal `APPROACH and on_ramp
-> ASCENT` edge, never `DESCENT`. This is documented here as a known
gap (see below) rather than fixed in this pass, since a proper fix
(e.g. remembering *which* on-ramp direction HALT interrupted, not just
that it happened) deserves its own design pass rather than a rushed
addition on top of two other FSM changes already made this session. For
the comparison test itself, isolated the variable actually under test
by reusing phase 7's "override" technique: kill
`state_machine_node.py`'s process before the measured leg and have the
test harness become the sole `/ramp_state` publisher, asserting
`DESCENT`/`HALT` directly using the real thresholds
(`STABILITY_HALT_THRESHOLD`, `WHEEL_SLIP_HALT_THRESHOLD`) against the
live `/stability_margin`/`/wheel_slip` topics - so HALT behavior in the
test is still authentic, only the state *label* is asserted rather than
derived.

### Corrected side-by-side comparison: baseline vs. `vx_max`-capping vs. `ax_max`-capping

Same methodology as phase 7 (ascend in a sustained drive watching real
IMU pitch, confirm the FSM reaches `EXIT` on its own, turn 180°, then
descend via `FollowPath` so MPPI's live parameters actually constrain
motion), but on the corrected `mu=0.45` world, with the FSM fix in
place, and with the DESCENT-override technique above for the measured
leg so the mechanism under test is guaranteed to actually be engaged:

| Run | distance | time | max slip | HALT count |
|---|---|---|---|---|
| Baseline (`descent_brake_node.py` killed) | 5.50 m | 17.01 s | 1.000 | 0 |
| `vx_max`-capping (phase 7's original approach) | 5.50 m | 76.44 s | 11.148 | **146** |
| `ax_max`/`az_max`-capping (this pass's new approach) | 5.50 m | 17.77 s | 1.000 | 0 |

**`vx_max`-capping is not just ineffective, it's actively harmful.**
146 HALTs and a 4.5x longer traverse is a feedback spiral, not noise:
cutting the speed ceiling forces the controller to decelerate *harder*
to comply (a bigger acceleration transient, exactly the thing that
causes slip on this surface - see phase 7's original root-cause
finding, which still holds), which produces more slip, which the old
logic read as "still slipping, cut `vx_max` further", demanding still
harder deceleration next tick. The mechanism meant to help was making
the actual cause worse on every iteration.

**`ax_max`-capping is statistically indistinguishable from doing
nothing at all** (17.77 s vs. 17.01 s, identical max slip) - and that's
the correct, expected result, not a null finding: on this corrected
(reliably-climbable, not razor's-edge) friction surface, slip during a
normal descent never crossed `SLIP_BRAKE_THRESHOLD` (0.25), so the
brake logic correctly did nothing. It targets the actual mechanism
(acceleration transients) directly rather than fighting the controller
for cruising speed, so unlike `vx_max`-capping it does no harm when
traction is fine, and - per the phase 7 design intent, not re-stressed
by this particular run since slip stayed low throughout - is still in
place to intervene if slip does spike. A follow-up test that
deliberately provokes a mid-descent slip spike (e.g. a brief lower-mu
patch partway down the ramp) would be needed to directly demonstrate
the intervention branch rather than only its "stays out of the way"
behavior; not done here.

### Task 4: does `/stability_margin` have the same peak-vs-instantaneous blind spot `/wheel_slip` had?

**No - verified by reading the actual event wiring, not just reasoning
about it.** The phase 7 `/wheel_slip` bug existed because
`descent_brake_node.py`'s *old* version read `self._wheel_slip`'s
last-known value from a genuinely independent, slow timer
(`CONTROL_PERIOD_SEC = 0.5`, i.e. 2 Hz) that had no relationship to how
often `/wheel_slip` messages actually arrived - a brief spike landing
between two ticks was invisible. Checked whether `/stability_margin`'s
consumer (`state_machine_node.py`'s HALT check) has the same shape:

- `stability_monitor_node.py` publishes `/stability_margin`
  **synchronously inside its own `/imu` callback** (20 Hz) - there is
  no independent timer on the publishing side at all.
- `state_machine_node.py`'s HALT check (`_update_state`, which compares
  both `/stability_margin` and `/wheel_slip` against their thresholds)
  is itself called **synchronously from its own `/imu` callback** (also
  20 Hz) - not from a separate slow poll timer either.

Both ends of this path are event-driven at the same ~20 Hz IMU cadence,
with no independent slow-poll timer anywhere in the chain - structurally
different from the bug, which required an independent, slower,
uncorrelated timer. This applies equally to `/wheel_slip` *as consumed
by the state machine's own HALT check* (separate from
`descent_brake_node.py`'s own consumption, which is already fixed via
peak-tracking). **No fix needed, and this does not change any phase 6
conclusions** - phase 6's margin/HALT numbers were never subject to the
bug pattern in the first place.

### A methodology lesson worth recording: zombie `parameter_bridge` processes

A large chunk of this phase's debugging time went into chasing
`tf2_buffer: Detected jump back in time` warnings, repeated
`controller_server` crashes (`tf2::NoDataForExtrapolationException`),
and wildly corrupted odometry that reappeared even on ostensibly
"fresh" launches. Root cause: this project's cleanup pattern
(`pkill -f "gz sim|nav2_|ramp_autonomy|robot_state_publisher"`) never
explicitly matched `parameter_bridge`, so roughly 20 zombie bridge
processes from earlier launches had silently accumulated over ~2 hours,
all still actively bridging `/clock`, `/imu`, `/odom`, `/tf`, etc. from
long-dead Gazebo instances into the live topic graph - multiple
conflicting publishers on the same topics, not a code bug. Fixed by
adding `parameter_bridge` explicitly to the kill pattern before every
relaunch, plus `ros2 daemon stop && ros2 daemon start` when
`ros2 lifecycle get` reported "Node not found" despite the process
being alive (stale DDS discovery cache). Recorded here since it cost
real time and will recur for anyone following this project's own
kill/relaunch pattern without the explicit `parameter_bridge` match.

## Phase 8: validation suite

Phase 7's testing (including its own "continued" pass above) was a
sequence of individual scenario tests, each aimed at one specific
question. Phase 8 is different in kind: a repeatable matrix run
systematically across both worlds, several ramp frictions, and two
payload configurations, using the real, fully-fixed FSM end to end
(no `/ramp_state` override anywhere in this section) - the goal is to
find out what actually happens across the space of conditions this
project claims to handle, not just re-confirm the one scenario each
bug was originally found in.

### Pass/fail criteria - declared before any cell was run

Fixed in `validation_test.py` before the matrix executed, not chosen
after seeing results:

1. **Ascent completeness**: `/ramp_state` must reach `EXIT` within a
   100 s driving budget. Not reaching it - with pitch never returning
   toward level - is scored `FAIL: DID_NOT_CLIMB`, which is the
   *correct* result for a friction below the climbing floor, not
   itself evidence of a bug.
2. **Descent reachability**: after turning around, `/ramp_state` must
   reach `DESCENT` during the `FollowPath`-driven descent leg. This is
   the direct end-to-end test of this session's HALT-recovery fix,
   run for real (no override) for the first time.
3. **HALT behavior**: HALT firing is not itself a failure - it's the
   designed safety response. 1-7 HALTs in one run is `MARGINAL`
   (system worked, but was stressed); **8 or more is scored
   `FAIL: HALT_THRASH`**, calibrated against the old `vx_max` approach
   hitting 146; 0 is clean.
4. Overall verdict per cell = the worst of the three (`FAIL` beats
   `MARGINAL` beats `PASS`).

These are deliberately coarse, mechanical criteria - they catch the
specific failure modes this project has actually hit (won't climb,
won't reach DESCENT, HALT-thrashes) but are not a substitute for
reading the actual state log and distance/pitch numbers, which are
reported alongside every verdict below rather than hidden behind the
label.

### Friction-floor characterization: theoretical vs. practical minimum

`tan(16°) ≈ 0.287` is the theoretical minimum - below it, climbing is
physically impossible regardless of driving strategy (see the "Phase 7,
continued" section). This sweep (sharp world, unloaded) asks a
different question: what's the minimum that actually works, given real
dynamic effects (weight transfer, acceleration, the discrete-timestep
physics engine) the pure static-friction number ignores?

| mu | vs. theoretical floor (0.287) | Result |
|---|---|---|
| 0.25 | below | **FAIL: DID_NOT_CLIMB** - pitch pegged at -16.0° for the entire 100 s budget, zero progress. Confirms the floor from below. |
| 0.30 | just above | **FAIL: DID_NOT_CLIMB** - notably, the FSM still progressed to `TRANSITION` (a pitch-rate spike or perception pre-emption fired), but pitch itself stayed pegged at -16.0° for the rest of the budget - the robot never actually moved further up. FSM state and physical progress disagreed. |
| 0.35 | above | **FAIL** (from the "Phase 7, continued" section's live test, not re-run here) - robot genuinely stuck mid-ramp for 70+ seconds real time, zero progress. |
| 0.45 | comfortably above | **PASS** - reliable climbing, confirmed across multiple runs this phase and last. |

**The practical climbing floor for this robot is meaningfully higher
than the theoretical one** - somewhere in (0.35, 0.45], not pinned down
further tightly here (a finer bisection between those two values wasn't
run, given time budget - a real follow-up if this number needs to be
exact rather than "safely above 0.45"). The mechanism is exactly what
the 0.35 live test already suggested: static-friction math ignores
acceleration and weight-transfer effects, which eat meaningfully into
the margin right above the theoretical minimum. Worth noting on its own:
0.30's result is a small methodological finding in itself - **the FSM's
own state can say "TRANSITION" while the robot has made zero actual
physical progress**, a reminder that `/ramp_state` alone is not a
sufficient progress signal for a test harness (or a real deployment) -
pitch/position telemetry has to be checked alongside it, which is why
every cell in this matrix reports final pitch and distance, not just
the verdict.

### Full matrix: 2 worlds x payload x friction

This matrix was rerun in full after the perception/FSM fixes elsewhere
in this phase (the `ramp_classifier_node.py` near-corner fix, the
multi-segment curved-world slope fit) to get a genuinely corrected
table, not just patch numbers in by hand. Changes from the original
run are called out explicitly in the "vs. original" column - including
where a number got *worse*, not just where it improved, per the same
honesty standard as every other table in this document.

| World | Friction | Payload | Ascent | Descent | HALTs | Overall | vs. original |
|---|---|---|---|---|---|---|---|
| sharp | 0.55 (normal) | unloaded | PASS | PASS (5.50 m) | 4 | MARGINAL | unchanged |
| sharp | 0.45 | unloaded | PASS | PASS (5.50 m) | 3 | MARGINAL | HALTs 4->3 (noise) |
| sharp | 0.55 (normal) | **loaded** (100 kg, +0.1 m CG) | **FAIL: DID_NOT_CLIMB** | N/A | 1 | **FAIL** | HALTs 0->1 (noise); same physical constraint, confirmed unaffected by this phase's fixes |
| sharp | 0.45 | **loaded** | **FAIL: DID_NOT_CLIMB** | N/A | 1 | **FAIL** | unchanged |
| curved | 0.55 (normal) | unloaded | PASS | PASS, only 1.27 m of the ~6 m leg | 5 | MARGINAL | distance 1.33->1.27 m (still short); **final pitch now 90.0° (new anomaly - none flagged in the original run)** |
| curved | 0.45 | unloaded | PASS | PASS (5.50 m) | 3 | MARGINAL | **final pitch 40.7° -> 14.0° (anomaly resolved this run)** |
| curved | 0.55 (normal) | loaded | PASS | PASS, only 1.14 m of the ~6 m leg | 4 | MARGINAL | distance 1.51->1.14 m (still short); **final pitch now -50.2° (new anomaly - none flagged in the original run)** |
| curved | 0.45 | loaded | PASS | PASS, only 1.15 m of the ~6 m leg | 3 | MARGINAL | distance ~unchanged (1.13->1.15 m); final pitch -50.2° - **identical value to the original run** |

None of the coarse PASS/MARGINAL/FAIL verdicts changed. What *did*
change, honestly: HALT counts shifted by ±1 in a couple of cells (run-
to-run timing noise at the crest, already a known pattern from earlier
phases, not a regression), and - more importantly - *which* curved-
world cells show the anomalous-pitch symptom shifted around (one
resolved, two new ones appeared, one reproduced exactly). Root-caused
below: this is expected given the mechanism (a real tip-over event
whose exact trigger point depends on accumulated lateral drift, which
varies slightly run to run), not evidence the fixes made anything
worse - **every curved cell still stalls at a short descent distance
both before and after this phase's fixes**, which is the actual,
stable signal, and is exactly why task 3 concluded this is a separate,
unresolved issue rather than a symptom of the perception/FSM bugs this
phase fixed.

Raw logs for every cell (full state-transition timelines), both the
original run and this corrected rerun, are in
`validation/phase8/matrix_logs/` and
`validation/phase8/matrix_corrected_logs/` respectively, alongside the
test harness itself (`validation_test.py`, `run_matrix.sh`) so the
whole matrix is re-runnable, not just its numbers transcribed here.

**The table above is preserved as-is: a historical, pre-EKF,
odometry-only record.** It predates the EKF fusion work (see the
unified "lateral drift under any turning motion" entry in Known Gaps
below) and - as that work went on to establish - `validation_test.py`
never polled Gazebo ground truth anywhere in its own logic, only wheel
odometry and `/ramp_state`. That is a real, confirmed gap in what this
table actually verified, not a hypothetical one. Rather than silently
overwrite it, the corrected rerun below is kept alongside it as its own
labeled section, because the discrepancy between the two is itself a
meaningful finding.

### Full matrix, corrected: EKF-fed controller + Gazebo ground-truth polling

Same 7 cells (the `sharp_unloaded_mu045` 8th row above came from a
since-not-preserved script, `halt_recovery_test.py`, and could not be
reproduced identically - excluded here rather than approximated),
rerun against `validation_test_ekf_groundtruth.py`
(`validation/phase8/validation_test_ekf_groundtruth.py`): same ascend
-> turn -> `FollowPath` descend structure as the original script
(preserved unmodified alongside this one), launched through the same
`ramp_autonomy_bringup.launch.py` - now with `ekf_node` live and
`controller_server` reading `/odometry/filtered` automatically, no
launch changes needed for that part - plus a new criterion that polls
Gazebo's own ground-truth model pose throughout the descent leg and
fails the cell if the robot's real lateral (Y) position ever exceeds
the ramp/zone panel's own half-width (1.5 m - both worlds' panels are
confirmed 3.0 m wide by inspection), the same mechanism already proven
to cause real tip-overs in this project's history.

| World | Friction | Payload | Ascent | Descent (FSM) | HALTs | Ground truth | Overall | vs. pre-EKF baseline |
|---|---|---|---|---|---|---|---|---|
| sharp | normal | unloaded | PASS | PASS | 4 | PASS (max\|y\|=0.08 m, n=16) | MARGINAL | unchanged |
| sharp | normal | **loaded** | **FAIL: DID_NOT_CLIMB** | N/A | 1 | N/A | **FAIL** | unchanged |
| sharp | 0.45 | **loaded** | **FAIL: DID_NOT_CLIMB** | N/A | 0 | N/A | **FAIL** | unchanged |
| curved | normal | unloaded | PASS | PASS | 1 | **FAIL: edge departure (max\|y\|=2.71 m, n=55)** | **FAIL** | **CHANGED: MARGINAL -> FAIL** |
| curved | 0.45 | unloaded | PASS | PASS | 1 | PASS (max\|y\|=0.39 m, n=90) | MARGINAL | unchanged (now genuinely ground-truth-confirmed safe, not just odometry-plausible) |
| curved | normal | **loaded** | PASS | PASS | **8** | **FAIL: edge departure (max\|y\|=2.33 m, n=57)** | **FAIL** | **CHANGED: MARGINAL -> FAIL** |
| curved | 0.45 | **loaded** | PASS | PASS | **8** | PASS (max\|y\|=0.98 m, n=16) | **FAIL: HALT_THRASH** | **CHANGED: MARGINAL -> FAIL** |

**3 of 7 cells changed verdict - all three in the FAIL direction, none
improved.** Reported plainly, not smoothed over:

1. **`curved/normal/unloaded` and `curved/normal/loaded` both flip from
   MARGINAL to FAIL purely because ground truth catches something the
   original odometry-only check structurally could not**: the FSM still
   reports `descent_verdict: PASS` (`DESCENT` was reached, nothing
   regressed to `ASCENT`/`APPROACH`) in both, exactly as the original
   table showed - but Gazebo's own model pose shows the robot's real Y
   position reaching 2.71 m and 2.33 m respectively, well past the
   1.5 m panel half-width. The robot's wheels left the ramp panel
   surface in both cells; the original matrix had no way to see that,
   for the same structural reason `construction_site.sdf`'s own
   turnaround failures went undetected until ground truth was checked
   there too. This is the SAME lateral-drift mechanism as the original
   "curved-world descent chatter" finding, just now caught directly
   instead of inferred from anomalous pitch readings after the fact.
2. **Both curved *loaded* cells newly hit `HALT_THRASH` (8 HALTs each,
   vs. 4 and 3 in the pre-EKF baseline) - a genuine, unexplained
   behavioral change, reported honestly rather than assumed benign.**
   Unloaded curved cells stayed low (1 HALT each, same ballpark as
   before); only the loaded cells jumped to the thrash threshold. One
   plausible mechanism, NOT yet confirmed: `controller_server` now
   plans against a materially different (more accurate) pose estimate
   than before, and the loaded payload's already-marginal stability
   margin may be triggering genuinely more borderline pitch-rate events
   under that more accurate feedback, rather than fewer. This has not
   been root-caused and should not be assumed harmless - flagged as an
   open finding, not patched or explained away here.
3. **`descent_leg_distance_m` values in this rerun (0.03-2.94 m) read
   much shorter than the pre-EKF baseline's (1.14-5.50 m) across every
   cell, including ones with an otherwise-clean ground-truth verdict.**
   This metric is NOT reliably comparable between the two scripts: both
   compute it from the test script's own raw `/odom` reads, used
   internally to decide when to stop polling and declare the leg
   "done" - and raw wheel odometry is exactly the signal now known to
   be unreliable under turning. A short value here does not by itself
   mean the robot made less real progress; it may just mean the test
   script's own (odometry-based) stopping condition fired earlier. Not
   investigated further per the same "report, don't keep patching"
   judgment call as the rest of this session - the ground-truth and
   HALT-count criteria above are the reliable signals from this rerun.

Every cell whose ascent already failed (both sharp loaded cells) or
whose ground truth and HALT counts were already clean (sharp unloaded,
curved 0.45 unloaded) is unaffected by the EKF fix, as expected - it
targets turning-induced lateral error specifically, and those cells
either never turn (ascent failures) or were never drifting in the first
place.

Full logs (launch + result, every cell): `validation/phase8/
matrix_ekf_groundtruth_logs/`; test harness:
`validation/phase8/validation_test_ekf_groundtruth.py`,
`validation/phase8/run_matrix_ekf.sh`.

### Full matrix, corrected: crosstrack fix applied (the trusted matrix going forward)

> **Correction (final structural experiment, `validation/phase8/path_structural/README.md`):** the `fillet_top_1/2/3` "seam stumble" and "unchecked
> crosstrack" explanations below were never tested as causes and are superseded (the departure comes from the harness turning the robot on the curved crest);
> and every ground-truth PASS on a normal-friction curved cell means "did not leave the panel before the leg ended", not "descended" (travel 0.1-2 m of 6 m).

The two ground-truth-caught failures above were root-caused with live
instrumentation (`validation/phase8/crosstrack_diagnosis/README.md` has
the full story - summarized in Known Gaps below). Short version: MPPI's
`PathAlignCritic` (the lateral-tracking critic) was structurally
disabled for this project's entire descent leg, on every prior run -
its own source only activates once the robot has advanced
`offset_from_furthest` (default 20) points into the path, and this
project's ~6 m descent path only has ~15 points total, so the critic
never fired even once. Three tuning attempts that raised its weight/
power without knowing this had no real effect (there was no active
critic to tune); fixing `offset_from_furthest` itself (dropped to 3 for
a new `descent_mode` profile, split off from `ramp_mode` so ASCENT's
already-proven-clean behavior is untouched) is the fix that actually
matters. HALT-thrash was root-caused to the same underlying mechanism:
genuine, previously-invisible physical instability at a confirmed real
geometric pinch point (`ramp_world_curved.sdf`'s `fillet_top_1/2/3`
seam cluster, X=7.6-8.3 m - not a threshold or fusion-timing bug, both
explicitly ruled out with evidence).

Rerunning the full 7-cell matrix with the fix live:

| World | Friction | Payload | Ascent | Descent | HALTs | Ground truth | Overall | vs. crosstrack-fix baseline (2 tables up) |
|---|---|---|---|---|---|---|---|---|
| sharp | normal | unloaded | PASS | PASS | 0 | PASS (max\|y\|=0.09 m) | **PASS** | **IMPROVED: MARGINAL -> PASS** (HALTs 4->0) |
| sharp | normal | loaded | FAIL: DID_NOT_CLIMB | N/A | 0 | N/A | FAIL | unchanged (unrelated physical constraint) |
| sharp | 0.45 | loaded | FAIL: DID_NOT_CLIMB | N/A | 0 | N/A | FAIL | unchanged |
| curved | normal | unloaded | PASS | PASS | 4 | **FAIL: edge departure (max\|y\|=2.68 m)** | **FAIL** | still FAIL - error reduced only slightly (2.71->2.68 m, still past 1.5 m); HALTs got worse (1->4) |
| curved | 0.45 | unloaded | PASS | PASS | 1 | PASS (max\|y\|=0.36 m) | MARGINAL | essentially unchanged |
| curved | normal | **loaded** | PASS | PASS | 4 | PASS (max\|y\|=0.97 m) | **MARGINAL** | **IMPROVED: FAIL -> MARGINAL** (ground truth now clean; HALTs halved 8->4) |
| curved | 0.45 | **loaded** | PASS | PASS | 9 | PASS (max\|y\|=0.93 m) | **FAIL: HALT_THRASH** | still FAIL - ground truth clean, but HALT count got slightly worse (8->9) |

**This is not a fully clean matrix, and it isn't reported as one.** 2 of
7 cells genuinely improved (one to a full PASS); 2 remain FAIL for the
same reason as before, just measured more precisely now; 2 are
unaffected as expected (the sharp loaded ascent failures, an unrelated
physical constraint); 1 is unchanged at MARGINAL. The two cells still
failing (`curved/normal/unloaded` on ground truth,
`curved/0.45/loaded` on HALT count) are consistent with the second,
separately-confirmed mechanism in `crosstrack_diagnosis/README.md`: a
real physical stumble at the curved world's own fillet-top seam
cluster, which MPPI parameter tuning cannot fully prevent or instantly
correct from - fixing it further would need a different category of
change (a stall/stumble detector, or smoothing the world's own seam
geometry), out of scope here per the same "propose, don't necessarily
implement" judgment already applied to the construction-site stall
finding.

**This is the matrix to trust going forward** - not because every cell
passes, but because every verdict on it (including the two persisting
FAILs) is now backed by ground truth and a confirmed mechanism, not
odometry that turned out to be unreliable. Full logs: `validation/
phase8/matrix_ekf_crosstrackfix_logs/`; test harness (unchanged from
the previous table, just rerun): `validation/phase8/
validation_test_ekf_groundtruth.py`; matrix runner: `validation/phase8/
run_matrix_ekf_crosstrackfix.sh`.

**Two real, non-cherry-picked findings this matrix caught - both since
root-caused (see the two subsections below), reported plainly rather
than smoothed into the clean-looking MARGINAL/PASS labels above:**

1. **The loaded payload (100 kg, +0.1 m forward CG offset) cannot
   climb the sharp-world ramp at all, even at normal (0.55) friction -
   comfortably above the climbing floor for the unloaded robot.** Both
   sharp-world loaded cells failed identically, in both the original
   run and this corrected rerun: FSM reached `TRANSITION` (an early
   pitch-rate/perception trigger, same as the 0.30-friction finding
   above) but pitch stayed pegged at -16.0° for the rest of the budget.
   **Root-caused: see "Root-causing the loaded-payload climb failure"
   below - this is a genuine physical/design constraint (a rigid-body
   ground-clearance interference at the sharp break point), not a
   software or tuning bug**, so it was documented rather than patched.
2. **Curved-world loaded climbs successfully in both frictions**
   (unlike sharp-world loaded, which fails both) - so the loaded
   climbing failure above is not universal, it's specific to the sharp
   world's single abrupt break point. Separately, **curved-world
   descent legs consistently stall well short of their ~6 m target
   (1.1-1.3 m in all four curved cells, both before and after this
   phase's other fixes) with anomalous final pitch readings appearing
   in most runs** (three of four in the rerun above, up to 90°/-50°
   depending on the run). **Root-caused: see "Root-causing the
   curved-world descent chatter" below - a genuine physical tip-over
   from lateral drift carrying the robot off the edge of the curved
   world's narrow fillet panels, confirmed via Gazebo's own
   ground-truth pose and orientation, not a perception, FSM, or
   HALT-recovery bug** (verified by ruling those out first: rerunning
   after the perception/FSM fixes left this essentially unchanged).
   Also not fixed, per the same reasoning as finding 1 - see that
   subsection for what actually would fix it.

**One clear positive finding, stated as plainly as the two above:**
the HALT-recovery fix held up robustly even under far heavier HALT
churn than it was designed against, in both the original run and this
corrected rerun - the curved-world cells fired 3-6 HALTs each (some
right in the middle of the chattery EXIT/DESCENT region, some during
the genuine tip-over event described below) and in every single case
recovery landed back on `EXIT` or `DESCENT`, never once falling back to
`ASCENT`/`APPROACH` the way the pre-fix code would have.

### Root-causing the loaded-payload climb failure

The matrix flagged this as a real, unexplained gap; a follow-up
session instrumented a single live run (loaded, sharp world, normal
0.55 friction - the exact failing config) to get a real causal story
instead of guessing. Three specific questions were asked up front,
mirroring the three places this could plausibly live:

**1. Physical/tip-over limit - is `/stability_margin` the cause?**
No. Logged continuously alongside everything else below,
`/stability_margin` stayed in the 0.46-0.74 range for the entire
attempt - comfortably above `STABILITY_HALT_THRESHOLD` (0.15) the
whole time, and HALT never fired even once. This isn't a tipping
failure, and the margin calculation isn't doing anything wrong here -
it's simply answering a question ("is the robot about to tip over?")
that isn't the one this failure is actually about.

**2. Control/tuning limit - is MPPI's `ramp_mode` profile the cause?**
No, for a more basic reason than tuning: the ascent leg in every phase
8 cell drives via a raw, constant `/cmd_vel_nav` publish (0.25 m/s),
not through `FollowPath`/MPPI at all - `ramp_mode`'s `vx_max`/`wz_max`
never enter the picture during ascent. The one real downstream stage,
`nav2_velocity_smoother`, has generous limits (1.2 m/s max, 2.0 m/s^2
accel) that a steady 0.25 m/s command clears in about 0.1 s and holds
thereafter - confirmed by `odom`'s reported linear velocity sitting at
a rock-steady 0.250 m/s (matching the command exactly) for the entire
run, including the stuck portion. Nothing in the commanded-velocity
path is the bottleneck.

**3. Perception/FSM limit - does the FSM even reach `ASCENT`?**
No, this is also ruled out - directly, not inferred: the logged
`/ramp_state` timeline shows `APPROACH -> ASCENT` at t=12.65s and even
`ASCENT -> TRANSITION` at t=22.67s, a completely normal progression.
(That `TRANSITION` firing this early while the robot is still stuck
mid-ramp is itself a symptom of the real cause below, not a separate
bug - the physical jolt of hitting an obstruction plausibly registers
as the pitch-rate spike that triggers it, the same way friction-floor
cell `0.30` showed FSM state advancing independent of real physical
progress.)

**So what is it?** The commanded-vs-achieved velocity comparison
`/wheel_slip` is built for (per-wheel commanded vs. actual *rotation
rate*) read exactly **0.000 for the entire run, including the stuck
portion** - the wheels were genuinely spinning at the commanded rate
the whole time, going by both `/wheel_slip` and `/odom`'s own
wheel-rotation-derived velocity (steady 0.250 m/s throughout).
Meanwhile `/odom`'s *position* climbed smoothly past 11 m over the
45 s run. Querying Gazebo's own ground-truth world pose directly
(`gz topic -e -t .../dynamic_pose/info`, bypassing ROS odometry
entirely) told a completely different story: **the robot's real world
X position climbed normally from 1.0 m to 7.12 m over the first ~30 s,
then froze - bit-for-bit identical (7.1190981...) across every sample
from t=32s to t=49s, a dead stop with zero drift.** Real position and
`/odom`'s reported position diverged by nearly 4 m. The robot's
orientation at that frozen position was a clean, pure -16.0° pitch
with zero roll and zero yaw contamination - not cocked or twisted,
which rules out an asymmetric snag (like a guard-rail clip: the rails
sit at y=±1.55 m, over a meter clear of the robot's ±0.2 m body/payload
width, so they were never in play anyway).

Putting the geometry together: the robot froze at world x=7.119 m -
close enough to the sharp break point (x=7.5 m) that, accounting for
`base_link`'s own half-length (0.4 m) and the payload's forward offset,
**the chassis's own front face is essentially sitting right at x≈7.5,
exactly where `zone_b_ramp`'s box collision geometry meets
`zone_c_exit`'s** - two independently-modeled, unfilleted boxes
forming a real dihedral ridge, not a smooth surface. Combined with
`base_link`'s own URDF geometry giving this chassis essentially **zero
ground clearance by design** (`base_footprint_joint`'s wheel_radius
offset, 0.12 m, almost exactly cancels `base_link`'s own half-height,
0.125 m - the chassis underside sits within 5 mm of wheel-bottom
height), this is a textbook **rigid-body ramp-breakover-angle
problem**: a low-clearance chassis crossing a sharp convex grade
change can physically hang up on the corner, wheels spinning
uselessly against a surface that isn't the ground. `/wheel_slip`
cannot see this failure mode at all - it only ever compares a wheel's
own commanded and actual rotation rate, never checks whether that
rotation is actually producing chassis displacement, so a wheel that's
spinning "correctly" while the whole vehicle is jammed on something
else entirely reads as perfectly healthy.

**Why does the unloaded robot clear the same corner fine, and why does
the (also loaded) curved world work?** Two independent, mutually
reinforcing pieces of evidence, not just one: the unloaded robot
apparently has just enough momentum/traction margin to power through
this marginal geometric interference, which the payload's extra 100 kg
(pressed down through a forward-shifted, elevated CG right over the
front of the chassis) tips from "survivable" to "actually stuck".
Separately, and consistent with the mechanism rather than coincidental,
**the curved world's loaded cells both climbed successfully** - its
break points are filleted (three ~5.3° sub-segments over ~0.9 m instead
of one instantaneous 16° jump), so there's no single sharp ridge for a
low-clearance chassis to catch on in the first place, independent of
payload.

**Conclusion: this is a genuine physical/design constraint, not a
software or tuning bug - not fixed, per the instruction that prompted
this investigation.** No FSM, controller, or physics parameter was
changed as a result of this diagnosis; forcing a workaround (e.g.
inflating friction further, or padding a timeout) would hide a real
mechanical limitation rather than fix it. What would actually need to
change, in rough order of how directly each addresses the mechanism
found:
- **Real ground clearance in the chassis design** - `base_link`'s
  current near-zero clearance (wheel radius almost exactly canceling
  the chassis's own half-height) is the most direct cause; a chassis
  with genuine clearance between its underside and the wheel-bottom
  plane would clear the same corner regardless of payload.
- **A lower/more centered payload mount** - less height above
  `base_link` and less forward offset both reduce how hard the payload
  presses the chassis's front into the corner during the interference
  window.
- **A wider wheelbase relative to chassis length**, which improves the
  real breakover angle a rigid chassis can clear.
- **Filleted (not sharp) ramp transitions on real deployment routes**
  - the curved world's success shows this sidesteps the problem
    entirely, independent of chassis geometry or payload; most real
    facility ramps use a transition plate for exactly this reason.
- **A lower maximum rated payload specifically for sharp-transition
  ramp routes**, if the chassis/mounting geometry above isn't changed -
  a capacity limit rather than a capability claim.

Raw evidence for this diagnosis (the combined margin/slip/state/
velocity timeline, and the independent ground-truth Gazebo pose trace
that caught the real-vs-odometry divergence) is in
`validation/phase8/loaded_payload_diagnosis/`, alongside the scripts
that produced it.

### Root-causing the curved-world descent chatter

The other open phase-8 finding was the curved world's short descent
legs, `EXIT`/`DESCENT` chatter, and anomalous final pitch readings. The
task framing for this investigation was explicit: don't assume it's a
downstream symptom of the perception/FSM bugs above - verify by
rerunning the exact failing cells after those fixes land, and only
dig further into whatever doesn't resolve on its own.

**Step 1 - did the perception and FSM fixes resolve it on their own?**
No. Rerunning the same four curved-world matrix cells after both the
`ramp_classifier_node.py` fix and the multi-segment slope fit landed
showed essentially unchanged behavior: descent distances stayed short
(1.27 m, 1.43 m, 1.18 m vs. the original 1.33 m, 1.51 m, 1.13 m - noise-
level differences, not a resolution), and one cell's pitch anomaly
(-50.2°) reproduced at the exact same value, while another cell that
had no anomaly before now showed a full 90°. This is a separate issue,
not a symptom of tasks 1/2 - confirmed by ruling it out, not assumed.

**Step 2 - live instrumentation.** Ascended, turned, and descended
exactly as the matrix does, but logged IMU pitch, `/wheel_slip`,
`/stability_margin`, AND Gazebo's own ground-truth world pose (full
position and orientation quaternion, not just IMU-derived pitch) on
one shared timeline throughout the descent leg. The result was
unambiguous:

| t (s) | state | pitch (IMU) | margin | gz_z (height) | gz_y | orientation |
|---|---|---|---|---|---|---|
| 1.6 | EXIT | 1.4° | 0.99 | 1.13 m | 0.65 | clean single-axis pitch |
| 2.4 | DESCENT | 8.0° | 0.94 | 1.05 m | 1.40 | clean single-axis pitch |
| 3.2 | HALT | 54.9° | 0.37 | 0.54 m | 1.62 | quaternion no longer a clean pitch |
| 4.0+ | HALT (latched) | 63.4° | 0.11 | 0.47 m | 1.51 | frozen, identical every sample for 12+ s |

Between t=2.4 and t=3.2, the robot's real height dropped by roughly a
metre and its orientation stopped being a simple pitch rotation - a
genuine tip-over/fall, confirmed via ground truth, not an estimation
artifact (the anomalous pitch readings are the IMU honestly reporting
a robot that really is no longer sitting upright). Critically, this
happens at the exact moment `gz_y` crosses past 1.5 m - the edge of the
curved world's fillet panels, which are 3.0 m wide (±1.5 m from
centreline) - the same width as every other zone in both worlds. The
robot's Y position had already drifted from 0.65 m to 1.40 m over the
two ticks before that, i.e. it was heading for the edge before it ever
reached the fillet.

**Root cause: lateral drift during the `FollowPath`-tracked descent
carries the robot off the side of the curved world's narrow fillet
panels, tipping it over** - not a perception, FSM, or HALT-recovery
bug. The FSM's response (HALT firing and staying latched while margin
is genuinely low) is correct behavior given a real instability, which
is exactly why every affected cell still "passes" this project's
coarse criteria (`DESCENT` reached, never regressed to
`ASCENT`/`APPROACH`) despite the underlying problem. The descent path
these tests (and currently this whole project - there's no
`planner_server` yet, see Known Gaps) send is a straight line in X with
no active correction for cross-track (Y) error; some accumulated
lateral drift under MPPI tracking is unsurprising on its own, and the
sharp world - a single continuous surface, same 3.0 m width throughout
- tolerates it without incident. The plausible (not independently
proven beyond this one instrumented run) reason curved specifically
fails: crossing several fillet-panel seams in quick succession (three
~0.3 m segments at slightly different angles, over less than a metre)
plausibly imparts small extra lateral/rotational perturbations at each
seam that a single sharp corner doesn't, compounding the drift enough
to reach the edge.

**Not fixed**, for the same reason the loaded-payload finding above
wasn't patched around: this is a real robustness limitation, not a
software defect with an obvious, verifiable one-line fix, and forcing
something without verifying it actually helps would just hide a real
gap. What would actually address it: stronger lateral-tracking cost
weighting in MPPI's own critics (a tuning question this project hasn't
done - see the `ramp_mode` TODO in `ramp_mppi_params.yaml`), a real
path source with active cross-track correction once `planner_server`
exists, or simply wider fillet panels in the world model if the goal
is a more forgiving test surface rather than a more precise controller.

Raw evidence (the ground-truth pose/orientation timeline and the script
that produced it) is in
`validation/phase8/curved_descent_diagnosis/`.

### Was this validation-suite phase affected by the stability-margin sampling question?

No - covered already in the "Phase 7, continued" section's task-4
analysis (`/stability_margin`'s publisher and consumer are both
IMU-callback-driven at the same ~20 Hz rate, no independent slow-poll
timer), and nothing in this phase's matrix run changes that: the same
`state_machine_node.py` and `stability_monitor_node.py` ran unmodified
throughout.

## Phase 8, continued: closing the LiDAR and cmd_vel-enforcement gaps

Two remaining sim-scope items from the Known Gaps list below, unrelated
to each other, closed in the same follow-up pass. Reported separately
since they touch completely different parts of the stack.

### 1. LiDAR obstacle-coverage gap - fixed with a second sensor

Phase 4 rolled the robot's single 2D LiDAR 90° to get the fore-aft
ground-profile fan `ramp_classifier_node.py` needs (see that node's own
docstring) - a real, unavoidable trade-off documented at the time: a
single 2D LiDAR physically cannot serve both a rolled ground-profiling
role and a level 360° obstacle-detection role at once, so rolling it
cost the robot its side/rear obstacle coverage entirely.

Since this is simulation, the fix is a second, independent sensor, not
a compromise: `robot.urdf.xacro` now has `obstacle_lidar_link`, a
level-mounted (unrolled) 2D LiDAR on its own topic (`/scan_360`),
completely separate from `lidar_link`/`/scan` (unchanged, still
`ramp_classifier_node.py`'s sensor). Mounted at `z=0.5` in `base_link`'s
frame specifically to clear the payload box's own top (`z=0.425`,
present at its full 0.5x0.4x0.3 m size even when `payload_mass=0.0` -
only its mass/inertia change with that arg, not its geometry) - a level
scan plane above the payload never intersects it, so obstacle coverage
stays full 360° regardless of payload configuration. `nav2_params.yaml`'s
`local_costmap.obstacle_layer` now reads `/scan_360` instead of `/scan`.

**Verified live**, not just wired up: a test-only world
(`ramp_world_sharp_obstacles_TESTONLY.sdf`, not referenced by any
launch default) adds two 1.2 m-tall obstacle boxes near the robot's
spawn pose - one directly behind it, one directly beside it, both
outside `/scan`'s forward-only detection cone. `/scan_360` detected
both at their exact true bearing and range (behind: bearing ~180°,
range 0.64 m; side: bearing ~-90°, range 0.85 m, both matching the
world file's geometry precisely), and `/local_costmap/costmap` showed
631 real occupied cells - confirming Nav2's `ObstacleLayer` is
genuinely consuming the new sensor, not just that the raw scan looks
right in isolation. Simultaneously, `/ramp_perception` (the *other*
sensor) reported slope_deg 17.68° / breakpoint_distance_m 2.55 m /
confidence 0.97 - matching this project's long-established sharp-world
baseline exactly, confirming ramp detection is completely unaffected
by the new sensor's addition. Full results and the test world's own
comments: `validation/phase8/obstacle_lidar/`.

An earlier live check caught a bug in the *test world itself*, not the
sensor: the first version's obstacle boxes were only 0.6 m tall and sat
entirely below `obstacle_lidar_link`'s ~0.85 m scan plane, so they were
(correctly) invisible - the same failure mode a too-short real obstacle
would have below a real 2D LiDAR's mount height, not a defect in the
new sensor. Fixed by making the test obstacles taller, not by changing
the sensor - documented here since it's a real "know your test world's
own geometry" lesson, not because it reflects on the actual fix.

### 2. nav2_param_switcher_node.py's safety-enforcement gap - closed with an independent, ramped stop

The gap, as documented since phase 6: `nav2_param_switcher_node.py`'s
HALT handling is retry + logging + accurate state tracking, not a
redundant *independent* enforcement path - if
`/controller_server/set_parameters` silently failed or lagged,
`controller_server` could keep running a faster-than-safe MPPI profile
for one retry-interval window before that was ever caught.

**Design: a single mandatory gate, not a second publisher.** An earlier
session already considered and rejected publishing directly to the
robot's actuation topic (`/model/ramp_robot/cmd_vel`) from a new node -
that would make it a third, unarbitrated publisher racing
`controller_server`/`velocity_smoother` for the same topic (see
`descent_brake_node.py`'s own docstring for the same reasoning applied
elsewhere). `cmd_vel_watchdog_node.py` avoids that by being inserted
*inline*, not in parallel:

```
controller_server -> cmd_vel_nav -> velocity_smoother
    -> cmd_vel_gated_in -> [cmd_vel_watchdog_node.py] -> /model/ramp_robot/cmd_vel
```

`velocity_smoother`'s output (fixed name `cmd_vel_smoothed`) is now
remapped to `cmd_vel_gated_in` instead of straight to the robot -
`cmd_vel_watchdog_node.py` is the *only* thing that ever publishes the
real actuation topic. There is exactly one writer on it at all times
(confirmed live via `ros2 topic info -v`, publisher count 1), so there
is nothing left to race - not "two nodes, arbitrated somehow," but
"one node, structurally." While `/ramp_state != HALT` it passes
`cmd_vel_gated_in` straight through unchanged; while `== HALT` it
forces the command toward zero, independent of whatever MPPI profile
is actually still running or whether the parameter push ever landed -
it never touches `/controller_server/set_parameters` at all.

**A real regression this caught in its own first version, before
shipping it.** The first implementation forced an instant, exact-zero
Twist the moment HALT was declared. Live A/B testing (repeated full
ascend/crest-cross runs, `world:=sharp_lowfriction_TESTONLY`) found
this measurably worse than no watchdog at all in a real fraction of
runs: old pipeline HALT counts across 3 runs were 4/1/0 (matching this
project's established baseline), but the instant-zero watchdog produced
35/1/1 - one run with severe, rapid HALT-thrashing the old pipeline
never showed. The likely mechanism: teleporting the commanded velocity
to exactly zero in one tick is an effectively-infinite deceleration
spike, physically harsher than anything MPPI/`velocity_smoother` would
ever command on their own (both respect real accel/decel limits) - on
a marginal-friction surface, that spike is plausibly enough on its own
to trigger a fresh wheel-slip or margin event right as the robot tries
to recover from the last one. In that one respect the "fix" was
briefly worse than the gap it was meant to close.

Fixed by ramping the command to zero instead of teleporting - at
`robot.urdf.xacro`'s own DiffDrive plugin rating exactly (2.0 m/s²
linear, 3.0 rad/s² angular), not an invented slower "gentle" limit, so
it still stops the robot as fast as the platform is physically rated
to stop, just not faster-than-physical via an instant discontinuity.
Re-run with this fix: 3 more full ascend/crest-cross trials gave
1/4/0 HALTs - back in line with the established baseline, no further
thrashing observed. Full A/B numbers: `validation/phase8/
cmd_vel_watchdog/regression_ab_test_summary.txt`.

**Verified live: stuck reconfiguration path, real upstream motion,
independent stop.** Killed `nav2_param_switcher_node.py` and
`state_machine_node.py` (so the reconfiguration path can never recover
and this script has sole control of `/ramp_state` - the same override
technique used elsewhere in this project's test history), forced
`FollowPath.vx_max=0.8` directly, and sent a real `FollowPath` goal so
MPPI was genuinely commanding non-zero velocity (confirmed:
`cmd_vel_gated_in` climbing steadily, reaching 0.086 m/s and still
rising). Then declared HALT. Result: the actuation topic ramped
cleanly to exactly zero in **121.4 ms** (clean ~0.04 m/s steps every
20 ms tick, matching the 2.0 m/s² rate exactly) and *stayed* at zero
for the rest of the test, while `cmd_vel_gated_in` kept climbing past
0.24 m/s throughout, since the reconfiguration path was deliberately
left permanently dead - direct, live proof the backstop works
regardless of that path's state. Full timeline:
`validation/phase8/cmd_vel_watchdog/stuck_reconfiguration_test_result.log`.

A pre-existing limit this does *not* newly solve, documented rather
than hidden: if `cmd_vel_watchdog_node.py` itself stops running,
nothing publishes the actuation topic at all - the robot holds its last
command rather than actively stopping. That is a general single-point-
-of-failure risk any inline gate carries (`velocity_smoother` already
carried the identical risk before this fix, unchanged by it), not
something this specific fix promises to solve; a real answer needs a
heartbeat/liveness pattern, out of scope here.

## Final corrected matrix (2026-09-20, after the crest-geometry fix) - supersedes every earlier verdict

Procedure: corrected turn (default: ground-truth-measured 2.4 m drive onto the flat platform, closed-loop turn on the EKF yaw) + ramp geometry fixed (the 0.05 m crest / foot steps removed, `validation/phase8/crest_freeze/README.md`) + unscoped fixed detector (48625b76) + ground-truth ascent gate.
One run per cell on a quiet host (min RTF 0.994-0.999). Verdict (HALTs; ground-truth max |y|). "Old" = the crest open-loop turn on the original geometry (prior / old det / fixed / scoped detectors; robot never descended on curved-normal / sharp cells, loaded cells stuck at the crest);
"previous corrected" = the 2026-09-19 corrected-turn matrices on the ORIGINAL geometry (run A / run B).

| Cell | Old verdicts | Previous corrected (original geometry) | **FINAL** |
|---|---|---|---|
| sharp / normal / unloaded | PASS / MARGINAL / MARGINAL / MARGINAL | MARGINAL (4) / FAIL HALT_THRASH (9) | **MARGINAL** (6; 0.03 m) - full descent, 6/6 in the regression sample |
| sharp / normal / loaded | FAIL DID_NOT_CLIMB (all) | FAIL DID_NOT_CLIMB (7) / (13) | **FAIL: HALT_THRASH** (8; 0.02 m) - climbs and descends fully (12/12 in the sample; HALT median 2, max 6): the FAIL is the HALT-count rule on one run |
| sharp / 0.45 / loaded | FAIL DID_NOT_CLIMB (all) | FAIL DID_NOT_CLIMB (12) / (14) | **MARGINAL** (4; 0.02 m) - 12/12 in the sample |
| curved / normal / unloaded | FAIL edge dep. / FAIL HALT_THRASH / FAIL 2.49 m / FAIL HALT_THRASH (robot stuck at the crest) | FAIL HALT_THRASH (11) / (11) | **MARGINAL** (1; 0.02 m) |
| curved / 0.45 / unloaded | MARGINAL / MARGINAL / MARGINAL / FAIL HALT_THRASH (reverse descent) | FAIL HALT_THRASH (8) / MARGINAL (5) | **MARGINAL** (4; 0.03 m) |
| curved / normal / loaded | MARGINAL x4 (stuck at the crest) | MARGINAL (3) / FAIL DID_NOT_REACH_PLATFORM (11) | **PASS** (0; 0.02 m) - 12/12 in the sample |
| curved / 0.45 / loaded | FAIL HALT_THRASH / MARGINAL / MARGINAL / FAIL HALT_THRASH (stuck at the crest) | FAIL edge dep. (timed-drive artifact) / FAIL DID_NOT_REACH_PLATFORM (9) | **MARGINAL** (5; 0.02 m) - 12/12 in the sample |

**7 of 7 cells now ascend (ground-truth verified) and descend the whole ramp on the centerline (max |y| <= 0.03 m, travel 5.6-5.8 m): 1 PASS, 5 MARGINAL, 1 FAIL (HALT count only).** Samples behind it: loaded cells 12/12 x 4 = 48/48 completions (Wilson 93-100%); unloaded regression cells 6/6 x 2.
Earlier samples on the original geometry with the corrected turn: curved / normal / unloaded 0/18 failures on both the scoped and unscoped detectors. Round trips (unscoped detector, ascent legs physically verified, GT-vs-EKF disagreement <= 0.035 m): loaded ok 103% (13 total HALTs, 3 in descent; leg-4 end (1.99, +1.56));
unloaded ok 103% (31 / 15; (1.95, -0.14)). Full tables, samples, instrument amendments, corrections and limits: `validation/phase8/crest_freeze/README.md`.

## Floor 2: two-floor under-construction building (2026-09-20)

`construction_site.sdf` now continues past Floor 1 (`elevated_level`) with a second, 3.2 m higher storey. Full design, dimensions with
OSHA / site-logistics citations, the Fuel sourcing table and the checks: `docs/construction_site_design.md`, section "Floor 2".
Decisions kept: stairs are scenery only (the robot only uses ramps), the payload drop-off mechanism is a later session (the drop-off
point is only *marked*), the Floor-1 route and the validated autonomy stack were not touched (the only edited existing line of the
world is the east perimeter fence, moved 31 m east, plus an `xmlns:experimental` attribute on `<sdf>`).

- **Layout**: Floor-1 extension slab (x 55.97-86.94, y +-10, top 3.35) -> **ramp 2** (ramp 1's validated 7-element chain translated +27.97 m / +3.2 m: 16 deg max, filleted, 4.5 m wide, mu 0.45) -> **Floor 2** (x 68.94-86.94, y +-8, top 6.55). Guardrails (42 in / midrail / toeboard) and debris netting, 6 columns with impalement-capped starter bars, a racked CMU wall with a mason scaffold, a netted facade scaffold, a **scenery** stair-tower core (35.5 deg, 16 risers) away from the lane, material staging near the ramp-2 exit, two Jersey barriers separating the exit lane from staging. **Delivery / drop-off point**: a 3 x 3 m green pad centred at world (79.5, 0) with a marker post - a marker only, nothing attaches or detects.
- **Assets**: Fuel (`OpenRobotics`) for Jersey barrier, Euro pallet, cinder blocks (48), large crate, 55-gal drum, cable spool; scaffolding, rebar, formwork, netting and the stair tower are **not on Fuel** (searched) and are authored from primitives to the OSHA dimensions.
- **Checks**: exact top-face seam math (all seams <= 2 mm step), exact-footprint overlap + lane-clearance check against every pre-existing model, live spawned-pose check, GUI screenshots (`docs/images/floor2_*.png`).
- **Performance finding**: static collision shapes (~2.5 us of step each), not visuals, dominated: the first detailed Floor 2 ran at RTF ~0.4. Scenery now uses envelope collisions (Fuel multi-shape collisions stripped with `<experimental:params>`); max-speed RTF 0.41 -> 1.12. During runs the mean RTF is **0.92-0.93** vs 0.98 before - below this project's 0.995 bar, above the guard's 0.95 start gate; the ground-truth results were unaffected in every metric compared.
- **Verification 1 - Floor-1-only round trip, zero regression** (existing driver unchanged; ground-truth-gated): loaded: leg-3 110%, 14 HALTs, min margin **0.463** (identical to before), final GT (2.12, -0.88) (inside the documented 0.53-2.54 m loaded leg-4 drift distribution); unloaded: 110%, 34 HALTs, margin **0.872** (identical), final (1.95, -0.10). Earlier records: loaded 101-109% / 13-22 HALTs / 0.463; unloaded 102-106% / 31-40 / 0.872.
- **Verification 2 - ramp-2 traversal to Floor 2, empty payload**: **PASS**. All four legs physically verified (GT vs EKF 0.008-0.034 m), final GT pose (76.53, 0.000, **6.675**) (= Floor 2 top + 0.125 m robot rest height), GT max |y| 0.000 m, min clearance to any Floor-2 object along the real path **1.98 m** (>= 1.5 m buffer).
- **Caveat found, then fixed (2026-09-24, next section)**: the ramp-state machine assumed a single ascent -> plateau -> descent cycle; on ramp 2 it labelled the climb `DESCENT`.
- **Also**: `robot.urdf.xacro` now renders the orange payload box only when `payload_mass > 0` (collision and mass unchanged); both payload configurations spawn.
- Files: `validation/phase8/floor2/` (generator `generate_floor2.py`, seam / layout checks, verification driver and logs, `run1_before_fence_fix/`, RTF variant experiments).

## Multi-ramp ramp-state machine (2026-09-24): ramp-2 climb mislabelled `DESCENT` - root-caused live, fixed, verified with Nav2/MPPI

Details, data and tools: `validation/phase8/fsm_multiramp/README.md`; design-doc section "Multi-ramp state machine" in `docs/construction_site_design.md`.

- **Root cause (live, MPPI-driven, unfixed code)**: `state_machine_node.py` chose the slope type by position in a one-way cycle and threw away the pitch sign (`abs(pitch) > 5`): `APPROACH + slope -> ASCENT`, `EXIT + slope -> DESCENT`. After ramp 1 the FSM is in `EXIT`, so ramp 2's climb (pitch -5.06 deg = nose up, +0.53 m/s, ground-truth z rising) became `DESCENT` at t = 208.3 s and stayed for 81 samples while z rose 3.5 -> 6.6 m; the live controller carried `descent_mode` (0.35 m/s, weight 60) and `descent_brake_node` held `ax_max` at 0.40-0.51 during the climb. Replaying the recorded inputs through the original code reproduces the live sequence exactly.
- **Fix**: from `APPROACH`/`EXIT`, a slope starts `ASCENT` if `nose up == travelling forward` (IMU pitch sign + sign of wheel-odometry vx) else `DESCENT`. Local rule (the slope direction is a local fact; profiles are per direction), no perception dependency, mid-ramp edges and HALT recovery untouched. Backup of the original beside the README.
- **Live verification, Nav2/MPPI `FollowPath` for every leg, ground-truth-gated, unloaded and 100 kg loaded**: gate -> ramp 1 up -> Floor 1 -> ramp 2 up -> Floor 2 -> turn -> ramp 2 down -> ramp 1 down, 5/5 legs verified in both; `/ramp_state` = `ASCENT(1 tick) > TRANSITION > EXIT` on **both** climbs and `DESCENT > EXIT` on **both** descents; 0 mislabelled samples vs ground-truth dz/dt; the live `/controller_server` parameters (read via `GetParameters`, what `ros2 param get` calls) matched the current state's profile in every stable sample; the ramp-2 climb now runs `transition_mode` with the braking loop idle (`ax_max` 2.00), the descents run `descent_mode` with the loop active (0.40-1.16).
- **Regression**: Floor-1-only round trips loaded/unloaded: identical state sequence, margins 0.463 / 0.872 unchanged, goal verification 108-109 %; full 7-cell sharp/curved matrix: all cells ascend and descend (GT max \|y\| <= 0.04 m), two HALT-count-only verdicts moved and were tested by interleaved A/B against the original state machine (6 + 6 interleaved trials each: sharp/0.45/loaded HALTs new 4,1,6,2,6,3 vs original 3,4,4,6,24,1; sharp/normal/loaded new 3,6,2,2,5,1 vs original 2,3,3,1,4,2 - no difference); `synthetic` scenarios (two ramps, up-down-up, reversing, no odometry, HALT mid-second-ramp) all as expected.
- **RTF under real autonomous driving** (noted, not fixed): 0.911 (unloaded) / 0.915 (loaded) mean, no drift over 9 min, ~0.01-0.02 below the ground-truth-driven tests (0.92-0.94) and ~0.07 below the Floor-1-only world (0.98): stable, left alone. Thin margin (min ~0.89): the guard's "two samples below 0.90" clause tripped on healthy runs, now configurable (`HG_LOW_RTF`, default unchanged). **Flag for the delivery session**: measure RTF with the attach/detach plugin before trusting guarded runs.
- **Seen in that session, resolved on 2026-09-26 (next section)**: `ASCENT` lasting one IMU tick (climbs ran `transition_mode`) and loaded MPPI climbs HALTed every 3.0 s.

## Keepalive HALTs isolated and fixed, `ramp_mode` decided, full regression (2026-09-26)

Details, tables and data: `validation/phase8/fsm_multiramp/README.md` section 7; design-doc section "Keepalive HALTs and the ramp_mode decision" in `docs/construction_site_design.md`.

- **HALT cadence: isolated by experiment.** Only the switcher's keepalive interval was changed (loaded robot, MPPI-driven ascent of both ramps, reverted after): HALT gaps followed it - 1.4 s -> 1.6 s (89 climb slip-HALTs), 3.0 s -> 3.2 s (38), 6.0 s -> 6.2/12.2 s (7), disabled -> none (0), shipped 3.0 s again -> 3.0 s cadence (6). Mechanism: every parameter push resets MPPI's optimizer; when the command then dips to ~0 the slip ratio (|commanded - actual| / |commanded|) explodes and crosses 0.5 (0 HALTs without a dip, 36/38, 7/8, 55/63, 6/8 with one).
- **Fix (installed)**: the keepalive now reads the seven parameters it owns back (GetParameters, no reset) and re-pushes only on a mismatch/unreadable answer. Idle: 0 parameter-set events in 40 s (91 before); corrupted parameters still restored within one keepalive period; loaded full-trip HALTs 65 -> 12.
- **`ramp_mode` decision: retuned.** The `ASCENT -> TRANSITION` trigger fired on one perception message with a break point <= 0.75 m ahead - true at every ramp foot and as noise mid-climb - and `TRANSITION` cannot return to `ASCENT`. Now: a crest = break point that FLATTENS the slope (`slope_deg <= -5`; foot +8..+13, noise ~0) and persists (>= 6 of the last 15 messages). Live A/B on both ramps, loaded and unloaded: climbs 44-46 s instead of 69-73 s, identical margins/lateral error, profile = state's profile in 100 % of samples, fewer climb HALTs (5 vs 11). The site now runs `ramp_mode` for the sustained climb and `transition_mode` for the last ~2.7 m, as designed.
- **Regression with the final code**: 7-cell matrix all ascend+descend (GT \|y\| <= 0.044 m; one HALT-count-only FAIL, sharp/normal/unloaded 10 HALTs, tested at 16+16 interleaved trials vs the baseline: not significant, p = 0.33); Floor-1 round trips 110 % / 111 % (margins 0.463 / 0.872 unchanged); full MPPI two-ramp trips loaded/unloaded 5/5 legs, 0 mislabels, HALTs 12 / 13; ramp-2 traversal PASS; the skipped curved/normal/loaded A/B closed (6+6, no difference).
- **Open for the delivery session (understood, not fixed)**: (i) HALT/recovery push loop - every state change while moving pushes a profile and resets MPPI, so a HALT recovery under load can re-trigger itself (one 75-HALT thrash observed: 84 halt_mode/flat_mode push pairs); (ii) RTF headroom ~7 % (mean 0.93 whole-stack; the heavy probe itself costs ~0.05; guard clause now `HG_LOW_RTF`-configurable).

## Delivery-session prerequisites resolved: detach mechanism + RTF baseline (2026-09-26 / 2026-09-28)

Details: `validation/phase8/delivery/README.md`; design-doc section "RTF dry-run of a no-op detachable payload" in `docs/construction_site_design.md` (now the corrected mechanism + clean numbers).

- **Detach mechanism decided and built**: releasing the payload directly above the chassis left it resting on the robot. Fixed with a delivery dock beside the pad (level with the payload's measured resting
  underside) and a sim-side actuator (gz `ApplyLinkWrench`, not part of the robot's own stack) that pushes the released, low-friction payload sideways onto it. Verified in every one of 7 repetitions: lands at
  dock height, within the dock footprint, and stays there through the robot driving away.
- **RTF baseline, real-time, repeated to convergence on verified AC/performance power** (host state checked before and every 10 s during every run; contaminated and harness-guard-starved reps discarded and
  topped up to 5+ clean reps): attached (today's config) 0.959, attached with the plugin unused 0.955 (no measurable plugin cost), **detached resting on the dock 0.970**, **detached with the robot driven away
  0.954** - all at or above the ~0.93 whole-stack baseline. **Verdict: affordable, no lighter collision/physics setup needed** before the real mission.
- **Not yet done (deferred)**: the real mission - detach trigger, ROS-side bridge, full loaded round trip with the corrected mechanism, ground-truth verified.

- **Real mission built and run (2026-09-30)**: `payload_delivery_node.py` (new node) watches the robot's own EKF pose and fires `/payload/detach` as a real autonomy-stack event
  (stopped in the delivery zone, payload still attached, 1 s dwell - not a test-script command), then zeroes the stability monitor's payload mass once gz confirms the release.
  `full_mission_test.py` drove the whole loaded round trip (gate -> ramp 1 -> Floor 1 -> ramp 2 -> Floor 2 -> pad -> release -> turn -> ramp 2 down -> Floor 1 -> ramp 1 down -> gate)
  with the same Nav2/MPPI + ground-truth-vs-odometry gate every leg in this project uses. **3 of 3 clean guarded runs pass**: every leg inside its gate (worst case -0.049 m over 27 m),
  the payload landed on the dock every time (y = 0.72-0.88 m of the 0.44-1.44 m dock span), HALTs (9-10) and RTF (~0.90) unchanged from the loaded full trip without the delivery event.

## Chassis mesh swap for public showcase, verified cosmetic-only (MiR100, 2026-09-30)

Details: `validation/phase8/mesh_swap/README.md`; design-doc section "Chassis mesh swap" in `docs/construction_site_design.md`. Supersedes an earlier
Husky A200 mesh swap done the same day - that work was fully removed and replaced.

- Replaced the placeholder box/cylinder visuals with real MiR100 chassis/wheel meshes (DFKI-NI's `mir_robot`, BSD 3-Clause), per-axis scaled to this
  project's own existing collision dimensions. Only `<visual>` elements (plus two cosmetic-only `<material>` color definitions, since STL carries no
  embedded color) changed.
- **MiR100, not the originally-requested MiR250**: the intended source has no properly-licensed MiR250 mesh package; substitution was raised with the user
  and confirmed rather than silently assumed. Labeled honestly as MiR100 everywhere (meshes, attribution, docs, screenshots).
- **Verified**: rendered-URDF diff (all 4 payload/detachable combinations) shows every link name, joint (name/type/parent/child/origin/axis), collision
  block, inertial block and `gz_frame_id` byte-identical before/after; only the visual/material content differs.
- **Regression check**: full loaded construction-site round trip, complete stack live - PASS, stability margin floor 0.463 bit-identical to the established
  baseline, HALT count and ground-truth checks all within/matching the existing range.
- Screenshots: `docs/images/mesh_*.png`.

## Unsecured-cargo physics demo mode (`cargo_mode:=loose`), isolated from all validated results (2026-09-30)

Details: `validation/phase8/mesh_swap/cargo_demo/`; design-doc section "Unsecured cargo demo mode" in `docs/construction_site_design.md`.

- New opt-in xacro/launch arg `cargo_mode:=secured|loose`, **default `secured`** = today's exact fixed-joint payload behavior, unchanged. Verified
  byte-for-byte identical rendered URDF (all payload/detachable combinations) whether `cargo_mode` is omitted or explicitly `secured`.
- `cargo_mode:=loose` makes the payload a genuine separate free rigid body resting on the chassis (no joint), able to slide/tip under real contact
  friction. **Never used by any validated/regression script** - `record_cargo_demo.sh` is the only place in the project that ever passes it.
- Demo finding (loaded ramp descent, secured vs. loose, same spawn/path/conditions): secured cargo shows **zero independent motion by construction** (it's
  welded into the chassis rigid body, not even a separate model to poll). Loose cargo stays essentially fixed relative to the chassis through the flat
  elevated-platform arc-turn, then visibly **slides backward and downward once the actual ramp descent begins**, and in a full gate-to-descent run,
  loose cargo can destabilize the robot severely enough to trigger a stability-margin floor-out and a HALT storm before the descent is even reached - a
  demo/illustration finding, not a claim about any validated result.

## Known gaps (honest, not hidden)

> **STATUS: FINAL - closed for this round (2026-09-20).** All investigation threads are closed: the stall-detector / curved-world descent (test-procedure artifact, scoping workaround removed), the loaded-robot crest freeze (0.05 m ramp-geometry step, fixed), the ascent goal-completion gate (added), and the post-fix perception re-checks (break-point accuracy and obstacle-LiDAR, `validation/phase8/perception_regression/README.md`: pass, with two measured changes noted there).
> **Only two items remain open, both explicitly OPTIONAL future work with no known impact on any current validation result:**
> 1. *Why turning in place on the curved crest displaces the chassis 0.2-1.3 m sideways* (documented, understood well enough to route around; whether it still matters now that the crest step is gone was not tested).
> 2. *No onboard ascent verification:* the ascent physical-reality gate uses Gazebo ground truth and therefore exists in the test harness only; the stack has no on-robot equivalent (it would need an exteroceptive floor-motion sensor).
> Everything else listed below is either resolved (struck through / annotated), or a standing, previously documented SCOPE BOUNDARY of this simulation project (e.g. no sim-to-real bridge, no planner_server / bt_navigator / AMCL / map, pitch-axis-only stability margin, `/cmd_vel_scale` has no consumer, `/odom` drift under slip, straight-travel stall-detector false latches, the leg-4 drift distribution) - not an open investigation.


- **LOADED-ROBOT CREST FREEZE: ROOT-CAUSED AND FIXED (2026-09-20) - a 0.05 m geometry authoring step, the same bug class as the construction-site freeze; also the sharp-world "loaded cannot climb" finding.**
  Full report: `validation/phase8/crest_freeze/README.md` (pre-registration, wheel-level recordings, exact corner math, reverification, ascent gate).
  - *Mechanism.* Exact top-face corner math over every seam of every `ramp_world_*.sdf` finds two steps per world (curved: +0.0500 m at the crest -> platform join and -0.0502 m at the ramp foot; sharp: +0.0502 / -0.0537 m); all other seams match to <= 1.4 mm.
    The ramp slabs were authored 0.05 m too low (0.05 / 1.05 = slab centres treated as surfaces 0.10 / 1.10); the construction site had been fixed earlier, these worlds never were. Wheel-level recording of curved / 0.45 / loaded (twice): chassis frozen at the identical pose
    (gz x 8.018, z 1.157) for 40-50 s, all four wheels turning (0.09 m/s) at a chassis speed of 0.0003 m/s, `/wheel_slip` mean 0.012 (blind), EKF x running +4.5 m ahead of ground truth, front wheel centre within 3 mm of the step corner. The old sharp-world freeze (gz x 7.119) is within 1.1 mm of the same kind of contact.
  - *Fix and result.* Ramp slabs raised 0.050 m (curved) / 0.052 m (sharp) in all 7 ramp worlds (originals kept); seams now <= 1.8 mm. Corrected turn procedure, unscoped detector, guarded: **loaded ascent + descent completion 48 / 48 (Wilson 93-100%)** across the four loaded cells (12 each; before: 0/10, 6/8, 0/8, 0/8), unloaded regression cells 6/6 each, descent travel 5.57-5.81 m, max |y| <= 0.03 m.
    Final 7-cell matrix: every cell ascends (ground-truth verified) and descends fully: 1 PASS, 5 MARGINAL, 1 FAIL (sharp / normal / loaded at 8 HALTs, HALT count only; its 12-run sample maximum is 6). Both construction-site round trips complete (ascent legs physically verified).
  - *Ascent gate.* ASCENT had no physical-reality gate (validation test: FSM state EXIT; round-trip driver: EKF x crossing a target). The descent's `/verified_progress_m` pattern would NOT have caught this freeze (it grew 96% of the raw wheel distance while the chassis was frozen: every stall/resume cycle re-accumulates ~5 s of spinning), so both drivers now
    also compare Gazebo ground-truth displacement with the odometry displacement (tolerance 0.5 m, calibrated on 24 healthy leg ends <= 0.09 m): `FAIL: ASCENT_POSITION_UNVERIFIED` / `DID_NOT_REACH_PLATFORM`. Harness-only (needs ground truth); the stack still has no on-robot physical ascent check.
  - *Supersedes* the statements below that the loaded robot "cannot reliably clear the crest" / is "a robot limitation": that was this bug.
- **STALL-DETECTOR SCOPING WORKAROUND: REMOVED (2026-09-19). The curved-world "edge departure" was a test-procedure artifact, now corrected; see "Final corrected matrix" below and `validation/phase8/path_structural/README.md` section 7.**
  Installed detector = the unscoped fixed detector (48625b76); the workaround version is kept as `stall_detector_control/data/motion_stall_detector_node.SCOPED.py`. The rest of this entry is the history of how the cause was found;
  where it says the workaround is "in place", "unresolved", or points at PathAlignCritic / crosstrack work, this paragraph and the final corrected matrix govern.
  - *Round 2 outcome (the corrected procedure is now the default for every world).* Larger samples on curved / normal / unloaded: **scoped 0 / 18 failures, unscoped 0 / 18 (Wilson 0-18% each), all 36 reached the ramp foot, max |y| <= 0.056 m**;
    positive control on the unscoped detector: forced freezes 18/18, false latches while rotating 0, same as its original validation; both construction-site round trips complete. New limits found (SUPERSEDED 2026-09-20: this was a 0.05 m geometry step, fixed - see the entry above): the loaded robot cannot reliably
    clear the crest (curved / normal / loaded 6 of 8 corrected runs; curved / 0.45 / loaded 0 of 7), which the old harness hid because /ramp_state reaches EXIT while the chassis is still on the crest.
  - *FINAL STRUCTURAL EXPERIMENT (`validation/phase8/path_structural/README.md`) - supersedes the "unresolved / crosstrack / PathAlignCritic" framing in this entry.*
    Densifying the descent path 0.4 -> 0.05 m (Arm A, n=12) and adding an explicit cross-track critic (Arm B, n=12) both "improved" ground truth
    (0/12 edge departures each, vs scoped 5/14: -36 points, 95% CI -61..-5, Fisher p=0.043; vs original 0/6: 0 points, CI -39..+24) **only because the
    robot does not move**: 0/12 genuine descents in either arm (median travel 0.10 / 0.05 m of 6 m; Nav2 aborts at 15 s with the robot at the crest).
    Recorded ground truth vs EKF vs commands then showed the cause: the harness's open-loop 180-degree turn achieves only ~90 degrees (robot faces
    the panel edge), and turning on the curved crest translates the chassis sideways by 0.2-0.4 m (open-loop) to 1.0-1.3 m (closed-loop 180 deg) while the
    EKF's y stays exactly 0.00 (it fuses wheel forward velocity only), so the controller starts the descent believing it is centered. Arm H (correct
    heading, turn still on the crest, n=6): 5/6 departures. **Arm P (closed-loop turn on the flat platform, n=6): 0/6 departures, max |y| 0.02-0.04 m, 6/6 full
    descents, unseen offset 0.02 m; Arm P with the UNSCOPED fixed detector (n=6): 0/6, 6/6, HALT_THRASH 0/6.** So the unchanged MPPI/0.4 m path/critics track the ramp
    correctly when the robot starts on it, the "structural limitation of MPPI" reading is refuted, and the scoped workaround is not needed under that procedure.
    **Not done, deliberately:** removing the scoping. It is only valid together with a change of the curved-cell test PROCEDURE (turn on the platform), which
    redefines the instrument for every curved cell; the scoped detector stayed installed at that point (round 2 later removed it - see the header of this entry). Also corrected: every ground-truth PASS on curved cells at
    normal friction has meant "did not leave the panel before the leg ended", not "descended" (travel 0.1-2 m of 6 m in every recorded run); the one matrix cell
    that reaches the ramp foot (`curved / 0.45 / unloaded`) reverses down it. The seam-stumble and unchecked-crosstrack explanations in "Crosstrack diagnosis" were
    never tested as causes for this cell and are superseded. **Updated removal condition:** turn the robot on flat ground (or give it an absolute lateral
    position reference), re-run curved/normal/unloaded on the unscoped detector, then empty `ROTATION_CHECK_EXCLUDED_STATES`, re-run the matrix and both round trips.
  - (Older text of this entry follows; where it says the cause is unknown or points at PathAlignCritic work, the paragraph above governs.)
  In the validation cell `curved / normal friction / unloaded` the robot leaves the 3.0 m fillet panel during the descent
  (ground-truth |y| beyond the 1.5 m half-width). **Nobody knows why.** The documented seam-stumble / crosstrack
  explanation ("Crosstrack diagnosis") is a hypothesis for this cell that has never been tested, and the PathAlignCritic /
  path-tracking thread that would address it is unfinished. What the code now contains is a *workaround that preserves an
  accidental mitigation of that unsolved problem*, and it should not be read as understanding or solving it.
  - *What the accident is.* With the original stall detector this cell passed ground truth 0/6 failures (max |y| 0.39-1.11 m).
    Once the gyro rotation check (added to stop false stall latches on the construction site) was active everywhere, it
    failed **10/10** (1.74-3.85 m; Fisher p = 0.00012 vs the original; an order effect was ruled out by a control batch).
    The difference is a **second stop inside `DESCENT`**: the original detector fires a "Motion stall confirmed" latch 0.4-0.8 s
    after `DESCENT` begins (6/6 original runs, 0/10 fixed), a 1.4-2.2 s pause, then a wheel-slip HALT on resume and
    `descent_brake_node`'s cautious 40% restart. That pause was never a designed feature; it is the detector misfiring, and
    it happens to help. (An earlier note here claimed the fixed detector had no crest HALT at all; that used a misaligned
    window and was wrong - every run has a slip HALT ~0.4 s before `DESCENT`.)
  - *The workaround.* `motion_stall_detector_node.py` now switches the rotation check OFF during `DESCENT`
    (`ROTATION_CHECK_EXCLUDED_STATES = ('DESCENT',)`), so the original scan-only logic governs there and the pause returns.
    Boundary by `/ramp_state` (documented in the code): `APPROACH`, `ASCENT`, `TRANSITION`, `EXIT` -> rotation check active
    (as validated); `DESCENT` -> original logic; `HALT` -> inherits the state it interrupted; unknown state -> original
    logic. It is keyed on FSM state, not on world identity (the node cannot tell curved from construction site); that costs
    nothing validated, because the rotation check never suppressed a latch during the construction site's ramp descent.
  - *Was the mechanism tested, not just correlated?* Partly. On the fixed detector (no in-`DESCENT` stop) three arms were
    injected at the first `DESCENT`, none a stall latch: **a deliberate 1.75 s hold** (via a temporary watchdog hook) brought
    4 of 6 runs back inside the panel (mean max |y| 1.59 m vs 2.49 fixed / 0.90 original; closes 57% of the gap; p = 0.008
    vs fixed, not distinguishable from the original at n=6, p = 0.45); **the descent brake's cautious limits without a
    pause** (0 of 5 pass, 19% of the gap) and **MPPI parameter re-pushes / optimizer resets without a pause** (0 of 5 pass,
    17% of the gap) did nothing distinguishable from the fixed detector. So the *stop* is the ingredient that matters, and it
    does not work through MPPI reset or reduced acceleration limits. It is a **partial** reproduction (2 of 6 hold runs still
    failed), and a stop cannot be isolated further because any stop-and-go here trips a wheel-slip HALT at resume (which
    drags in MPPI pushes and the cautious restart).
  - *What the workaround delivers, honestly (closing sample: 14 scoped runs, guarded clean host, none starved).*
    `curved/normal/unloaded` on the scoped detector: **9 pass, 5 fail = 36% failure (Wilson 95% 16-61%)**; max |y| mean 1.32 m,
    median 1.03 m, range 0.38-2.54 m. (An earlier 6-run sample was 2/6; 8 more runs gave 3/8 - the two agree.) Three of the 14
    runs dipped to a real-time factor of 0.992-0.994, under the requested 0.995; with them excluded the result is unchanged
    (4/11 fail, 36%, Wilson 15-65%).
    - **Versus the fixed detector (10/10 fail): meaningfully better, not in doubt.** Failure-rate difference -64 points,
      95% CI -84 to -27; Fisher p = 0.002; mean max |y| 1.17 m lower (bootstrap 95% CI 0.67-1.69 m; rank test p = 0.003).
    - **Versus the original detector (0/6 fail): NOT shown to be the same, and NOT shown to be worse - the data cannot say.**
      Difference +36 points, 95% CI **-8 to +61**; Fisher p = 0.26; mean max |y| 0.42 m higher (bootstrap CI +0.03 to +0.84;
      rank test p = 0.44 - the two mean-based and rank-based readings disagree at the edge of significance). The point estimate
      is that the scoped detector fails about a third of the time where the original failed 0 of 6, but the original's own
      6-run interval (0-39%) contains the scoped rate, so "indistinguishable" here means *the original sample is too small to
      tell*, not *equivalent*. Every reading points the same way (scoped not better than original), so the honest expectation
      is that the workaround recovers most, not all, of the original's behaviour. Pinning it down would need ~10-15 more
      original-detector runs (0/10 vs 5/14 gives p = 0.053, 0/15 gives p = 0.017); that needs a detector swap and was not part
      of this request.
    - The in-`DESCENT` stall latch remains the marker of a good outcome: across the 30 runs of original, scoped and fixed
      detectors, with the latch 15 of 16 pass, without it 0 of 14 pass (Fisher p = 2e-7); the latch fired in 10 of 14 scoped
      runs (original 6/6, fixed 0/10), and the 4 scoped runs without it all failed. This is a post-hoc association across
      detectors, consistent with the causal test, and it explains why the scoped detector falls short of the original: the
      latch does not always fire under it.
  - *Final regression on the scoped detector.* Both construction-site round trips complete (`ok=True`, 102-103% verified;
    loaded HALTs 15, unloaded 31, matching the fixed detector's 14 and 30, so the validated rotation-check gain is kept).
    Ground truth passed in every matrix cell that reached the descent, and in every repeat of the two curved cells below
    (0.05-1.08 m). The two sharp-world loaded cells still fail `DID_NOT_CLIMB` (unchanged, unrelated).
  - *The two curved cells whose overall verdict moved MARGINAL -> FAIL on HALT count (repeated on the scoped detector).*
    - **`curved / 0.45 / loaded`: run-to-run variation, not a regression.** 5 scoped runs: 1, 1, 2, 7, 8 HALTs (median 2,
      mean 3.8); HALT_THRASH (>= 8) in 1 of 5 (Wilson 4-62%); ground truth 0.94-1.08 m, 0/5 fail. The very first matrix, before
      any stall-detector work, already produced 9 HALTs (FAIL) in this cell, and the old / fixed detectors gave 3 / 2. The
      8-HALT run was the tail of a wide distribution.
    - **`curved / 0.45 / unloaded`: probably a GENUINE, additional cost of the workaround (not just noise), physically
      harmless.** 9 scoped runs: 3, 7, 7, 7, 8, 9, 10, 13, 17 HALTs (median 8, mean 9.0); HALT_THRASH (>= 8) in **5 of 9**
      (Wilson 27-81%), i.e. `FAIL: HALT_THRASH` about half the time. The earlier runs of this cell were 1 (first matrix), 3
      (old detector) and 4 (fixed detector), all MARGINAL. Scoped vs those three: Mann-Whitney p = 0.032 (0.068 when the 3
      scoped runs under RTF 0.995 are dropped; 3 of 6 still thrash), bootstrap 95% CI of the median difference +3 to +9 HALTs.
      Caveats, stated plainly: the comparison runs are single runs of three different detectors, so the elevation cannot be
      attributed to the scoped detector *versus* the cell's own variability with certainty (Fisher on the thrash fraction,
      5/9 vs 0/3, is p = 0.20); the extra HALTs were not traced to a cause. What is not in doubt: **ground truth is unaffected
      (max |y| 0.05-0.41 m in all 9 runs, the robot stays on the panel); the cost is extra stop-and-go on this cell.** Treat it
      as a known cost of the workaround and expect this cell's overall verdict to read FAIL (HALT_THRASH) on the matrix's
      >= 8 HALT rule about half the time.
  - *Removal condition (updated by the structural experiment).* The workaround exists because the curved-cell procedure turns the robot on the
    curved crest, where the chassis is displaced sideways by an amount the EKF cannot see. It becomes unnecessary once the robot turns on flat
    ground (Arm P / P-fixed: 0/6 and 0/6 on the scoped and unscoped detectors) or has an absolute lateral position reference. Then: adopt the
    procedure change for the curved cells, re-run curved/normal/unloaded (10+ runs) on the unscoped detector, set `ROTATION_CHECK_EXCLUDED_STATES = ()`
    (or reinstall the fixed detector), re-run the positive control, the 7-cell matrix and both round trips. Until then do not describe the curved-world
    descent at normal friction as working, and do not tune the stall detector's DESCENT behaviour without re-running `curved/normal/unloaded`.
  - *Status of this entry.* Closing entry for the stall-detector / curved-world investigation. What stays open: the procedure change and the removal it
    would license, and the localization blind spot itself (no absolute lateral reference; why the crest turn displaces the chassis was not investigated).
  - Data and tools: `validation/phase8/curved_unloaded_{origdet,repeat,fixedctl,pause,scoped,scoped2}_logs/`,
    `cell_curved_mu045_{unloaded,loaded}_scoped_logs/`, `analyze_closing_samples.py` (`closing_samples_analysis.txt`), `path_structural/` (final structural experiment),
    `matrix_regression_scoped_logs/`, `roundtrip_scoped_*`, `stall_detector_control/` (`test_state_scope.py`,
    `crest_pause_injector.py`), and the validation README, "State-scoped detector: causal test and final validation".
- ~~`state_machine_node.py`'s `ASCENT -> TRANSITION` edge requires a
  sharp pitch-RATE spike and can fail to fire under gentle, controlled
  driving, and `ramp_classifier_node.py`'s `breakpoint_distance_m`
  estimator itself can stall instead of counting down~~ **- both
  root-caused and fixed in phase 8.** The FSM side was fixed earlier (a
  completeness fallback edge, `ASCENT and not on_ramp -> EXIT` - kept in
  place as defense in depth, not removed). The estimator side - "why"
  the earlier fix's own investigation left open - was root-caused in
  phase 8 by instrumenting the actual two-segment search tick-by-tick
  (point count, chosen split, accepted/rejected) during a real climb,
  not guessed at: right at a sharp break point, the "before the corner"
  side of the point cloud genuinely only has 1-3 valid returns (a real
  gap in the scan - the corner occludes what's beyond it from that
  close, not sensor noise), and `MIN_SEGMENT_POINTS`'s old default of 5
  made that cluster categorically too small to ever be tried as a
  candidate segment - forcing every candidate split into the long
  uniform surface beyond it, which correctly (but unhelpfully) found no
  slope difference there and reported no break point well before the
  robot actually lost the corner for real. Fixed by lowering
  `MIN_SEGMENT_POINTS` to 3, confirmed live to extend valid,
  still-decreasing detections down to ~0.2 m from the corner (was ~0.5
  m), comfortably inside `BREAKPOINT_TRIGGER_DISTANCE_M`'s 0.75 m - and
  confirmed end-to-end with the ORIGINAL gentle-burst driving style that
  first exposed this bug: `ASCENT -> TRANSITION` now fires from genuine
  `breakpoint_imminent` (not the pitch-rate fallback), and the
  completeness edge never needs to fire. See `ramp_classifier_node.py`'s
  `_best_piecewise_fit` docstring for the full evidence chain.
- ~~`state_machine_node.py`'s HALT recovery can mislabel a genuine
  `EXIT` as `APPROACH`, silently preventing `DESCENT` from ever being
  reached`~~ **- fixed.** Found live in a later phase-7 pass (see the
  "Phase 7, continued" section for the full mechanism): HALT recovery
  used to send any non-DESCENT pre-halt state, `EXIT` included, back to
  `APPROACH`; a HALT blip right at the ramp crest could therefore erase
  the fact that the robot already reached `EXIT`, so heading back
  downhill afterward got relabeled `ASCENT` instead of `DESCENT`. Fixed
  by always recovering to `self._pre_halt_state` exactly (generalizing
  phase 7's original DESCENT-specific special case rather than adding
  another one) - a single remembered value is provably sufficient even
  if HALT fires again shortly after recovery, since HALT episodes
  cannot nest in this FSM (see `state_machine_node.py`'s HALT-entry/
  HALT-exit comments for the full reasoning). Verified live by
  reproducing the exact scenario (HALT firing naturally at the ramp
  crest, twice, during phase 8's validation matrix and a dedicated
  repro test) and confirming `DESCENT` was reached and sustained
  afterward, and re-confirmed under much heavier HALT churn during
  phase 8's curved-world matrix cells (2-6 HALTs per run, recovery
  never once regressed past `EXIT`/`DESCENT` back to `ASCENT`/
  `APPROACH`).
- **`vx_max`-capping for descent braking (phase 7's original approach)
  was re-tested in a later phase-7 pass with corrected methodology and
  found to be
  actively harmful, not just ineffective**: 146 HALTs and a 4.5x longer
  traverse than doing nothing, from a feedback spiral where cutting the
  speed ceiling forces harder deceleration, which causes more slip,
  which cuts the ceiling further. **Replaced** by capping MPPI's
  `ax_max`/`az_max` instead, which matches the unbraked baseline exactly
  in the corrected test (see the "Phase 7, continued" section) - this
  is the
  approach `descent_brake_node.py` now implements.
- The stability margin model is a simplified pitch-axis-only planar
  calculation; a real 3D support-polygon model accounting for roll
  simultaneously is a documented next step.
- ~~`nav2_param_switcher_node.py`'s failure handling is retry + loud
  logging + accurate state tracking, not a redundant independent
  enforcement path~~ **- fixed.** `cmd_vel_watchdog_node.py` (see
  "Phase 8, continued") now sits inline between `velocity_smoother` and
  the robot's real actuation topic, ramping the commanded velocity to
  zero on `/ramp_state == HALT` independent of whether
  `/controller_server/set_parameters` ever lands - verified live with
  the reconfiguration path deliberately left dead and upstream commands
  still climbing: the actuation topic still reached and held zero in
  121 ms. A live A/B regression the fix's own first (non-ramped)
  version caused is also documented there, not swept under the "fixed"
  label.
- ~~The LiDAR can't be both a ramp-profiling sensor and a 360°
  obstacle-detection sensor at once~~ **- fixed.** `robot.urdf.xacro`
  now has a second, independent, level-mounted LiDAR
  (`obstacle_lidar_link`, topic `/scan_360`) purely for Nav2's
  `ObstacleLayer`, leaving the original rolled sensor (`/scan`)
  untouched for `ramp_classifier_node.py` - see "Phase 8, continued"
  for the live verification (a real obstacle detected at its exact true
  bearing/range, actually marked in the costmap, with ramp perception
  confirmed unaffected).
- **`ramp_classifier_node.py`'s slope estimate is still not fully
  reliable at very close range to a sharp (non-filleted) break point**,
  though phase 8 changed the failure mode and reduced how often it
  happens. Root cause (found live, not guessed): the same near-corner
  seam artifact documented since phase 4 (a downward ray grazing a
  sharp box-to-box seam can catch a near-vertical sliver of the ramp's
  end face) is far more exposed once the reported slope comes from a
  small local window (needed to fix the curved-world onset-angle bug
  below) instead of a large averaged segment. Mitigated with a
  plausibility guard (no real surface here exceeds ~16 deg, so a
  window reading beyond that falls back to the largest plausible
  DP-fitted segment instead) plus corroboration-by-size (prefer the
  best-supported plausible reading, not just the first one found) -
  live-verified this recovers the correct ~17.8 deg reading in the
  large majority of ticks (roughly 3 in 4, up from wild ±60-90 deg
  swings before the guard), with the remainder landing on a smaller,
  still-plausible wrong value rather than an obviously-bogus one.
  **Still not consumed downstream** (only `breakpoint_distance_m` and
  `confidence` drive `breakpoint_imminent`, and neither the fix nor
  this residual noise touches either), so this remains informational-
  only, same as the original phase 4 finding - a future consumer of
  `slope_deg` should still not trust it within roughly the sensor's
  own resolution-limited range of a corner.
- ~~Curved-world slope estimation converges to the ramp's peak angle
  instead of the true local onset angle on filleted (multi-segment)
  transitions~~ **- fixed in phase 8.** `ramp_classifier_node.py`'s
  original two-segment fit could only ever report one "far" slope for
  everything past its single split, so approaching
  `ramp_world_curved.sdf`'s 3-panel fillets (0 -> 5.33 -> 10.67 -> 16
  deg) it reported ~15-18 deg - the ramp's ultimate peak - even while
  the robot was still on flat ground nowhere near the fillet yet (live-
  logged: 14.5-16.6 deg for the entire flat approach). Fixed by
  extending to a proper multi-segment piecewise fit (dynamic
  programming, segment count chosen by BIC rather than a fixed target -
  a plain RSS-improvement threshold wasn't strict enough and let the DP
  carve out fake segments from sensor noise on flat ground) plus a
  small local window for the specifically-reported onset slope, since
  the true first fillet panel's ~3 cm height change over its 0.3 m run
  is close enough to real scan noise that BIC often can't justify
  isolating it as its own segment even with the multi-segment fit.
  Live-verified: readings during approach now correctly show 4-9 deg
  once close enough (previously a flat ~15-18 deg throughout);
  very-long-range readings still lean toward the peak angle, which
  looks like a genuine sensor-resolution limit (the fillet's short,
  shallow first panel isn't angularly resolvable from several metres
  out) rather than an algorithm bug, and re-verified this doesn't
  regress the sharp world's own accuracy numbers above. See
  `ramp_classifier_node.py`'s `_best_piecewise_fit` and `_on_scan` for
  the full evidence and design reasoning.
- No sim-to-real bridge yet; this is scoped as a simulation project
  first, hardware validation is future work.
- **No `planner_server`/`bt_navigator` yet**, so there's no
  waypoint-following goal source to exercise `FollowPath` end-to-end -
  phase 3's live test drove the robot with a manual `/cmd_vel` publish
  rather than a Nav2 goal. Phase 5 is the right place to add a
  goal source once slope-aware planning is in scope.
- **`/odom` drifts hard from wheel spin under real slip or when the
  robot is stuck** (e.g. driven off the end of the modeled world, or
  wedged) - the DiffDrive plugin's odometry is wheel-rotation dead
  reckoning with no contact feedback, confirmed directly during phase
  6's low-friction slip test (see above). The `/cmd_vel` timeout gap
  flagged alongside this in the phase 3 session is now fixed (phase 6,
  `nav2_velocity_smoother`'s `velocity_timeout`) - this bullet is just
  the odometry-accuracy half, which remains a real limitation for
  anything that trusts `/odom` for localization during a slip event.
- `/cmd_vel_scale` (from `state_machine_node.py`) still has no
  consumer - the MPPI profile switch now provides an equivalent,
  planner-enforced hard limit per state, so this may end up redundant
  rather than needing its own wiring, but that's a phase 5 call.
- ~~The loaded payload configuration (100 kg, +0.1 m forward CG offset)
  cannot climb the sharp-world ramp at all~~ **- root-caused: a genuine
  physical/design constraint, not a software bug, not fixed by
  design.** **[REFUTED 2026-09-20 - see `validation/phase8/crest_freeze/README.md`: the freeze
  at gz x = 7.119 m was located correctly but the cause was a 0.05 m authoring step (the ramp slab
  0.05 m too low) at the ramp/platform seam, not breakover/ground clearance: the front wheel is
  within 1.1 mm of that step corner at the freeze pose, and with the step removed the same 100 kg
  robot climbs the sharp world 24 / 24 times.]** Found live in phase 8's validation matrix, root-caused in a
  follow-up session (see "Root-causing the loaded-payload climb
  failure" in the phase 8 section for the full evidence chain). Ruled
  out, with direct evidence, not assumption: `/stability_margin`-driven
  HALT (margin stayed 0.46-0.74 throughout, never near the 0.15
  threshold), MPPI/`ramp_mode` tuning (ascent drives via raw
  `/cmd_vel_nav`, bypassing MPPI and `velocity_smoother`'s limits
  entirely), and FSM/perception gating (the FSM reaches `ASCENT` and
  `TRANSITION` completely normally). The actual cause: Gazebo's own
  ground-truth world pose (queried directly, bypassing `/odom`, which
  had drifted nearly 4 m ahead of reality) showed the robot's real
  position freezing bit-for-bit at world x=7.119 m - almost exactly
  where the chassis's own front face reaches the sharp, unfilleted seam
  between `zone_b_ramp`'s and `zone_c_exit`'s separately-modeled
  collision boxes - while `/wheel_slip` stayed at exactly 0.000 the
  entire time (wheels spinning at the correct commanded rate against
  *something*, just not producing chassis motion - a failure mode this
  metric cannot see, since it only ever compares wheel rotation rate,
  never actual displacement). This is a textbook rigid-body
  ramp-breakover-angle problem: `base_link`'s URDF geometry gives this
  chassis essentially zero ground clearance by design (wheel radius
  almost exactly cancels the chassis's own half-height), and the
  payload's weight - pressed down through a forward-shifted, elevated
  CG right over the front of the chassis - tips a marginal, unloaded-
  robot-survivable interference into an actual stuck condition.
  Cross-validated by the curved world's loaded cells both succeeding -
  its filleted transitions have no single sharp ridge to catch on,
  independent of payload. **Not fixed** (a genuine physical constraint,
  documented rather than patched); what would actually fix it for
  real: real ground clearance in the chassis design, a lower/more
  centered payload mount, a wider wheelbase relative to chassis length,
  filleted rather than sharp transitions on real deployment routes, or
  a lower rated payload specifically for sharp-transition routes if the
  chassis itself isn't changed.
- **Lateral drift under any turning motion, caused by wheel odometry
  itself being an unreliable localization source when Nav2's
  `FollowPath`/MPPI controller trusts it - fixed with an EKF
  (`robot_localization`).** Originally surfaced as two apparently
  separate findings that turned out to share one root cause:
  - The curved world's descent legs frequently stalled well short of
    their target distance, with `EXIT`/`DESCENT` chatter and anomalous
    final pitch readings (seen up to 90°/-50° in different runs) far
    outside the ramp's 16° design grade - root-caused live (see
    "Root-causing the curved-world descent chatter" in the phase 8
    section) to a genuine physical tip-over: ground-truth pose showed
    the robot's Y position drifting past the edge of the curved world's
    3.0 m-wide fillet panels during `FollowPath` tracking (the robot's
    path specified a straight X line with no active Y correction),
    until its wheels left the panel edge. The FSM's own response (HALT
    firing and staying latched while genuinely still unstable) was
    working exactly as designed - not an FSM or HALT-recovery bug.
  - `construction_site.sdf`'s turnaround maneuver (phase 8 session A/B)
    generalized this finding much further: the same lateral divergence
    reproduces on a flat, single-level elevated platform with zero
    fillet seams, during a turnaround alone, before the robot even
    reaches the ramp - ruling out "fillet-seam compounding" as the
    actual mechanism. Root-caused by testing two structurally different
    turnaround maneuvers and finding both fail the same way: an
    open-loop in-place spin (wheel odometry reported landing within
    ~1-4° of the 180° target, but ground truth implied a true heading
    error of 35-40°+; trusting the IMU instead of odometry afterward
    made it *worse*, the two sensors disagreeing by a further ~40° on
    one run) and, after removing the open-loop spin entirely, a single
    continuous closed-loop `FollowPath` arc-turn (same failure -
    odometry tracked a plausible-looking arc while ground truth showed
    the robot walking off the platform edge, 6.27 m lateral, the whole
    time). This confirmed the real mechanism: it was never open-loop
    vs. closed-loop control - `FollowPath`/MPPI closes its loop
    continuously either way, but against wheel odometry, and wheel
    odometry's YAW is corrupted by wheel slip under the differential
    wheel speeds ANY turn requires here. A controller can execute its
    control law correctly and still walk the robot off a physical edge
    if its localization input is wrong.

  **Fixed** with `robot_localization`'s `ekf_node`
  (`ramp_autonomy_nav/config/ekf.yaml`), fusing wheel odometry's
  position + linear velocity with the IMU's orientation + angular
  velocity into `/odometry/filtered`, which now feeds
  `controller_server` (`nav2_params.yaml`'s `odom_topic`) and owns the
  `odom -> base_footprint` TF (`ramp_world.launch.py` no longer bridges
  gz-sim's own raw TF, to avoid two publishers of the same transform).
  Isolated validation BEFORE trusting the real round trip
  (`validation/phase8/ekf_validation/README.md`) confirms this is
  actually accurate, not just running, per this project's own repeated
  lesson that "active and plausible-looking" does not mean "correct":
  yaw error peaks at 3.25° during the exact spin that used to produce
  35-40°+ of error, and position error does not grow with distance
  traveled afterward. (One real EKF config bug was caught and fixed
  along the way, not just a test-harness issue: the first attempt also
  fused wheel odometry's own absolute x/y position, which re-imports
  the same yaw-under-slip corruption one step removed even with
  accurate fused orientation - fixed by fusing only wheel odometry's
  velocity and letting the EKF integrate position from its own fused,
  IMU-corrected orientation.) Rerunning the construction-site arc-turn
  with the EKF live (twice, for reproducibility) completed the full
  physical round trip both times with no safety abort, pitch settling
  at the correct -16° on the ramp (vs. 57.3°/tipping before), and
  stability margin never dropping below 0.872 (vs. 0.305 right before
  the abort, before). Numbers: `validation/phase8/
  construction_site_descent/README.md`; `docs/construction_site_design.
  md`'s "Live traversal-test results" section has the summary.

  **A second, distinct finding surfaced by this rerun, not expected to
  be fixed by this EKF and now diagnosed (using the two existing runs'
  own logs, no rerun needed)**: reproducibly (2/2 runs), the robot's
  actual position and pitch freeze completely for ~58-62 seconds right
  at the ramp-top fillet transition - genuinely stuck, confirmed not a
  monitoring artifact - while the fused estimate keeps advancing because
  it trusts wheel velocity unconditionally and the wheels are still
  being commanded. Checked `/wheel_slip` during both stall windows
  first, since an elevated reading would mean this is a wiring gap (a
  signal that exists but isn't connected to a response) rather than a
  sensing gap: **slip was NOT elevated in either run** (max 0.233 and
  0.210 - both near the lower edge of `descent_brake_node.py`'s own
  "ordinary descent slip" band, nothing like a genuinely spinning wheel).
  Root cause: `stability_monitor_node.py`'s `_compute_wheel_slip()`
  compares commanded vs. actual WHEEL JOINT rotation only
  (`/joint_states` vs. kinematically-derived command) - it has no
  connection whatsoever to whether that rotation produces real chassis
  displacement, so a chassis genuinely wedged/stuck while its wheels
  keep turning close to commanded (which the evidence points to here -
  `/odometry/filtered` kept advancing smoothly the whole time, only
  possible if the underlying wheel-derived velocity reading was itself
  consistently non-zero) is invisible to it by construction. Same for
  `/stability_margin` - pitch stayed a stable, non-tipping -5.12° the
  entire stall, so the tip-over margin model had nothing to flag either.
  **This is a genuine architectural blind spot, not a wiring bug**: no
  existing signal in this project can distinguish "wheels rotating and
  the chassis progressing" from "wheels rotating and the chassis stuck."

  **Since implemented as a real, live safety feature, not just a design
  proposal.** `motion_stall_detector_node.py` (`ramp_autonomy_control`)
  runs as part of the normal launch graph now, wired into
  `state_machine_node.py`'s HALT logic as a third independent trigger
  alongside margin and slip. LiDAR scan-consistency is the primary,
  confirming signal - a HYBRID of both LiDARs (prefers `/scan_360`,
  falls back to `/scan`), not a single fixed choice: `/scan_360` alone
  returns 0/360 valid beams on the actual open platform where the stall
  happens (too open for its 12m range in every direction); `/scan`
  alone fixes that but causes chronic FALSE stalls during ordinary flat
  driving (a beam pointed at a large uniformly flat floor reads nearly
  the same range whether the robot has translated 0m or 10m along it).
  A short-window IMU forward-accel check is a non-authoritative early
  warning only (logged, not wired to force anything - an in-place turn
  legitimately shows zero forward accel and would otherwise false-
  positive it).

  Three real bugs were found and fixed by testing against the actual
  failing scenario, not assumed correct from design: (1) a single noisy
  near-zero `cmd_vel` tick reset the "commanded for the full window"
  timer, so the first version never fired even once across ~90s of
  continuous driving with a genuine stall in the middle - fixed with a
  0.5s debounce; (2) fixing that revealed a HALT-thrash feedback loop -
  `cmd_vel_watchdog_node.py` correctly zeroing the actuation topic
  during HALT un-satisfied the detector's own commanded-motion gate,
  clearing the signal, letting the FSM recover, and repeating (40 HALTs
  in one descent leg) - fixed by latching the confirmed-stall signal;
  (3) the latch fix, tested against the real loaded scenario, revealed
  a WORSE problem - clearing the latch only on evidence of real
  movement is circular once HALT is correctly holding the robot still
  (there is no way to ever observe the movement being waited for), and
  one HALT never recovered for over 700s straight on ordinary flat
  ground with nothing physically wrong - fixed with a 10s cooldown that
  auto-clears the latch regardless of scan evidence, giving the FSM a
  bounded (~12-15s) retry cycle instead of a silent permanent deadlock.

  **Rerun live against the actual loaded construction-site stall**: the
  failure is now correctly and repeatedly flagged - `/ramp_state`
  reaches and re-enters `HALT` 94 times during the descent leg (vs. 0
  before), and the leg ends in `state=HALT` with the fused position far
  short of its target, not a false `EXIT`/"success". **A follow-up
  session then closed the remaining gap: the detector made the stall
  visible and safe, but nothing yet stopped a client from accepting
  `FollowPath`'s own "succeeded" result at face value once the fused
  position eventually drifted into tolerance.** Fixed with an
  independent goal-verification signal - see the paragraph below and
  "Audit table: which past results can be trusted" for the rerun result.
  Separately checked whether this loaded stall is the same mechanism as
  the sharp-world breakover finding (a real, physical ground-clearance
  interference) recurring at a fillet under load - it is not: pitch
  stays flat at 0.0 deg the entire freeze (the sharp-world case showed a
  clean, sustained -16 deg wedge) and margin/slip showed zero
  disturbance signature *at the fused-position freeze reported by the
  test's own summary logging* - a genuinely different symptom profile
  from the sharp-world constraint.

  **Root cause since found and fixed, with direct evidence - not the
  closest-fitting story.** The claim above that the frozen position sat
  "no seam nearby at all" was wrong, and finding out why is what closed
  this: a follow-up session instrumented per-wheel commanded-vs-actual
  velocity (`/joint_states`, real physics, via a new standalone
  observer) during a fresh repro. At the freeze, commanded wheel
  velocity was substantial (~0.5 m/s equivalent) and actual wheel
  velocity matched it almost exactly - the wheels were genuinely
  spinning at commanded rate - while the chassis's full 6-DOF
  ground-truth pose stayed completely frozen, and `/wheel_slip` read
  ~0.0 throughout that exact window. This is mechanism (c) - wheels
  spinning, chassis not translating - and it directly confirms a
  concern flagged two sessions earlier: `/wheel_slip` compares commanded
  wheel rotation to *actual wheel rotation*, never to chassis
  displacement, so a wheel spinning freely against an obstruction it
  cannot climb is invisible to it by construction.

  Cross-referencing the exact frozen coordinate against
  `construction_site.sdf`'s own collision geometry (precise box corners
  from pose/size/pitch, not "is it near a fillet") found a real,
  isolated defect: six of the eight seams in the
  `ramp_fillet_bottom_*`/`ramp_mid`/`ramp_fillet_top_*`/`elevated_level`
  chain match to within 1.1 mm (the intended, tangent-continuous
  fillet curve); the join into `elevated_level` and, symmetrically, the
  join into `laydown_area` at the ramp's bottom, were each off by
  almost exactly 0.05 m - an SDF authoring bug (these flat slabs' pose.z
  was set to the intended top-surface height without accounting for SDF
  box poses being the geometric *center*), not a physical/design
  constraint. The robot's frozen front-axle position matched the
  theoretical first-contact distance for a 0.12 m wheel meeting a
  0.05 m step almost exactly. Only the top seam caused a live failure -
  it's reached mid-turn under active MPPI steering correction with a
  forward-loaded payload pressing into the step's face, unlike the
  same-sized bottom seam, crossed calmly in a straight raw-`cmd_vel`
  drive.

  **Fixed in `construction_site.sdf`** (`elevated_level`'s z corrected
  3.2500 -> 3.3000 m; `laydown_area`/`access_road`'s z corrected
  0.0500 -> 0.1000 m) - every seam now matches to within 1.1 mm.
  **Reverified live, full round trip**: the robot no longer freezes,
  `FollowPath` genuinely reports success, leg 4 runs and completes back
  at the gate, and the descent is clean while actually on the ramp
  panel (margin never below 0.463 against a 0.15 HALT floor, slip near
  zero, lateral drift under 0.6 m against a 2.25 m panel half-width). A
  separate, already-documented turn-quality roughness (more HALTs and
  transient margin/slip dips during the arc-turn and flat-ground legs,
  tied to the existing `PathAlignCritic`/MPPI-stochasticity limitations
  below, not this mechanism) is still present and was not newly
  introduced by this fix - **since diagnosed and partially fixed, see
  the entry below.** Full diagnosis, the exact telemetry, the seam
  geometry table, and the reverification data:
  `validation/phase8/construction_site_descent/README.md`.

  **Crosstrack diagnosis: why the earlier partial fix (`offset_from_
  furthest` 20 -> 3) didn't fully close the gap, and a second fix that
  actually does for the arc-turn.** Triggered by a real regression
  signal: a loaded construction-site run logged `-11.26 m` of leg-4
  drift, nearly double the previously-recorded `-5.92 m` worst case.
  Root-caused with live instrumentation (an external proxy for
  `PathAlignCritic`'s own activation gate, reconstructed from its real
  source in the local nav2_mppi fork, cross-checked against actual
  ground-truth cross-track deviation) rather than re-tuned by guessing:
  **the earlier fix only lowered `descent_mode`'s
  `path_align_offset`** (used once `/ramp_state` reaches `DESCENT`) -
  **`flat_mode`'s was never touched**, and the entire arc-turn maneuver
  runs under `flat_mode` (`EXIT` state - the arc happens on a flat
  elevated platform, pitch stays 0 deg the whole time, so `on_ramp`
  never fires until well after the turn ends). With `flat_mode`'s
  `offset_from_furthest` still at 20 and the arc itself only 16 path
  points long, `PathAlignCritic` was **fully disabled for the entire
  arc-turn, confirmed live** (proxy first activates at path index 20,
  ~82 s in - matching the source-level math exactly, identically timed
  regardless of payload). Also confirmed: this is a genuinely different
  mechanism from the curved-world edge-departure finding (a real
  physical seam stumble, occurring even with an *active* critic) and
  from the arc-turn's own HALT-thrash (driven entirely by wheel-slip/
  motion-stall triggers per direct log evidence, never by anything
  crosstrack-related) - three distinct causes previously grouped
  together as one "crosstrack weakness," now checked side by side and
  found not to be. **Fixed**: `flat_mode.path_align_offset` lowered
  20 -> 3 in `ramp_mppi_params.yaml`, matching `descent_mode`'s
  already-validated value. Verified same-day, same repro: arc-phase
  mean cross-track deviation 0.218 m -> 0.090 m (-59%), and a full
  official-protocol reverification's leg-4 drift down to 0.76 m - the
  smallest this project has logged for this maneuver, loaded or
  unloaded, before or after any fix. **A real side effect this
  surfaced**: the smaller, better-behaved post-fix drift threads
  through a different part of the access road than the large,
  blow-past drift last session's props were margined against - one
  post-fix run reproduced the elevated-level freeze's exact "wheels
  spinning, chassis frozen, fused position running away" signature,
  0.58 m from `gate_cone_2`'s center. Fixed by removing the at-risk
  gate-area props rather than re-guessing a margin off 2 data points;
  see `validation/phase8/construction_site_descent/README.md`'s
  "Crosstrack diagnosis" section for the full instrumentation, every
  pre/post-fix number, and why access-road prop placement needs leg 4's
  own open-loop drift bounded before anything goes back there.

  **Closed in a follow-up session ("Drift characterization")**: ran 8
  independent, fresh-launch trials of leg 4 per payload config instead
  of re-guessing off 2 points. Loaded drift (the binding case)
  converged cleanly by n=6-8: min=0.526 m, max=2.535 m, mean=1.263 m,
  sample stdev=0.695 m across n=8 - a real, stable distribution (min/
  max/range unchanged from n=6 onward, not still swinging), but
  inherently wide (~5x spread between smallest and largest trial, ~55%
  coefficient of variation). Unloaded drift was tight (n=4 valid,
  min=-0.203 m, max=0.005 m). **Correction**: this entry first said
  half the unloaded attempts "never completed the arc-turn" (a "real
  leg-3 stall-rate finding"). That was measured while the trial harness
  leaked ~57 stale ROS/gz processes (load average 71-76 on 12 cores) and
  does not reproduce on a clean host - see "Leg-3 completion recheck"
  in `validation/phase8/construction_site_descent/README.md`. The drift
  numbers were collected while that leak was building and carry the
  same caveat. **Clean-host result: the arc-turn completed 15/16 planned
  trials (93.8%, Wilson 95% 72-99%; unloaded 8/8, loaded 7/8); the one
  failure was a harness timeout at 240 s while descending normally, and
  the cap is now 360 s.** A clean-host loaded drift re-collection (n=7,
  mean 0.81 m, max 1.38 m) is not statistically distinguishable from the
  original (mean 1.26 m, max 2.54 m; p=0.15), so the prop decision
  stands, though it hangs on a max-of-n statistic (see the validation
  README's caveat). **Clean-host regression pass (one run each, harness-guarded, none starved):** the 7-cell matrix gave 2 x FAIL: DID_NOT_CLIMB (the known unrelated sharp-world loaded constraint, unchanged), 1 x FAIL: HALT_THRASH (curved/normal/unloaded, whose one-off 1.01 m ground-truth pass turned out to be an outlier - see the OPEN FINDING at the top of this section), 4 x MARGINAL. Both construction-site round trips (loaded and unloaded) completed leg 3 (`ok=True`, 102% verified) and returned to the gate with no abort; the loaded run's 1.572 m leg-4 drift makes the prop decision hold even on clean-host data alone. Full table and caveats: validation README, "Clean-host regression pass". **Stall detector: fixed for rotation.** The leg-3 platform phase was ~10x slower than the arc needs because the LiDAR stall detector latched a >=10 s HALT on a robot that was moving (on the open platform its only working sensor sees the robot's own body and the flat floor). Built a positive-control test (a low cage spawned around the moving chassis, scored against Gazebo ground truth) and added a gyro-integrated rotation check. Against ground truth: false latches while rotating 14 -> 0, leg-3 time spent stall-latched 50.2% -> 27.4%, leg-3 duration 185 s -> 123 s (6/6 completed), forced freezes detected 18/18 (95% interval 82-100%; 18/20 under a stricter reading, both misses pre-existing). Still open and physical: false latches during *straight* travel (~12 per session), because no inertial sensor can tell steady straight motion from a blocked chassis, and the sim IMU has no noise (a real gyro needs bias subtraction; masked freezes at 0.05 rad/s bias). Regression rerun on the fixed detector (matrix + both round trips, one run each, guarded): both round trips completed with fewer HALTs (loaded 24 -> 14, unloaded 40 -> 30); six of seven matrix cells match the earlier verdicts; the seventh, `curved/normal/unloaded`, is deliberately not counted among the matches and is tracked as its own OPEN FINDING at the top of this section (fails ground truth in 7 of 8 runs). **Known adverse effect of the fix, now worked around:** applied everywhere it regressed `curved/normal/unloaded` (0/6 -> 10/10 ground-truth failures); the rotation check is therefore switched off during `DESCENT` (a scoped workaround that only partly restores the original, 2/6 still fail; see the first entry of this section). See the validation README, "Stall detector: positive control, fix and validation". Applying this project's own established margining
  convention (max relevant excursion + 1.5 m buffer, same formula used
  elsewhere in `construction_site.sdf`): required clearance = 2.535 +
  1.5 = 4.035 m from the leg-4 centerline, which exceeds
  `access_road`'s own half-width (3.05 m of its 6.1 m total) - **no
  position on the road can satisfy this margin**, converged distribution
  or not. The removed gate-area props stay removed **permanently**, a
  structural limitation of the road width against this maneuver's own
  characterized worst case, not an open question awaiting more data.
  Full trial data, the convergence table, and the leg-3 stall finding:
  `validation/phase8/construction_site_descent/README.md`'s "Drift
  characterization" section; raw CSVs in
  `crosstrack_diagnosis_flatmode/drift_characterization/`.

  **Follow-up audit, before populating the site with props**: checked
  every other flat-box-to-flat-box and flat-box-to-ramp join in
  `construction_site.sdf` for the same box-center-vs-top-surface
  authoring mistake, with the same precise corner-math method (not
  visual inspection) - all 8 wall-to-pad bases, all 5 perimeter walls
  against `ground_plane`, and the one other floor-to-floor adjacency
  (`access_road`/`laydown_area`, already fixed). **No other instance
  found** - every other join matches exactly (dz=0.0000 m for every
  wall base checked). `elevated_level`/`office_pad`/`waste_pad` don't
  geometrically border any other flat slab at all, so there was nothing
  else to mismatch. No further world-file change was needed. See
  "Follow-up audit" in the same construction_site_descent README for
  the full per-seam table.

  **Site populated with real Fuel props, reverified clear of the path**:
  22 real Gazebo Fuel assets added (queried and dimension-checked
  before use, not assumed to exist - barriers/pallets/crates/a rack/a
  40 ft-container office/a dumpster/cones/fencing), zoned per the
  original design doc's own real-world logic (delivery-to-point-of-use
  flow in the laydown area, gate-flanking fencing, etc.). Every
  placement checked two ways before being written: exact-footprint
  overlap against every existing model (0.3 m margin) and distance from
  all 1197 real ground-truth points logged across every previously-saved
  verification run (1.5 m buffer, minimum clearance found: 2.0 m). A
  real bug was caught this way, not assumed away: several Fuel models
  default to `<static>false</static>` (SDF's own default when the tag
  is omitted) unlike every hand-authored model in this world - one prop
  was found to have fallen under gravity in a smoke test before the fix
  (explicit `<static>true</static>` added to all 22 includes).
  **Reverified live, full round trip on the populated world: `ok=True`**
  - same clean numbers as the unpopulated world (margin never below
  0.463, slip near zero, lateral drift under 0.5 m while on the ramp
  panel). Two transient instabilities hit while verifying this were
  root-caused and confirmed unrelated to the props, not quietly
  papered over: a real-time-factor drop traced to this session's own
  leftover processes (not the props - a clean isolated A/B test showed
  only a ~1% physics-cost difference), and an already-razor-thin
  180 s test-script timeout (the one saved pre-props successful run had
  already used 179.0 of its own 180 s budget) raised to 240 s. Full
  method, the real bug, and the reverification data:
  `validation/phase8/construction_site_descent/README.md`.

  **Audit requested alongside this fix, since actioned**: of this
  project's past validated results involving turning or lateral motion
  (not pure straight-line ramp ascent/descent), which were verified
  only against wheel-odometry expectation rather than Gazebo ground
  truth? `validation/phase8/validation_test.py` - the main Phase 8
  validation matrix (`run_matrix.sh`), covering every cell in the "Full
  matrix: 2 worlds x payload x friction" results below - performed the
  open-loop turn then a `FollowPath` descent for every cell, with its
  `descent_verdict` computed entirely from `/ramp_state` reaching
  `DESCENT` plus a wheel-odometry-derived `descent_leg_distance_m`, and
  never polled Gazebo ground truth anywhere in the script. By contrast,
  `curved_descent_diagnosis.py` (which is how the curved-world finding
  above was originally root-caused) and `construction_site_descent`'s
  own scripts DO poll ground truth throughout, so their results stood
  as-is. `watchdog_test.py` and `loaded_climb_diagnosis.py` involve no
  turning or lateral motion at all and are unaffected regardless.

  **The matrix was rerun** with the EKF live and ground-truth polling
  added (`validation_test_ekf_groundtruth.py`, preserving the original
  script and its results unmodified as a labeled historical record -
  see "Full matrix, corrected" below the original table). Result: 3 of
  7 cells changed verdict, all three from MARGINAL to FAIL, none
  improved - two (`curved/normal/unloaded`, `curved/normal/loaded`)
  because ground truth caught the robot's real position leaving the
  ramp panel (up to 2.71 m past a 1.5 m half-width) while `/ramp_state`
  still reported a clean `DESCENT`, confirming the audit's concern was
  justified rather than hypothetical; the third
  (`curved/0.45/loaded`) because HALT count jumped to the thrash
  threshold (8, vs. 3 before) for a reason not yet root-caused. Full
  comparison table and per-cell reasoning below.

  **Both new findings above were then root-caused directly (instrumented
  live, read the actual `nav2_mppi_controller` source rather than keep
  guessing) - see `validation/phase8/crosstrack_diagnosis/README.md` for
  the full story.** Cross-track error: confirmed growing unchecked (0.44
  -> 2.85 m, smooth and monotonic) in a live instrumented run. Three
  tuning attempts on `PathAlignCritic`'s weight/power all failed to fix
  it - the actual root cause, found by reading the critic's own source,
  is that `offset_from_furthest` (static default 20) exceeds this
  project's entire ~15-point descent path, so the critic was
  structurally disabled the whole time on every attempt; every weight/
  power change was tuning a critic that never activated once. Fixed
  (`offset_from_furthest` dropped to 3, `ASCENT`/`DESCENT` split into
  separate MPPI profiles so ascent's proven-clean behavior is untouched)
  - genuine, measurable improvement (2.85 -> 2.03 m), but **not a full
  fix**: a second, independently-confirmed mechanism (a real physical
  stumble at `ramp_world_curved.sdf`'s own `fillet_top_1/2/3` seam
  cluster, X=7.6-8.3 m - confirmed by direct correlation with the
  world's own geometry, reproducing at that exact location across every
  run regardless of MPPI tuning) contributes the remainder and needs a
  different category of fix, not more critic tuning. HALT-thrash:
  correlates temporally with the same seam-stumble episodes; threshold
  mismatch and EKF fusion-rate/timing mismatch both explicitly checked
  and ruled out (no config changes to either, no timing anomalies in any
  log) - genuine, previously-invisible instability, amplified for loaded
  cells by their reduced margin, not a bug in either mechanism. Full
  7-cell matrix rerun with the fix live: 2 cells genuinely improved (one
  to a full PASS), 2 remain FAIL for the confirmed seam-stumble reason,
  1 unaffected as expected - see "Full matrix, corrected: crosstrack fix
  applied" below, the matrix to trust going forward.

  **The construction site's own ramp uses the same fillet-transition
  design as the curved world, so its loaded-payload behavior was
  checked too - and the same fillet-top mechanism found here does not
  just reappear there, it is dramatically worse.** With a 100 kg/+0.1 m
  CG payload, `construction_site.sdf`'s round trip reported a clean
  "success" (`aborted: False`, all 4 legs completed) - but ground-truth
  polling shows the robot's real position and pitch froze at the
  ramp-top transition (X~41.3 m, elevated-platform height) from
  roughly a third of the way into the test onward and **never moved
  again for the rest of the ~270 s run**, including through what the
  test logged as a completed arc-turn, ramp descent, and return to the
  gate. The entire remainder of the "successful" round trip was
  fictional: `/odometry/filtered` kept advancing because it integrates
  wheel velocity, which stayed consistent with the fused position the
  controller itself believed, in a closed loop entirely divorced from
  the robot's real, frozen position - the same architectural blind spot
  as the fillet-top stall finding above, but this time the robot never
  broke free at all within the test's full runtime, unlike the unloaded
  case's ~60 s self-recovery. **Session B should not build on top of an
  assumption that loaded traversal of this ramp works - it currently
  does not, and the test harness's own success signal cannot detect
  the failure.** Full data: `validation/phase8/construction_site_
  descent/loaded/`.

  **Goal-completion check fixed, loaded round trip rerun, and every
  past result named above actually classified (not just flagged) - see
  the "Audit table: which past results can be trusted" section below,
  right before the project status summary.** Short version: `FollowPath`'s
  own goal-completion check now has a second, independent requirement
  before a leg counts as complete (`/verified_progress_m`, wheel-rotation
  distance gated on the LiDAR scan-consistency check never having
  latched) - see `motion_stall_detector_node.py` and
  `validation/phase8/construction_site_descent/README.md`. Rerun with
  this live: the loaded round trip still does not complete the descent
  (real position freezes at the identical X~41.35 m location, to three
  decimal places, as the original finding) - the new gate correctly
  refuses to accept the leg (`ok=False`), and the existing HALT ->
  pre-halt-state recovery alone (no new "back up and retry" strategy was
  added - none was justified by this evidence) never gets the robot
  unstuck within the test's runtime. This is now an honest FAIL, not a
  fictional PASS.

## Audit table: which past results can be trusted

Requested alongside the goal-verification-gate fix above, once the
construction-site stall proved this project's "FollowPath declares
success while the chassis is physically frozen" failure mode was real,
not hypothetical: go through every past result that could plausibly be
exposed to the same mechanism (or a related one) and actually classify
it, rather than leave it flagged as "should probably be rechecked
someday." **This is the reference to check before citing any specific
number from this project's history - not a one-time note, a standing
table.** Three categories, used consistently below:

- **(a) unaffected** - the test involved no turning and no
  FollowPath/goal-completion ambiguity (straight-line driving, or a
  criterion read directly off a sensor/actuation topic rather than
  integrated position). Safe to trust as reported.
- **(b) affected but already reverified** - the original test *could*
  have been fooled by this mechanism, but a later, ground-truth-checked
  session already reran it and its result stands on that later evidence,
  not the original odometry-only one.
- **(c) genuinely unverified** - would need a real rerun (with ground
  truth or the new verified-progress gate) before citing it as trusted;
  not done in this session.

### Table 1: `validation_test.py`'s original 7-cell matrix

The "Full matrix: 2 worlds x payload x friction" table (original,
odometry-only, `matrix_logs/`/`matrix_corrected_logs/`) has been
superseded by two later reruns of the same 7 cells:
`validation_test_ekf_groundtruth.py` first with EKF + Gazebo
ground-truth polling added (`matrix_ekf_groundtruth_logs/` - "Full
matrix, corrected: EKF-fed controller + Gazebo ground-truth polling"),
then again after the crosstrack fix (`matrix_ekf_crosstrackfix_logs/` -
"Full matrix, corrected: crosstrack fix applied (the trusted matrix
going forward)"). Both reruns poll Gazebo's real model pose
continuously through the descent leg, not just odometry/FSM state.

| Cell | Original verdict (odometry-only) | Trusted verdict (ground-truth reruns) | Classification | Why |
|---|---|---|---|---|
| sharp / normal / unloaded | MARGINAL (4 HALTs) | **PASS** (0 HALTs, max\|y\|=0.09 m) | **(b) reverified** | Ground truth clean in both reruns; the MARGINAL->PASS change came from the crosstrack fix (a real, unrelated MPPI bug fix), not from ground truth catching a problem. |
| sharp / normal / loaded | FAIL: DID_NOT_CLIMB | FAIL: DID_NOT_CLIMB (unchanged, both reruns) | **(a) unaffected** | Ascent-only failure - IMU pitch pegged at -16.0 deg for the whole budget; never reaches the turn or a `FollowPath` goal at all. Root-caused separately as a genuine physical ground-clearance constraint (see "Root-causing the loaded-payload climb failure"). **[CORRECTED 2026-09-20: it was a 0.05 m geometry authoring step at the crest; fixed, 12/12 now climb and descend - see `validation/phase8/crest_freeze/README.md`.]** |
| sharp / 0.45 / loaded | FAIL: DID_NOT_CLIMB | FAIL: DID_NOT_CLIMB (unchanged, both reruns) | **(a) unaffected** | Same as above. **[CORRECTED 2026-09-20: same 0.05 m step; fixed, 12/12 climb.]** |
| curved / normal / unloaded | MARGINAL ("PASS", 1.27 m of ~6 m) | **FAIL: edge departure** (max\|y\|=2.71->2.68 m, past the 1.5 m half-width) | **(b) reverified, verdict corrected** | This is the concrete proof the concern was real: the original odometry/FSM-based verdict was wrong, and ground truth is what caught it - the robot's real position left the ramp panel while `/ramp_state` still reported a clean `DESCENT`. |
| curved / 0.45 / unloaded | MARGINAL ("PASS", 5.50 m) | MARGINAL (max\|y\|=0.39->0.36 m, clean) | **(b) reverified** | Held up under ground truth in both reruns - genuinely clean, not just odometry-plausible. |
| curved / normal / loaded | MARGINAL ("PASS", 1.14 m of ~6 m) | FAIL (2.33 m departure) -> **MARGINAL** (0.97 m, clean) after the crosstrack fix | **(b) reverified, verdict corrected then improved** | Ground truth first caught a real departure the odometry-only run missed entirely, then confirmed a genuine fix. |
| curved / 0.45 / loaded | MARGINAL ("PASS", 1.15 m of ~6 m) | **FAIL: HALT_THRASH** (8->9 HALTs, ground truth clean both times) | **(b) reverified** | Ground truth rules out edge departure here - the real cause is the separately-confirmed physical stumble at `ramp_world_curved.sdf`'s `fillet_top_1/2/3` seam cluster (X=7.6-8.3 m), not a fusion/threshold bug. |

**Two open caveats that apply to the "trusted" reruns too, not just the
original matrix** - the ground-truth fix closed one gap, not every gap
of this shape:
1. Every generation of this matrix (original and both ground-truth
   reruns) turns 180 degrees with the same **open-loop, dead-reckoned
   `drive(0.0, 0.5, 6.6)` spin** - the exact mechanism later found, in
   `construction_site_descent`'s own testing, to produce ~35-40 deg of
   real heading error with no way to detect it while it's happening (see
   that section's v6 rewrite to a closed-loop `FollowPath` arc-turn).
   This matrix was never rebuilt on that fix, so heading accuracy into
   each descent leg here is itself unverified.
2. `validation_test_ekf_groundtruth.py`'s ground-truth check is
   **lateral-only** (max\|Y\| against the panel half-width) - it would
   not have caught a construction-site-style freeze that stays within Y
   bounds the whole time (the actual construction-site stall sat at
   Y~-0.2 to -0.3 m throughout, well inside \|Y\|<1.5 m). The two
   ground-truth mechanisms (lateral-departure check here,
   verified-progress gate in `construction_site_descent`) catch
   different failure modes, not a strict superset - none of these 7
   cells has been rechecked against the newer verified-progress signal.

### Table 2: Phase 6/7 claims

The scripts that originally produced these numbers are not preserved in
this repo (unlike phase 8's diagnostic scripts, which were saved on
purpose) - classification below is based on the methodology actually
documented in the "Wheel slip & safety hardening (phase 6)" / "Controlled
descent braking (phase 7)" / "Phase 7, continued" sections above, plus
(for `watchdog_test.py`) reading the surviving script directly this
session.

| Claim | Classification | Why |
|---|---|---|
| Phase 6 wheel-slip live test (mu=0.12 ramp, slip up to 2.08, HALT firing correctly) | **(a) unaffected** | Straight-line, raw-`cmd_vel` driving only - no turn, no `FollowPath` goal anywhere in this test. `/wheel_slip` is a direct commanded-vs-actual wheel-joint comparison; no integrated position in the loop at all. |
| `vx_max`-capping feedback spiral (146 HALTs, 76.44 s, max slip 11.148) - mechanism and HALT count | **(a) unaffected** | HALT count comes from real-time threshold checks against live `/stability_margin`/`/wheel_slip` topics (not `FollowPath`'s goal-completion, not fused position). The spiral mechanism itself is corroborated by direct log evidence (`slip=1.16 -> vx_max 0.13 -> 0.05`), not inferred from position or distance. |
| Same run's reported **distance** (5.50 m, identical to the baseline's 5.50 m despite 76.44 s vs. 17.01 s and 11.148 vs. 1.000 max slip) | **(c) genuinely unverified** | The identical figure across wildly different runs strongly suggests this was a fixed target/stopping distance, not an independent per-run measurement, tracked via pre-EKF raw `/odom` (wheel dead-reckoning, already documented since phase 6 as unreliable under real slip) during a run with the highest slip ever recorded in this project. The original script no longer exists to check exactly how it was used - treat this specific number, not the spiral finding, as unverified. |
| `ax_max`-capping "statistically indistinguishable from doing nothing" (17.77 s vs. 17.01 s, identical 1.000 max slip) | **(a) unaffected** for timing/slip; **(c) unverified** for the paired 5.50 m distance figure | Same split and same reasoning as the row above - timing and slip come from direct topic/log readings, the distance figure carries the same pre-EKF-odometry caveat. |
| Descent-braking slip-band tuning (`SLIP_BRAKE_THRESHOLD=0.25`, `SLIP_RELEASE_THRESHOLD=0.08`) | **(a) unaffected** | A static parameter choice derived directly from phase 6's `WHEEL_SLIP_HALT_THRESHOLD` (0.5), itself confirmed via the unaffected straight-line low-friction test above - not itself a live measured result, and does not depend on position or goal-completion. |
| `watchdog_test.py` (cmd_vel-watchdog HALT-to-zero timing) | **(a) unaffected - confirmed by reading the script this session**, not just inferred | Its sole pass criterion is elapsed time from the `HALT` publish to `/model/ramp_robot/cmd_vel`'s own `linear.x` reading exactly zero - a direct actuation-topic value check. No position, no `FollowPath` goal-completion anywhere in the script. |

**Not re-audited here** (out of scope for this session's request):
`curved_descent_diagnosis.py`, `loaded_payload_diagnosis/loaded_climb_
diagnosis.py`, and `construction_site_descent`'s own diagnostic scripts
were already confirmed ground-truth-polling in the audit that preceded
this table and are unchanged - see the paragraph above this section.

## Project status summary

Written for anyone doing a fresh read before a project review, not
just someone who followed along phase by phase.

**Fully works, end to end, verified live (not just unit-level):**
- The 5-state ramp FSM (`APPROACH -> ASCENT -> TRANSITION -> DESCENT ->
  EXIT`, plus `HALT` pre-emptable from any state) completes a full
  round trip - up the ramp, across the top, back down - driven for
  real through Nav2's `FollowPath`/MPPI, with no test-harness override
  of `/ramp_state` anywhere in phase 8's validation matrix.
- `ASCENT` is guaranteed to reach `EXIT` even if neither of its two
  upstream triggers (perception pre-emption, IMU pitch-rate spike)
  fires - the bug that originally motivated this phase's FSM work.
- HALT recovery returns to whatever state it actually interrupted, not
  unconditionally to `APPROACH` - fixed and verified this session,
  including under heavy repeated HALT churn (curved-world matrix
  cells).
- Live MPPI parameter switching (`nav2_param_switcher_node.py`) and
  live descent acceleration braking (`descent_brake_node.py`,
  `ax_max`/`az_max`-based) both confirmed actually landing on the
  running controller, not just believed-sent.
- Tip-over margin and wheel-slip HALT triggers both fire from real
  physics (confirmed via a genuine low-friction slip event and a
  genuine steep-pitch margin drop, not synthetic values), independent
  of each other.
- Unloaded traversal of both worlds (sharp and curved) at both normal
  (0.55) and reduced-but-climbable (0.45) ramp friction: confirmed
  reaching `EXIT` and `DESCENT` reliably (phase 8 matrix) - **with one
  correction**: `curved/normal/unloaded` reaching `DESCENT` is real, but
  the rerun with Gazebo ground-truth polling added
  (`validation_test_ekf_groundtruth.py`, see "Full matrix, corrected"
  below) found the robot's real position leaving the ramp panel (2.71 m
  past a 1.5 m half-width) during that same run - the original
  odometry-only check had no way to see that. `curved/0.45/unloaded` is
  unaffected and now genuinely ground-truth-confirmed clean (max
  \|y\|=0.39 m).
- `ramp_classifier_node.py`'s break-point distance estimate now
  correctly counts down through the final approach to a sharp corner
  (root-caused and fixed - see Known Gaps), verified end to end with
  the original gentle-burst driving style that first exposed the bug:
  `ASCENT -> TRANSITION` fires from genuine perception, not the
  IMU-rate fallback. Its slope estimate on the curved world's filleted
  transitions now resolves toward the true local onset angle instead of
  always converging to the ramp's peak angle (also fixed and verified).
- The robot has genuine 360° obstacle awareness now, on a sensor fully
  independent of ramp detection - verified live with real obstacles at
  their exact true bearing/range, actually marked in Nav2's costmap,
  with zero effect on ramp perception (see "Phase 8, continued").
- HALT is now enforced by two genuinely independent mechanisms, not
  one: the existing MPPI reconfiguration path, and
  `cmd_vel_watchdog_node.py`'s inline actuation-topic gate, which was
  itself live-tested with the reconfiguration path deliberately killed
  and confirmed to still stop the robot (ramped, at the platform's own
  rated deceleration) in ~120 ms regardless.

**Documented known limitations (see "Known gaps" above for the full
list with mechanisms):**
- The loaded payload configuration (100 kg, +0.1 m forward CG) cannot
  climb the sharp-world ramp at all, at any friction tested -
  root-caused to a genuine physical/design constraint (near-zero
  chassis ground clearance meeting a sharp, unfilleted break point;
  see "Root-causing the loaded-payload climb failure" in the phase 8
  section), not a software bug, and therefore not fixed by design. The
  same payload climbs the curved (filleted) world fine. **[REFUTED 2026-09-20: the cause was a 0.05 m geometry
  authoring step at the crest, now fixed - the loaded robot climbs and descends the sharp world 24/24 and every curved cell 24/24; see
  `validation/phase8/crest_freeze/README.md`.]**
- The curved world's descent legs consistently stall well short of
  their target distance (1.1-1.3 m of ~6 m in every curved matrix cell)
  with anomalous pitch readings in most runs - root-caused to a genuine
  physical tip-over from lateral drift off the edge of the curved
  world's narrow fillet panels (confirmed via Gazebo's own ground-truth
  pose/orientation; see "Root-causing the curved-world descent
  chatter"), not a perception, FSM, or HALT-recovery bug - verified by
  ruling those out first (the chatter was unchanged after fixing both).
  Not fixed, for the same reason as the loaded-payload finding: a real
  robustness limitation, not a one-line software patch. **[SUPERSEDED 2026-09-20: the robot never descended in those runs - the harness turned it
  on the curved crest (open-loop, ~90 of 180 deg, unseen 0.2-1.3 m sideways displacement) and the loaded cells were frozen on a 0.05 m geometry step.
  With the corrected turn procedure and the fixed geometry all 7 cells descend the whole ramp on the centerline; see `validation/phase8/path_structural/README.md` section 7 and `validation/phase8/crest_freeze/README.md`.]**
- `ramp_classifier_node.py`'s `breakpoint_distance_m` estimate used to
  stall near a sharp corner instead of counting down - root-caused and
  fixed (see Known Gaps): the near-corner point cluster genuinely has
  only 1-3 valid returns, and the old `MIN_SEGMENT_POINTS=5` made that
  cluster too small to ever be used. Fixed by lowering it to 3; the
  `ASCENT` completeness edge remains in place as defense in depth but
  should now rarely if ever be needed.
- The stability margin model is pitch-axis-only (no roll term yet);
  `nav2_param_switcher_node.py`'s failure handling has no independent
  enforcement path if `set_parameters` kept failing forever; the LiDAR
  can't do ground-plane fitting and full-surround obstacle detection
  at the same time. `/odom` drifting under real slip during turning is
  now fixed for the controller's own localization (EKF fusion - see
  Known Gaps), but a distinct, related gap remains: no signal in this
  project (not `/wheel_slip`, not the EKF) can detect a chassis that's
  genuinely stuck/jammed while its wheels keep rotating close to
  commanded - see the fillet-top stall finding in the same Known Gaps
  entry.
- The practical minimum climbable friction for this robot/ramp is
  somewhere in (0.35, 0.45], not pinned down more precisely than that
  bracket - meaningfully above the theoretical `tan(16°) ≈ 0.287`
  floor. (2026-09-20 note: measured on the geometry with the 0.05 m crest step; not re-measured on the fixed geometry.)

**Explicitly out of scope for this project as scoped so far:**
- No sim-to-real bridge - this has been a simulation project
  throughout, with hardware validation as an intentional later phase,
  not an oversight.
- No `planner_server`/`bt_navigator`/AMCL/map - there is still no
  waypoint-following goal source; every live test in this README drives
  the robot with a manual `/cmd_vel` publish or a hand-built
  `FollowPath` goal, not a real navigation stack goal. This is phase
  5's job ("slope-aware planning & control"), not started.
- No tuning pass on `ramp_mode`/`transition_mode`'s exact speed/
  heading-tolerance numbers - phase 3 only needed the profile-switching
  *mechanism* proven live; the numbers themselves are still
  reasonable-looking placeholders, not validated against a real
  requirement.
- `/cmd_vel_scale` has no consumer and may end up redundant now that
  the MPPI profile switch provides an equivalent planner-enforced
  limit - an open question for phase 5, not a gap in this phase's work.

## Licensing note

This log predates the repository's public release. Licensing is now governed by [LICENSE](../LICENSE) and
[THIRD_PARTY_LICENSES.md](../THIRD_PARTY_LICENSES.md) at the repository root, not by the line this section used to end on.
