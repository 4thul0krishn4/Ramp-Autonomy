# construction_site.sdf descent/turnaround finding

Ascent (gate -> laydown -> up the 16 deg ramp -> onto the elevated level)
is fully verified: 5/5 clean runs, zero ground-truth lateral drift, HALT
count 0, stability margin 0.87-0.91 throughout, pitch settling exactly at
-16.0 deg on the ramp and 0.00 deg on the elevated level.

The return leg (turn around on the elevated level, descend, drive back to
the gate) has not yet been achieved cleanly. Two structurally different
turnaround approaches were tried; both fail, for related but distinct
reasons.

## Attempt 1: open-loop in-place spin (superseded, not used going forward)

`turn_180`: command a fixed angular velocity (0.5 rad/s) for a fixed
duration (6.6 s), matching the exact maneuver used in the earlier Phase 8
diagnosis scripts (`validation/phase8/curved_descent_diagnosis/`,
`validation/phase8/validation_test.py`). After the spin, wheel odometry
reported a final heading of -175.9 to -178.7 deg across runs (close to the
180 deg target) - but the OBSERVED lateral drift on the subsequent
straight-line leg implied a true heading error of 35-40+ deg. Reading
heading from the IMU instead (a real onboard sensor, not wheel-encoder
dead reckoning) did not fix it either - on one run the IMU read 135.8 deg
immediately after the same spin, a further ~40 deg disagreement from
wheel odometry, and actively steering off that reading made the drift
*worse* (the robot ended up outside the site perimeter entirely, at world
x=-15).

**Finding, kept as a documented platform limitation regardless of the
deeper diagnosis below:** an in-place spin at this platform's default
angular rate (0.5 rad/s) is not a reliable maneuver on this world/robot
combination - it demands maximum rotational force against static friction
with zero forward velocity (close to worst-case for inducing wheel slip),
and it is never corrected by closed-loop feedback while it's happening
(feedback is only consulted afterward, once the error is already
committed). This matches the project's own established pattern: every
OTHER maneuver that has held up under testing in this project routes
through Nav2's closed-loop FollowPath/MPPI controller with continuous
live correction; the scripted in-place spin was the one exception, and it
is the one maneuver that has proven unreliable.

## Attempt 2: closed-loop FollowPath arc-turn (`arc_turn_followpath_test.py`)

Per the above finding, the open-loop spin was removed entirely rather
than patched further. The turnaround is now part of ONE continuous Nav2
`FollowPath` goal: a semicircular arc (radius 2.0 m, using the elevated
level's own open space - the platform is 15 x 12 m,
`construction_site.sdf` pose `x=48.47 y=0 size=15x12` -> spans
X:[40.97,55.97] Y:[-6,6], comfortably larger than the arc's 4 m lateral
extent and 2 m forward bulge), tapering back onto the ramp's Y=0
centreline, continuing straight across the ramp itself. Same MPPI
controller that produced the clean, zero-drift ascent; no separate
open-loop maneuver anywhere in the leg.

Result: still fails. `arc_turn_run.csv` / `arc_turn_run.log` (this run,
the one referenced in the design doc's round-trip numbers) show the
robot's OWN wheel-odometry position tracking a plausible-looking arc (X
peaks around 48.8, Y rises to ~3.7 then correctly tapers back down toward
~1.5-1.8, matching the planned path shape) while ground-truth Gazebo pose
tells a completely different story: `gz_y` climbs *monotonically* for the
entire maneuver (0.0 -> 6.27 m) and never turns back down, diverging
further from the odometry belief every sample. The robot physically
walked off the elevated level's Y-edge (platform half-width 6 m) while
its own odometry-fed controller believed it was correctly re-centering
onto the ramp. A safety-abort watchdog (cancel goal + hard stop if
stability margin < 0.15 or |pitch| > 45 deg) caught this before a full
violent tip this time, but the robot still fell ~3.3 m to ground level.

**This is the deeper finding:** the problem was never really about
open-loop vs. closed-loop control. Nav2's FollowPath/MPPI controller
*is* closing its loop - continuously, every tick - but it is closing that
loop against wheel odometry, and wheel odometry itself is unreliable
under the differential wheel speeds ANY turn requires (not just a fast
in-place spin). A controller can execute its control law perfectly and
still drive the robot off a physical edge if the localization feeding it
is wrong. Fixing this needs a real sensor-fusion localization layer (e.g.
`robot_localization`'s EKF, fusing wheel odometry with the IMU, with
Gazebo ground truth as the way to validate the fused estimate against
reality) - not a further correction bolted onto either raw source, and
not a different path shape. That is a control-stack change, out of scope
for this session; flagged here as the concrete next step.

## Numbers requested for the descent leg specifically (arc-turn run)

From `arc_turn_run.log` / `arc_turn_run.csv`:
- descent HALT events: 1
- descent min stability_margin: 0.305 (the safety abort fired at
  margin=0.579/pitch=-46.1 deg; margin continued falling to 0.305 as the
  goal-cancel + hard-stop took effect before the robot came to rest)
- descent max wheel_slip: 1.118
- descent max |pitch|: 57.30 deg
- descent max ground-truth lateral offset (|gz_y|): 6.266 m (i.e. off the
  6 m platform half-width - confirms the physical edge departure)
- outcome: SAFETY ABORT triggered (margin < 0.15 or |pitch| > 45 deg),
  goal cancelled, robot came to rest off the platform at ground level
  (gz z dropped from 3.425 to 0.125-1.68 m across the abort window)

Ascent-leg numbers (same run, for contrast - this is the part that
remains fully verified):
- HALT events: 0
- min stability_margin: 0.913
- pitch: settles at -16.0 deg on the ramp, 0.00 deg on the elevated level
- ground-truth lateral drift: 0.0 m for the entire ~47 m ascent

## Fix: EKF fusion (robot_localization), and the corrected round trip

Root cause confirmed: `FollowPath`/MPPI was closing its control loop
against wheel odometry, and wheel odometry's YAW diverges from ground
truth under any turning motion (reproduced above on a gentle, fully
closed-loop arc - not just the open-loop spin). Fixed with
`robot_localization`'s `ekf_node`, fusing wheel odometry's position +
velocity with the IMU's orientation + angular velocity into
`/odometry/filtered`, now feeding `controller_server` and owning the
`odom -> base_footprint` TF. Full isolated validation (confirming the fix
is actually accurate, not just running) is in
`validation/phase8/ekf_validation/README.md` - short version: yaw error
peaks at 3.25 deg during the exact spin that used to produce 35-40+ deg
of error, and position error does not grow with distance traveled
afterward.

Rerunning the SAME arc-turn FollowPath approach (`arc_turn_followpath_
test_ekf.py`, identical to `arc_turn_followpath_test.py` except it reads
`/odometry/filtered` instead of raw `/odom`) with the EKF now feeding the
controller, twice for reproducibility:

| metric | before (raw odom) | after (EKF), run 1 | after (EKF), run 2 |
|---|---|---|---|
| outcome | SAFETY ABORT, fell off platform edge | completed, no abort | completed, no abort |
| descent HALT events | 1 | 1 | 2 |
| descent min stability_margin | 0.305 | 0.872 | 0.872 |
| descent max wheel_slip | 1.118 | 9.088 (see finding below) | 1.353 |
| descent max \|pitch\| | 57.30 deg (tipping) | 16.00 deg (correct ramp angle) | 16.00 deg (correct ramp angle) |
| descent max ground-truth \|lateral Y\| | 6.266 m (off the 6 m platform half-width) | 4.339 m (within the arc's own 4 m design extent, not drift) | 4.319 m (same) |

Both EKF runs completed the full physical round trip: gate -> laydown ->
up the ramp -> onto the elevated level -> arc-turn -> back down the ramp
-> laydown -> gate, arriving safely with pitch and margin behaving
exactly as designed. This is a genuine fix for the originally diagnosed
failure (turning-induced localization error walking the robot off the
platform edge) - not fixed by accident, per the isolated validation above.

### A second, distinct finding surfaced by this rerun (not the same bug, not yet fixed)

Both EKF runs show a reproducible, LOCATION-SPECIFIC anomaly: right at
the ramp-top fillet transition (ground-truth x approx 40.79-40.85,
matching `RAMP_TOP_X=40.9702`), the robot's ACTUAL (ground-truth)
position and IMU pitch both freeze completely - genuinely stationary,
not a monitoring artifact (confirmed via the test script's own sample
timestamps, which stay perfectly regular through the freeze) - for
~58-62 seconds, while `/odometry/filtered`'s position keeps advancing
because it trusts wheel VELOCITY unconditionally (by design - see
`ekf_validation/README.md`) and the wheels are still being commanded and
apparently turning. This produces a temporary but large (~14.5 m at
its peak) position-estimate divergence, then the robot abruptly breaks
free and completes the rest of the ramp descent correctly (clean -16 deg
-> 0 deg pitch profile, matching the ascent's own proven behaviour).
Reproduced in both runs, same location, similar duration (~58s and
~62s) - not a one-off.

**This is a different root cause from the turning/yaw problem the EKF was
built to fix, and the EKF is not expected to fix it**: no dead-reckoning
approach (raw wheel odom OR EKF-fused) can distinguish "wheels commanded
forward, actually moving" from "wheels commanded forward, physically
stuck" without either a slip-detection measurement model or an external
position reference (e.g. a costmap/lidar-based localization correction),
neither of which this EKF configuration includes. Unlike the original
failure, this does NOT threaten safety: stability margin stayed at 0.872
(descent minimum) and pitch stayed at a stable, modest -5.12 deg the
entire stuck period in both runs - no tip risk, no HALT thrashing, and
the robot self-recovered and finished the traversal safely both times.
Flagged here as a genuine, reproducible robustness/efficiency finding
(a ~1-minute stall at a specific geometric transition) worth its own
dedicated diagnosis session, per the task's own "stop and report clearly
rather than keep patching" instruction for anything past the originally
scoped fix.

## Fillet-top stall diagnosis (using the two existing EKF runs' logs, no rerun)

`/wheel_slip` during the ~58-62 s frozen-ground-truth window, in BOTH
saved runs (`arc_turn_run_ekf1.csv`, `arc_turn_run_ekf2.csv`):

| run | stall window (t) | samples | slip min/max/mean | nonzero samples |
|---|---|---|---|---|
| 1 | 161.18 - 221.59 (60.4 s) | 122 | 0.000 / 0.233 / 0.038 | 35/122 |
| 2 | 160.01 - 224.04 (64.0 s) | 129 | 0.000 / 0.210 / 0.002 | 9/129 |

**Slip was NOT elevated in either run.** Both peaks (0.233, 0.210) sit
right at the *lower* edge of `descent_brake_node.py`'s own existing
"slip band [0.08, 0.25]" (its trigger range for ordinary controlled-
descent slip, not a stall signature) - not remotely close to the kind of
value a genuinely spinning, ungripped wheel would produce, and nothing
like the 9.088 one-off spike logged in run 1's overall summary (which,
checked against the row-level data, did not occur during this stall
window at all - it happened elsewhere, briefly, and was never sustained).

### Why: `/wheel_slip` measures wheel-joint tracking, not chassis displacement

`stability_monitor_node.py`'s `_compute_wheel_slip()`:

```python
slip = abs(cmd - velocity) / abs(cmd)
```

`cmd` is the commanded per-side wheel angular velocity, derived
kinematically from `/model/ramp_robot/cmd_vel` (the actuation topic).
`velocity` is that same wheel JOINT's actual simulated angular velocity,
from `/joint_states` - itself a real, ground-truth-accurate physics
reading, but of joint rotation only. This metric can only ever detect a
mismatch between "what the wheel was told to do" and "what the wheel
joint actually did" - it has no connection whatsoever to whether that
joint rotation produced any real chassis displacement. A wheel spinning
in place with nothing underneath it (classic high-slip case) would show
up here; a wheel rotating in good agreement with its command while the
CHASSIS is physically wedged against something and simply isn't moving
would not - and did not.

The corroborating evidence for exactly that second scenario: during both
stall windows, `/odometry/filtered` (fed by wheel odometry's own
VELOCITY - see `ekf.yaml`'s design) kept advancing smoothly and at a
normal rate the entire time, which is only possible if the DiffDrive
plugin's own wheel-derived velocity calculation - built from the same
`/joint_states` readings `/wheel_slip` also reads - was reporting
consistent, roughly-commanded forward wheel rotation throughout. Put
together: the wheels were very likely turning close to their commanded
rate (hence low slip, hence continued fused "progress") while the
chassis's real, ground-truth position and the IMU's own pitch reading
both stayed completely frozen. This is a genuinely stuck/jammed chassis
with normally-rotating wheels, not wheels spinning uselessly - and it is
architecturally invisible to `/wheel_slip` and to `/stability_margin`
(pitch stayed a stable, non-tipping -5.12° throughout, so the margin
model - which only cares about tip-over risk from pitch/CG, not
progress-over-time - had nothing to flag either).

**Conclusion: this is a genuine blind spot in the current architecture,
not a wiring bug.** No existing signal in this project measures "is the
chassis actually displacing in response to commanded motion" - every
existing safety signal (`/wheel_slip`, `/stability_margin`) operates
entirely on proprioceptive, joint/IMU-local information that cannot by
construction distinguish "rotating and progressing" from "rotating and
stuck."

### Proposed fix (design only - not implemented, needs its own scoped session)

A ground-truth-independent stall detector needs a source of TRUE
displacement information that is not itself derived from wheel rotation
(anything built purely from wheel odometry or `/joint_states` is
circular with respect to this exact failure mode - it's the same signal
being reused). Two candidate signals, in order of robustness:

1. **LiDAR scan-consistency check (recommended primary signal).** This
   robot already carries two LiDARs (`/scan`, the ramp-profiling sensor,
   and `/scan_360`, the level-mounted obstacle sensor). Comparing a
   current scan against one from N seconds ago and checking whether the
   surrounding range profile has shifted by an amount consistent with
   the commanded/expected displacement is a genuinely independent check
   - it directly observes motion relative to the environment, the closest
   real-hardware equivalent to what ground-truth polling let this session
   diagnose in simulation. This is the only proposed signal here that
   isn't proprioceptive.
2. **IMU-based velocity cross-check (secondary/early-warning only).**
   Integrate the IMU's linear acceleration (gravity component already
   removable using the pitch this project already computes) over a
   SHORT window (a few seconds - accelerometer double-integration drifts
   far too fast to trust over 60 s) to get an estimate of velocity
   independent of wheel rotation, and compare it against the commanded/
   fused velocity. Faster and cheaper than scan matching, but lower
   confidence - best used to raise an early flag, with the LiDAR check
   as confirmation before actually triggering a response.

Sketch of where this would plug in: a new node (e.g.
`motion_confirmation_node.py`) subscribing to the chosen scan topic(s),
`/cmd_vel`, and `/imu`, publishing something like `/motion_stalled`
(bool) once both signals agree commanded motion has produced no real
displacement for longer than a deliberately conservative threshold (to
avoid false-triggering on a normal brief replan pause) - `state_machine_
node.py` or `cmd_vel_watchdog_node.py` could then treat that similarly
to how `/ramp_state == HALT` is already handled, matching this project's
existing "independent enforcement gate" pattern. Left unimplemented here
deliberately: picking real thresholds, validating the scan-consistency
check doesn't false-positive at legitimately low commanded speeds near a
goal, and designing the actual recovery behavior once a stall is
detected are all non-trivial decisions that deserve dedicated attention,
not something to squeeze in alongside this diagnosis.

## Loaded-payload round trip: the same blind spot, far more severe

`loaded/site_loaded_ekf.csv` / `.log`: the same round trip rerun with a
100 kg / +0.1 m forward-CG payload, to check whether the fillet-top
stall above reappears under load now that its mechanism is understood.
It does, far more severely - the robot's ground-truth position and
pitch freeze at the ramp-top transition (X~41.3 m, elevated-platform
height) about a third of the way into the test and **never move again
for the rest of the ~270 s run**, while `/odometry/filtered` and the
test's own success criteria (both built on the same fused odometry)
report a complete, clean round trip. Unlike the unloaded case (self-
recovers after ~58-62 s), the loaded case never recovers within the
test's full duration - this is the exact same architectural blind spot
(no signal can tell "wheels rotating, chassis progressing" from "wheels
rotating, chassis stuck"), just severe enough this time that the robot
never breaks free at all. Margin and slip both stay low/healthy the
entire stuck period (slip near-zero - not a wheel-traction struggle),
consistent with the controller genuinely believing, via its own fused
position, that it had already reached the goal and stopping trying, not
with an active-but-failing correction attempt. **This means loaded
traversal of this ramp does not currently work reliably, and the test
harness cannot detect that on its own - session B should not build on
the assumption that it does.**

## Breakover-mechanism check: NOT the same as the sharp-world finding

Before building a detector, checked whether this is the same physical
mechanism as the sharp-world breakover finding (README's "Root-causing
the loaded-payload climb failure") - a genuine rigid-body ground-
clearance interference at a sharp, unfilleted grade change - just
occurring at a fillet under load instead. It is not, on direct evidence:

- **Position**: the frozen position (ground-truth X~41.3-41.4 m) sits
  0.3-0.6 m short of `ramp_fillet_top_3`'s own edge (X=40.97, the start
  of the fillet - see `construction_site.sdf`'s own geometry) - the
  robot never reaches the fillet panels at all, let alone their seams.
  There is no box-to-box seam anywhere near the frozen position; it is
  in the middle of `elevated_level`'s own single, continuous flat slab.
- **Orientation**: pitch stays at 0.0 deg (confirmed via fine-grained
  logging right through the freeze transition) - completely flat and
  level throughout. The sharp-world finding's own signature was a
  clean, sustained -16.0 deg pitch (the chassis physically wedged
  against and resting on an angled surface). Zero pitch means the
  chassis never contacted any sloped surface here.
- **Signal behavior at the freeze**: `/stability_margin` and
  `/wheel_slip` both show ZERO transient disturbance at the exact
  moment real motion stops (margin flat at 0.742-0.745, slip exactly
  0.0 for many samples before and after) - no jolt, no traction
  struggle. A genuine mechanical impact (chassis hitting an
  obstruction) would be expected to show at least a brief perturbation
  in one of these; there is none.

**Conclusion: this is not a physical/ground-clearance constraint - it
is a control-loop/localization-belief failure.** The evidence points to
`FollowPath`'s own goal-completion check operating on the fused (EKF)
position rather than ground truth: real chassis motion stops (for a
reason not fully instrumented at the lowest level - possibly a
transient MPPI planning artifact right at the flat-to-slope transition
boundary in the combined arc+taper+ramp path, given the frozen position
consistently lands very close to where that path's own phases hand off
- see `RAMP_APPROACH_X` in `arc_turn_followpath_test_ekf.py`), while
`/odometry/filtered` keeps integrating nonzero wheel-reported velocity
and its position estimate keeps "progressing" toward the goal, entirely
decoupled from the chassis's real, frozen position, until the fused
estimate crosses the goal checker's tolerance and the whole leg reports
false success. This is a genuinely different failure class from the
sharp-world finding (a real, physical, unfixable-by-software hardware
constraint) - this one is a software/estimation gap, and per task scope
this session addressed it with a ground-truth-independent detector
(below) rather than chasing the exact low-level trigger further.

**Update, a later session: the "not fully instrumented at the lowest
level" gap above was closed, and two of the bullets above turned out to
be wrong in a specific, important way - see "Root cause found and fixed"
near the end of this document.** There *is* a seam essentially at the
frozen position once the robot's 0.28 m wheelbase offset is accounted
for (the reported freeze position is the chassis center; the front
wheels are already at the seam) - it was a real 0.05 m vertical step in
`construction_site.sdf`'s own collision geometry, not "no seam nearby at
all." And there *is* a mechanical interaction at the freeze - real
wheel-spin-without-chassis-motion, confirmed via `/joint_states` - it
just doesn't show up in `/stability_margin`/`/wheel_slip` because those
signals compare commanded to actual **wheel** motion, not chassis
motion, exactly as flagged as a possible gap earlier in this same
document. The "control-loop/localization-belief failure" framing above
was directionally right (a real, physical obstruction was masked by
`FollowPath` trusting fused position instead of noticing it couldn't
actually make progress) but the root cause is a fixable geometry defect,
not a permanent estimation gap - it has since been fixed and reverified.

## Live stall detector: implemented, not just proposed

`motion_stall_detector_node.py` (`ramp_autonomy_control`) is now a real,
running safety node, wired into `state_machine_node.py`'s HALT logic as
a third independent trigger alongside margin and slip. Design: LiDAR
scan-consistency as the primary, confirming signal (hybrid of `/scan_360`
and `/scan`, preferring whichever has enough valid comparable beams -
see the node's own docstring for why neither sensor alone was enough),
with a short-window IMU forward-accel check as a non-authoritative early
warning. Three real bugs were found and fixed by testing against the
actual failing scenario, not assumed correct from design alone:

1. **Continuity gate too fragile** - a single noisy near-zero `cmd_vel`
   tick (MPPI/velocity_smoother can emit one even while otherwise
   driving continuously) reset the "commanded for the full window"
   timer, so the detector never fired even once across ~90s of
   continuous real driving with a genuine, confirmed stall in the
   middle of it. Fixed with a 0.5s debounce before a sub-threshold tick
   counts as "stopped commanding."
2. **HALT-thrash feedback loop** - the first fix above worked (detector
   fired, forced HALT), but `cmd_vel_watchdog_node.py` correctly zeroing
   the actuation topic during HALT then immediately un-satisfied the
   detector's own "continuously commanded" gate, clearing the signal,
   letting the FSM recover, resuming commanding, failing to move again,
   and repeating - 40 HALTs in one descent leg. Fixed by latching the
   confirmed-stall signal so losing the commanded-motion gate alone
   does not clear it.
3. **Permanent deadlock** - the latch fix above, tested against the
   actual loaded scenario, revealed a worse problem: clearing the latch
   only on evidence of real scan movement is circular once HALT is
   active (HALT correctly zeros the actuation topic, so the scan
   correctly shows "no movement" for as long as HALT holds - there is
   no way to ever observe the "real movement" evidence being waited
   for). One HALT never recovered for over 700s straight, stuck on
   ordinary flat ground with nothing physically wrong. Fixed with a 10s
   cooldown: the latch also auto-clears after that period regardless of
   scan evidence, giving the FSM a periodic chance to retry - a bounded
   ~12-15s retry cycle if the underlying problem persists, not a silent
   permanent deadlock.

Also found and fixed along the way: `/scan_360` (the level-mounted,
purely horizontal obstacle ring) alone returns 0/360 valid beams on the
open elevated platform (too open for its 12m range in every direction -
structurally blind exactly where this session's own stall happens);
`/scan` (the ramp-profiling LiDAR) alone fixes that but causes chronic
false stalls during ordinary flat, straight-line driving (a beam
pointed at a large uniformly flat floor returns nearly the same range
whether the robot has translated 0m or 10m along it - translation-
invariant, not evidence of "not moving"). Fixed with the hybrid: prefer
`/scan_360` when it has enough valid data (most of this site has nearby
structure within 12m), fall back to `/scan` only when it doesn't (the
open-platform case).

### Live round-trip result with the fixed detector

Reran the loaded round trip end to end (staged into two shorter runs to
work around an unrelated environment issue - see below). Stage A (gate
-> laydown -> elevated level, ordinary flat driving + ascent): completed
successfully, 17 HALT events, all recovered via the cooldown, reaching
the elevated level cleanly (pitch 0.00 deg, matching the proven ascent
behavior). Stage B (arc-turn -> ramp crossing -> return to gate):

| metric | before detector | after detector (live) |
|---|---|---|
| leg 3 (arc+ramp) outcome | reports `state=EXIT`, fused odom claims target reached - **false success** | ends in `state=HALT`, fused odom (30.56) far short of the real target (26.30) - **honestly incomplete** |
| descent HALT events | 0 | 94 |
| ground truth at end of leg 3 | frozen at (41.35, ..., 3.425) - never left the platform | frozen at (41.35, ..., 3.425) - still never leaves the platform |
| HALT re-entry spacing | N/A | mean 11.85s (34 sampled entries), consistent with the ~12-15s cooldown-driven retry cycle design, not rapid thrash |

**The core requirement is met: a confirmed stall now forces real,
repeated HALT instead of silently reporting fictional success - the
`/ramp_state` topic and HALT count tell the true story even though the
robot still cannot actually get past this point loaded.** One honest
caveat: this diagnostic test script's own final "ok"/`odom_x` print
statements (built before the detector existed, checking fused-odometry
targets) can still look superficially plausible after many HALT-recover
cycles let the fused estimate creep forward in small increments between
retries (visible in leg 4's own summary line) - a test-harness
reporting nuance, not a safety-system failure; the authoritative signal
(`/ramp_state` reaching and repeatedly re-entering `HALT`, confirmed
against ground truth) is unambiguous. **The underlying reason the
robot cannot complete this leg loaded is still not fixed - the detector
makes the failure visible and safe (real HALT, no silent false
success), it does not resolve it. Session B still should not assume
loaded traversal works.**

An unrelated environment issue surfaced during this verification and is
worth recording for future sessions: wrapping a long-running rclpy test
script in a shell `timeout N` shorter than the script's actual needed
runtime causes a `RCLError: context is not valid` crash a few seconds
before N (SIGTERM arriving mid-`spin_once`/mid-publish), which looks
identical to an environment/resource fault but is actually just too
short a timeout - lost significant time this session before the timing
correlation (crash time scaling exactly with the chosen `timeout N`
value) revealed it. Fixed by using generous or no shell-level timeouts
for full round-trip tests going forward.

## Goal-verification gate: closing the "ok" caveat above with a real fix

The caveat right above this section (the diagnostic script's own `ok`/
`odom_x` print statements can look superficially plausible after many
HALT-recover cycles) is exactly the same mechanism that made the
original, undetected stall report `state=EXIT`/"success" in the first
place - a test/client accepting `FollowPath`'s own goal-completion
report, which trusts fused EKF position, at face value. The stall
detector above made the physical failure visible and safe (real HALT);
it did nothing to stop a client from also accepting a stale or
eventually-drifted-into-tolerance "succeeded" result. Closing that
gap properly (not just re-flagging it) was the next session's task.

**The fix - a genuine goal-verification gate, not just another HALT
trigger.** `motion_stall_detector_node.py` now also subscribes to
`/joint_states` and integrates actual (not commanded) wheel angular
velocity into two new published signals, using the same kinematics
convention as `stability_monitor_node.py`'s own slip calculation
(`wheel_radius_m=0.12`, `track_width_m=0.7`, same `WHEEL_JOINT_SIDE`
mapping):

- `/raw_wheel_distance_m` - a running total of `|implied linear speed|
  x dt`, from real wheel rotation, never commanded velocity and never
  anything from the EKF. Already more honest than fused position, but
  still proprioceptive on its own - a spinning, non-translating wheel
  (the exact loaded-stall failure mode) still accumulates it.
- `/verified_progress_m` - the same integration, but only accumulated
  during windows where `/motion_stalled` is not latched. This combines
  the wheel-rotation and LiDAR scan-consistency signals rather than
  picking one, matching the same hybrid-signal spirit already used for
  the stall detector itself - independent of the fused position that
  fooled `FollowPath`'s own check, and independent of wheel rotation
  alone (a spinning-but-stuck period stops counting once the stall
  latches).

The construction-site round-trip test script
(`arc_turn_followpath_test_ekf.py`, and its staged `arc_stage_a.py`/
`arc_stage_b.py` derivatives) now samples `/verified_progress_m`
immediately before sending the arc-turn+ramp `FollowPath` goal and
again once it reports done, and requires the delta to cover at least
85% of the path's own geometric length (`VERIFIED_PROGRESS_MIN_FRACTION`)
**in addition to** `FollowPath` itself reporting
`GoalStatus.STATUS_SUCCEEDED` before accepting the leg as complete -
either condition failing (including simply timing out without any
result at all, which the original script accepted as an implicit
success) now produces `ok=False`. A second, unrelated bug was found and
fixed while first running this: a fixed 1.5s startup spin before
building the first goal path is not a reliable wait for a fresh node's
`/odometry/filtered` subscription to actually receive its first message
(DDS discovery time varies) - one early rerun built a goal path from a
stale `odom_x=0.0` default instead of the robot's real ~47m position,
producing a meaningless immediate `FollowPath` rejection. Fixed by
blocking on an actual "message received" flag (`wait_for_odom()`)
instead of guessing a duration.

### Rerun with the gate live

Same staged approach as before (Stage A: legs 1-2, ordinary flat
driving + ascent, unaffected by this fix - completed normally, robot
parked on the elevated level). Stage B (arc-turn -> ramp crossing):

| metric | before the gate (detector only) | after the gate (this session) |
|---|---|---|
| leg 3 outcome | ends `state=HALT`, fused odom (30.56) short of target - reported honestly by `/ramp_state`, but the script's own `ok` print still looked plausible | `ok=False` - the script itself now reports the leg as failed, not just `/ramp_state` |
| why `ok=False` | N/A (not checked) | `FollowPath` never reported `STATUS_SUCCEEDED` within the test's own 180s window this run (`timed_out=True`); verified progress reached 88% (24.92 of 28.30 m intended path length) - close to the threshold, consistent with the robot genuinely covering almost the whole curved arc+taper+ramp-approach before wedging in the final stretch, not a spurious reading (cross-checked against `/raw_wheel_distance_m`: only ~1.6 m of cumulative gap between raw and verified wheel distance across the *entire* session, legs 1-3 combined) |
| descent HALT events | 94 | 15 (same mechanism, fewer cycles fit in this run's real-time window) |
| ground truth at end of leg 3 | frozen at (41.35, -0.22, 3.425) | frozen at (41.3477, -0.2303, 3.425) - same position to three decimal places, same failure, reproduced deterministically |
| leg 4 | ran anyway (leg 3's fictional "success" let it proceed) | **skipped** - the script now refuses to attempt the return leg when leg 3 didn't pass the gate, logged explicitly as `leg 4 SKIPPED: leg 3 did not pass the goal-verification gate` |

**Answering the actual question this session was asked**: does the
robot now recover and complete the descent, or does it correctly HALT
and stay safely stopped without ever completing? **The latter, observed
directly, not assumed.** No new recovery strategy was added (the
existing HALT -> pre-halt-state cycling is unchanged) - it correctly
cycled 15 times during this run and never once got the robot past the
freeze point. The robot never accelerates into anything unsafe (each
HALT correctly holds it stationary), but it also never gets unstuck
within the test's own runtime using only the FSM's existing recovery
behavior. This is an honest, reproducible FAIL, not a fictional PASS -
**exactly the outcome the goal-verification gate exists to surface.**
The underlying root cause (the EKF trusting integrated wheel velocity
during a physical stall) is still not fixed at the source; what changed
this session is that nothing downstream can be fooled by it anymore.

## Root cause found and fixed: a real 0.05 m geometry defect, not a control/localization bug

Every prior pass diagnosed *consequences* (pitch, margin, slip, fused
position) - none had looked at commanded-vs-actual wheel velocity or the
raw world geometry at the exact freeze coordinate. This pass did both.

### Mechanism (c), confirmed with direct per-wheel telemetry

A new standalone observer, `loaded/freeze_diagnosis/wheel_diagnosis.py`,
subscribes to `/model/ramp_robot/cmd_vel` (deriving per-wheel commanded
speed with the same kinematics `stability_monitor_node.py` already
uses), `/joint_states` (real per-wheel actual velocity and effort - see
below), `/wheel_slip`, and polls Gazebo's ground-truth pose+orientation
throughout. Run in parallel with a fresh staged repro (immune to the
timeout-wrapper crash, same pattern as the earlier `stall_monitor.py`).

**`/joint_states`' `effort` field is present in the message but always
reads `0.0`** - `gz-sim-joint-state-publisher-system` does not actually
model torque, confirmed by direct inspection this session. Effort was
not usable as a signal; velocity was.

**A fresh repro this time did not reproduce a *permanent* freeze** - a
genuinely new finding on its own: MPPI is a stochastic sampling-based
controller, and this project's turn-then-descend maneuver is not
perfectly deterministic run to run (see "A related, still-open finding"
below). This run's robot got stuck only briefly (~1 s) at essentially
the same location before lurching through, which turned out to be
exactly the right outcome to instrument - it gave a clean before/during
/after telemetry window instead of just a static endpoint.

**The stuck window itself (t=327.34-327.94s in the observer's own
clock), read directly from `/joint_states` at ~250-1000 Hz resolution:**

| quantity | value during the stuck window |
|---|---|
| commanded linear/angular (`/model/ramp_robot/cmd_vel`) | ~0.47-0.50 m/s / -0.19 to -0.31 rad/s (a real, substantial forward command, not near-zero) |
| front-left wheel: commanded vs. actual | 4.47 rad/s commanded, 4.50-4.54 rad/s actual - matching almost exactly |
| front-right wheel: commanded vs. actual | 3.35 rad/s commanded, 3.28-3.33 rad/s actual - matching almost exactly |
| `/wheel_slip`'s own live reading | 0.0-0.0075 - effectively zero |
| ground-truth chassis pose (all 6 DOF) | frozen bit-for-bit: x=41.3321, y=-0.2444, z=3.4338, roll=0.003°, pitch=-1.815°, yaw=179.996 - unchanged across every sample in the window |

This is **mechanism (c) from the task list, unambiguous**: wheel joint
velocity is nonzero and matches commanded velocity almost exactly
(confirming free rotation, not an actuation-layer fault - ruling out
(b)), while the chassis's full ground-truth pose does not move at all -
and `/wheel_slip` reads essentially zero throughout, **directly
confirming the concern flagged two sessions ago**: that metric compares
commanded wheel rotation to *actual wheel rotation*, not to chassis
displacement, so a wheel spinning freely against an obstruction it
cannot climb is invisible to it by construction. No redesign of
`/wheel_slip` was needed this session once the actual obstruction was
found and fixed (see below) - the metric was doing exactly what it was
built to do, just not the thing needed to catch this specific failure.
Commanded velocity was never near-zero at any point during the
approach, ruling out (a) as well.

At t=328.34s the chassis suddenly moved 0.31 m in one 0.2 s sample and
pitch swung through -5.1°, +0.2°, +4.3°, +7.5°, +10.5° before settling
into the ramp's genuine -16° descent angle - a lurch/pop signature, not
a smooth roll, consistent with a wheel overcoming static contact against
a physical edge rather than gradually climbing a slope.

### The geometry: an exact, isolated 0.05 m step at the elevated-level seam

Cross-referencing the frozen pose directly against `construction_site.sdf`'s
own collision geometry (not just "is it near a fillet," computed
precisely): the robot's front axle (base_footprint at x=41.3321, yaw
~180°, wheel_x_off=0.28 m -> front axle at world x=41.0521) sat almost
exactly at the theoretical first-contact distance (0.0975 m, from
`sqrt(wheel_radius^2 - (wheel_radius-step_height)^2)` with
wheel_radius=0.12 m) for a 0.12 m-radius wheel meeting a step of height:

Computing every seam's exact top-surface continuity along the whole
`ramp_fillet_bottom_*` -> `ramp_mid` -> `ramp_fillet_top_*` chain (corner
positions from each box's pose/size/pitch, not eyeballed) found **six
seams matching to within 1.1 mm** (the intended, tangent-continuous
fillet curve, exactly as designed) and **two seams off by exactly
0.0498-0.0500 m** - a full two orders of magnitude larger, and
suspiciously exactly half of the 0.1 m slab thickness used throughout
this world:

| seam | dz (vertical gap) |
|---|---|
| `ramp_fillet_bottom_1` -> `ramp_fillet_bottom_2` | -0.0007 m |
| `ramp_fillet_bottom_2` -> `ramp_fillet_bottom_3` | -0.0011 m |
| `ramp_fillet_bottom_3` -> `ramp_mid` | -0.0002 m |
| `ramp_mid` -> `ramp_fillet_top_1` | +0.0008 m |
| `ramp_fillet_top_1` -> `ramp_fillet_top_2` | +0.0006 m |
| `ramp_fillet_top_2` -> `ramp_fillet_top_3` | +0.0002 m |
| **`ramp_fillet_top_3` -> `elevated_level`** | **-0.0500 m** |
| **`laydown_area` -> `ramp_fillet_bottom_1`** | **+0.0498 m** |

**Root cause: an authoring bug, not a design choice.** `elevated_level`,
`laydown_area`, and `access_road` are all flat SDF `<box>` slabs whose
`<pose>` z was set to what should have been their *top surface* height,
but SDF box poses are the geometric **center** - with these 0.1 m-thick
slabs, every one of them sat 0.05 m too low, breaking continuity with
the ramp fillet chain at both ends. The ramp segments themselves (the
curved, hand-fit fillet geometry) are correct and precise - the defect
is entirely in the three flat ground/platform slabs.

**Why only the top seam caused a live failure, not the bottom one (same
size):** the ascent leg (leg 2) crosses the bottom seam via a raw,
constant-velocity, straight-line `cmd_vel` drive with no active steering
- a step approached calmly, head-on, with sustained torque climbs
reliably. The top seam is reached mid-maneuver, still inside the
closed-loop `FollowPath` arc-turn+taper (the telemetry above shows a
real, nonzero `cmd_angular` of -0.19 to -0.31 rad/s at the moment of
contact - MPPI still actively correcting heading), with a 100 kg
forward-offset payload pressing extra weight onto the front wheels
against the step's face rather than onto its top surface. The same
physical step, encountered calmly vs. mid-correction, produced
different odds of getting past it on the first attempt.

### The fix

Both defects corrected in `construction_site.sdf` (`elevated_level`
z: 3.2500 -> 3.3000; `laydown_area` and `access_road` z: 0.0500 -> 0.1000,
raised together since they already matched each other and must stay
matched). Recomputing every seam's continuity after the fix: **every
seam in the chain now matches to within 1.1 mm**, the bottom two
(previously 0.0 mm apart before the fix, since `access_road`/
`laydown_area` were already mutually consistent) unchanged relative to
each other. `office_pad`/`waste_pad` were left untouched - not on this
round trip's path, not exercised by any existing test, out of scope
this session.

### Reverification: the loaded round trip now genuinely completes, end to end

Full staged rerun against the corrected world (`freeze_diagnosis/
site_fixed_stage_a.{csv,log}`, `site_fixed_stage_b.{csv,log}`):

| metric | before the geometry fix | after the geometry fix |
|---|---|---|
| leg 3 outcome | `ok=False`, `timed_out=True` - never completes | **`ok=True`** - `FollowPath` reports `STATUS_SUCCEEDED`, verified progress 270% of path length (well past the 85% gate) |
| leg 4 (return to gate) | skipped (leg 3 never passed the gate) | **ran and completed** - `odom_x=0.99`, back at the gate |
| ground truth at end of leg 3 | frozen at (41.33-41.35, ..., 3.42-3.43) every time | (26.15, 0.03, ...) - genuinely at the bottom of the ramp, target was 26.30 |
| descent HALT events (leg 3) | 8-94 depending on run, none ever cleared the freeze | 18, all cleared - the robot always eventually continued |
| margin/slip/lateral-Y **while actually on the ramp panel** (gz_x in [26,41.5]) | N/A (never got that far in the failing runs) | min margin 0.463 (well above the 0.15 HALT floor), max slip 0.024, max \|lateral Y\| 0.588 m (panel half-width is 2.25 m) - a clean, safe descent |

**The specific mechanism investigated this session is fixed and
reverified with a live, end-to-end rerun - the loaded robot no longer
freezes at the elevated-level seam, and the round trip completes.**

### A related, still-open finding - not this session's mechanism, flagged honestly

The reverified run's *overall* summary numbers (18 HALTs for the leg,
overall min margin 0.199, overall max slip 4.692, overall max \|lateral
Y\| 5.919 m) look rougher than the clean on-panel numbers above. Checked
directly rather than left as a discrepancy: `stability_margin` never
crossed `STABILITY_HALT_THRESHOLD` (0.15) at any point (0.199 is the
true minimum, comfortably above it) - no margin/tip-over-triggered HALT
occurred at all this run, all 18 were slip/stall-triggered. The worst
slip (4.692) and lateral-Y (5.919 m) readings both occurred **off the
ramp panel entirely** - the high slip during the arc-turn wander on the
open elevated level (gz_z=3.475, flat), the large lateral-Y during leg
4's return drive on open flat ground near the gate (gz_z=0.125) - normal
locations to be imprecise in, not safety-relevant there. This is
consistent with, and not a new instance of, the already-documented
`PathAlignCritic`/crosstrack-tuning and MPPI-stochasticity limitations
(see "Known Gaps" in the main README) - a separate, pre-existing
turn-quality issue, not the freeze mechanism this session targeted, and
not newly introduced by this session's geometry fix. Not investigated
further this session; flagged for whoever picks up turn-quality tuning
next.

## Follow-up audit: is the same authoring mistake anywhere else in `construction_site.sdf`?

Before populating the site with props, audited every remaining
flat-box-to-flat-box and flat-box-to-ramp join in the file the same
precise way (exact corner math from each model's pose/size/pitch, not
visual inspection) - not just the two joins already found and fixed.

**Method**: every model's pose+size extracted directly from the SDF.
For the five flat "floor" slabs (`access_road`, `laydown_area`,
`elevated_level`, `office_pad`, `waste_pad`), computed each one's exact
X/Y footprint and checked every pair for a shared boundary (overlapping
range on one axis, touching within 5 cm on the other - generous enough
to catch the ~4.6 mm gaps the ramp fillet chain uses by design). For
every wall (`office_wall_*`, `waste_wall_*`, all 5 `perimeter_*`
segments), computed its exact base height and compared to whatever
floor it stands on. For the ramp fillet chain, recomputed all 8
seams (already fixed, reverified here as part of the same pass).

**Result: no other instance of the bug found.**

- **Floor-to-floor adjacency**: only one pair of the five flat slabs
  actually shares a boundary - `access_road`/`laydown_area` at x=8.0 m
  (already fixed together last session, since they were always mutually
  matched to each other). `elevated_level`, `office_pad`, and
  `waste_pad` do not geometrically border any other flat slab at all -
  each is surrounded by open `ground_plane`, so there is no seam to
  mismatch. (`office_pad` and `waste_pad` sit at a different height from
  `access_road`/`laydown_area` - 0.1 m top vs. 0.15 m - but since they
  never touch, this is not a seam defect, just two independently-heighted
  pads with no shared boundary.)
- **Wall bases**: all 8 `office_wall_*`/`waste_wall_*` segments match
  their own pad's top surface exactly (dz=0.0000 m, all 8 checked) -
  correctly authored, no sign of the box-center-vs-top-surface confusion
  (if it were present, the walls would sit 0.05 m too low relative to
  their pad, the same signature the ramp-seam bug had - they don't).
  All 5 `perimeter_*` boundary walls sit exactly flush on `ground_plane`
  (dz=0.0000 m, all 5 checked).
- **Ramp fillet chain**: all 8 seams (`laydown_area` through
  `elevated_level`) reconfirmed matching to within 1.1 mm, as fixed.

**Why the bug didn't recur elsewhere**: the original mistake was
specific to `access_road`/`laydown_area`/`elevated_level`'s pose.z being
set to the intended top-surface height without the 0.05 m box-center
correction. Every wall in the file was evidently authored the *other*
way around (base height computed correctly relative to its own pad's
actual, already-correct top surface) - consistent with the walls having
been added referencing the pads, not independently miscalculating
height the same way. No code or world-file change was needed as a
result of this audit - it is a confirmation, not a fix.

## Site populated with real Fuel props, and reverified

`construction_site.sdf` was populated with 22 real Gazebo Fuel assets
(sourcing, real dimensions, and placement zoning logic in
`docs/construction_site_design.md`'s "Session B" section - not
repeated here). This section covers the placement-safety verification
method and the reverification result specifically.

### Two independent clearance checks before anything was written

1. **Exact-footprint overlap** against every existing model's real
   pose/size (0.3 m margin, each prop's own host pad excluded).
2. **Distance from all 1197 real ground-truth points** logged across
   the 4 previously-saved verification runs (both clean unloaded round
   trips, both stages of the loaded round trip) - 1.5 m safety buffer.
   Minimum clearance found: 2.0 m.

Both checks are reproducible - the exact Python used is in this
session's own record; every existing model's pose/size was read
directly from `construction_site.sdf`, not eyeballed.

### A real bug caught by live-testing, not assumed away

Fuel models don't all default to `<static>true</static>` the way every
hand-authored model in this file does - `Pallet_Rack_Section`,
`Dumpster_Large`, `Euro pallet`, and `Cinder Block` have no `<static>`
tag at all in their own `model.sdf` (SDF's own default is `false`). A
smoke-test load caught `laydown_pallet_rack` having fallen to
`z=-0.463` under gravity, well below its placed `z=0.150`. Fixed with
an explicit `<static>true</static>` override on every one of the 22
new `<include>` blocks; reconfirmed via `gz model -m <name> -p` that
every prop settles exactly at its placed pose.

### Reverification: `ok=True`, full round trip, on the populated world

Same protocol as every prior reverification in this file (stall
detector + goal-verification gate live, ground-truth polling, the same
metrics reported): leg 3 completes (`FollowPath` reports
`STATUS_SUCCEEDED`, `odom_x=26.29` against a 26.30 target), leg 4 runs
and completes back at the gate. While actually on the ramp panel: min
margin 0.463, max slip 0.076, max lateral drift 0.497 m against a
2.25 m half-width - matching every prior clean run. Full logs:
`prop_population/site_populated7_stage_{a,b}.{csv,log}`.

### Two transient instabilities hit and root-caused along the way - both confirmed unrelated to the props

**1. Real-time factor dropped to ~0.70x during one verification
attempt** (`gz topic -e -t .../stats`), instead of the ~1.0x this
project has always run at. Root-caused, not assumed: a clean, isolated
`gz sim -s -r` A/B test (populated world vs. the pre-props world,
otherwise identical, no ROS bringup) measured 0.991 vs. 0.999 real-time
factor - a ~1% difference, nowhere near the ~30% drop actually
observed live. The real cause: this session's own repeated launch/
verify/kill cycles had left a dozen-plus stale `parameter_bridge`/
`robot_state_publisher` processes running (`pkill` pattern-matching
had silently failed to catch them, the same category of tooling
gotcha as this project's earlier shell-`timeout`-wrapper issue),
pushing system load average to 10-20 on a 12-core machine. Fixed by
killing every stale PID explicitly (`ps aux | grep ... | awk
'{print $2}'`, not relying on `pkill`'s own pattern match) and
confirming real-time factor back to ~1.0 before the reverification
that actually succeeded.

**2. The arc-turn+descent maneuver's 180 s test-script timeout was
already razor-thin before any props existed** - the one saved
pre-props successful run (session D) used 179.0 of its own 180 s
budget. Two clean-system populated-world attempts both completed the
actual physical descent (109-111% verified progress, finishing within
<1 m of the target, a clean 16 deg pitch profile matching every prior
successful descent, no HALT thrash) but had `FollowPath`'s own success
report land just after the 180 s mark - `timed_out=True`, not a safety
abort, not a prop interaction. Raised to 240 s
(`follow_arc_and_ramp`'s own `timeout` parameter in
`arc_turn_followpath_test_ekf.py`) - correcting an already-too-tight
pre-existing budget, not adding margin to route around a props-caused
problem.

### A related, not-props-caused observation from the successful run

Leg 4 (open-loop return, no steering correction once the arc-turn
hands off) drifted to `gz_y=-11.26 m` in the successful run - more
extreme than the `-5.92 m` worst case the prop placement's own safety
margins were built against (see "A related, still-open finding" above
for that mechanism). Checked directly rather than assumed safe: the
drift grows steadily and monotonically as leg 4 progresses, so by the
X range where any placed prop sits, `gz_y` had already moved well past
that prop's own Y position, continuing in the same direction - none
were approached. This is the same pre-existing turn-quality gap this
document already tracks, now observed with a larger worst-case
magnitude than previously recorded (-11.26 m vs. -5.92 m) - flagged
explicitly, since a future, still-larger excursion is not something
this session's prop clearances are proven safe against indefinitely.
Not investigated further this session (out of the scope this session
was asked to cover); relevant for whoever next works on the
`PathAlignCritic`/MPPI-stochasticity turn-quality gap.

## Crosstrack diagnosis: why the partial fix didn't fully close the gap

The `-11.26 m` leg-4 excursion above was the trigger for this session:
a follow-up specifically asked to find WHY the already-partial
`offset_from_furthest` fix (20 -> 3, applied to `descent_mode` when the
crosstrack-error finding was first root-caused) still let drift grow
past its previously-recorded worst case, with direct evidence, before
touching any parameters again.

### Method: an external, code-consistent proxy for the critic's own activation gate

There is no live ROS topic for per-critic MPPI cost (`FollowPath`'s own
`visualize` output, checked directly, is trajectory-sample markers and
the optimal/transformed path only - no cost breakdown). Rather than
add debug output to `nav2_mppi_controller`'s C++ source and rebuild a
system package, `path_align_critic.cpp` (the local fork at
`~/priest_reimpl/nav2_mppi_fork_src/`) was read directly and its own
activation gate reproduced externally:

```cpp
utils::setPathFurthestPointIfNotSet(data);
const size_t path_segments_count = *data.furthest_reached_path_point;
if (path_segments_count < offset_from_furthest_) { return; }   // critic fully disabled, zero cost
```

`crosstrack_diagnosis_flatmode/crosstrack_diag.py` reconstructs the
exact `Path` message sent (reusing `build_turn_and_ramp_path` unchanged
from the real test script), tracks a running-max "nearest path point
index" as a proxy for `furthest_reached_path_point`, and compares that
against whichever profile's `offset_from_furthest` is live for the
current `/ramp_state` (read from `ramp_mppi_params.yaml` - `flat_mode`/
`ramp_mode`/`transition_mode`/`halt_mode` = 20, `descent_mode` = 3).
Cross-checked against the actual perpendicular cross-track deviation of
the real (ground-truth-verified) robot position from the sent path, so
the finding doesn't depend on the proxy being exact.

### Task 1: activation problem, not a weighting problem - confirmed both by source and live data

`nav2_param_switcher_node.py`'s own `RAMP_STATE_PROFILE` maps `EXIT`
(and `APPROACH`) to `flat_mode` - and `state_machine_node.py`'s
`EXIT -> DESCENT` edge requires `on_ramp` (`abs(pitch_deg) >
PITCH_ON_RAMP_DEG`), which stays false for the entire arc-turn (it
happens on a flat elevated platform - pitch reads 0.0 deg throughout
every logged run). **The whole arc-turn runs under `flat_mode`, and
`flat_mode`'s `path_align_offset` was never touched by the earlier
fix - only `descent_mode`'s was.**

Live confirmation, loaded config, fresh repro (`crosstrack_diagnosis_
flatmode/ct1_diag.csv`): the arc phase of the combined path is 16
points (radius 2.0 m, 0.4 m waypoint spacing - logged directly from
the real path-building call, not assumed). The proxy's `furthest_idx`
stayed at or below 16 for the *entire* arc phase and only crossed
`flat_mode`'s offset of 20 at **t=81.8 s**, several points into the
taper that follows the arc - i.e. **PathAlignCritic was completely
disabled for the whole arc-turn, every single time**, the identical
"activation problem" the descent leg had before its own fix, just
never applied to this profile. This is an activation problem, not a
weighting problem, for the arc-turn specifically - raising
`path_align_weight` further (already tried once, unsuccessfully, for
the descent leg before that root cause was found) would have kept
tuning a critic with nothing to weight.

### Task 2: what predicts drift magnitude - not payload, a structural path-length mismatch

Turn radius (`ARC_RADIUS = 2.0`, hardcoded) and the arc's own point
count (16, a direct function of that radius and the fixed 0.4 m
waypoint step) are constant across every run in this project's current
test suite - changing them would need code changes, out of scope for
a diagnosis session, but the relationship is exact and worth recording:
at 0.4 m spacing, `path_align_offset=20` structurally disables the
critic for the *entire* arc of any turn with radius up to `20*0.4/pi
≈ 2.55 m` - this isn't specific to a 2.0 m arc, any turn this small
would have hit the same wall.

**Payload was checked directly, side by side, same repro, same day**
(`ct1_diag.csv` loaded vs. `ct2_diag.csv` unloaded): the critic proxy
activated at **t=81.8 s (loaded)** vs. **t=81.9 s (unloaded)** -
essentially identical timing, both crossing `furthest_idx=20` within
0.1 s of each other. Leg-4 drift was also comparable in magnitude
regardless of payload (-6.06 m loaded, -5.30 m unloaded, this
session's pre-fix baseline). **Payload is not the dominant correlate**
- the structural offset-vs-arc-length mismatch reproduces identically
either way, and whatever residual run-to-run variance exists on top of
it is consistent with this project's already-documented MPPI sampling
stochasticity, not a payload effect.

One more signal checked before trusting any of this: `/odometry/
filtered` vs. Gazebo ground truth throughout the arc-turn, in case the
drift was actually a recurrence of the *original* wheel-odometry-yaw-
under-slip corruption the EKF was built to fix (see "Root-causing the
curved-world descent chatter" in the main README - that mechanism
produced 35-40 deg yaw errors). It is not: Y tracked within ~0.06-0.15 m
of ground truth throughout (the value that actually matters for
cross-track), and the consistent ~1.0-1.3 m X offset present from the
first sample matches the robot's own spawn-relative `odom` frame origin
exactly (a benign coordinate convention, not a growing error). The EKF
fix from that earlier session is holding; this is a genuinely different
mechanism.

### Task 3: three distinct mechanisms, not one shared "crosstrack weakness" - checked side by side, not assumed

- **Curved-world edge departures** (`ramp_world_curved.sdf`, the
  original crosstrack finding): per this project's own prior
  root-causing (see "Root-causing the curved-world descent chatter" in
  the main README), attributed to a genuine **physical stumble at a
  real geometric seam** (`fillet_top_1/2/3`, X=7.6-8.3 m), persisting
  even with `descent_mode`'s already-fixed `offset_from_furthest=3`
  live for that leg. The critic *is* active there; a real physical
  event still produces edge departure faster than any critic can react.
  Unrelated to this session's activation-gap finding.
- **Arc-turn HALT-thrash** (the elevated-level wandering/HALT cycling
  seen in every construction-site loaded run): checked directly against
  the launch log's own HALT-trigger messages during the exact repro
  windows above - **every single HALT during the arc phase was tagged
  either `Wheel slip X above threshold 0.5` or `Motion stall confirmed
  (LiDAR scan-consistency)`**, never anything tied to lateral position
  or PathAlignCritic (that critic isn't even a signal `state_machine_
  node.py`'s HALT logic reads - HALT is `margin_unsafe or slip_unsafe
  or stall_unsafe`, full stop). This is the already-understood
  wheel-slip/motion-stall-detector mechanism from earlier sessions,
  coincidentally co-occurring with the crosstrack issue because an arc
  turn is independently demanding for both (differential wheel slip is
  inherent to cornering; nothing about it engages or disables
  PathAlignCritic).
- **Leg-4 growing drift** (this session's actual target): the genuinely
  new finding - `flat_mode`'s `offset_from_furthest` never being
  lowered, confirmed both by source-code analysis and live
  instrumentation, above.

**These are not the same mechanism**, despite having been informally
grouped together as "the crosstrack weakness" - each has its own
confirmed, independent cause, and fixing one does not fix the others.

### Task 4: (a) - a specific, evidence-backed reweighting, implemented and verified this session

The evidence points cleanly at (a): a critic-configuration problem
within the current MPPI approach (an un-lowered `offset_from_furthest`
on one specific profile), not a structural path-source change or
anything not yet considered. `flat_mode`'s own "already proven
driftless" justification for leaving it untouched (in the original
comment) turned out not to actually be evidence about this setting at
all: legs 1/2/4 (the flat-ground driving that claim rests on) all use
raw `/cmd_vel_nav`, never `FollowPath` - PathAlignCritic isn't in that
loop regardless of its offset. The arc-turn is the *only* place
`flat_mode`'s PathAlignCritic setting is actually exercised by
`FollowPath`, and that's exactly where the gap was found.

**Fixed**: `ramp_mppi_params.yaml`'s `flat_mode.path_align_offset`
lowered 20 -> 3, matching `descent_mode`'s already-validated value (no
new number invented). `ramp_mode`/`transition_mode` were left
untouched - not implicated by any evidence gathered this session.

**Verified, same repro, same day** (`ct3_diag.csv`, loaded, post-fix):

| metric | pre-fix (`ct1_diag.csv`) | post-fix (`ct3_diag.csv`) |
|---|---|---|
| critic proxy first activates | t=81.8 s (idx 20) | t=9.5 s (idx 3) |
| arc-phase mean cross-track deviation | 0.218 m | 0.090 m (-59%) |
| arc-phase max cross-track deviation | 0.376 m | 0.237 m (-37%) |
| leg-4 final drift (this specific run) | -6.06 m | +3.37 m |

**Full official-protocol round-trip reverification** (stall detector +
goal-verification gate live, ground-truth polling, same metrics as
every prior reverification in this file), fresh launch:
`ct5_stage_a.csv`/`ct5_stage_b.csv` - `ok=True`, leg 3 completes
(`FollowPath` reports `STATUS_SUCCEEDED`, 102% verified progress), leg
4 completes and returns to the gate, **final drift only 0.76 m** - the
smallest of any run this project has logged for this maneuver, loaded
or unloaded, before or after any fix.

### An unplanned but real consequence of the fix: it broke last session's prop clearances

An intermediate post-fix full-round-trip run (`ct4_stage_b.csv`, same
protocol, before the final clean one above) reproduced the exact
"wheels spinning, chassis frozen, fused position runs away" signature
from the elevated-level seam bug - `/odometry/filtered`'s Y climbed
from ~1.8 m to 47.75 m over the back half of leg 4 while Gazebo's own
ground truth **froze at (3.42, 1.72) and never moved again**, with the
first HALT at that exact moment logged as `Motion stall confirmed
(LiDAR scan-consistency)`. Checked directly rather than assumed
coincidental: that frozen position is **0.58 m from `gate_cone_2`'s
center** (placed at (3.5, 2.3) last session, margined against the
*pre-fix* worst-case drift of -5.9 to -11.26 m). The fix correctly
shrank leg-4 drift - but a smaller, better-behaved drift threads
through a different part of the access road's width than the large,
blow-past drift the props were margined against. A second post-fix run
(`ct3_diag.csv`) ended at Y=+3.37 m, past the gate fence panels' own
Y=3.15 m position - two data points now span nearly the entire
navigable width of the 6.1 m-wide access road.

**Fixed by removing** `gate_fence_north/south`, `gate_cone_1/2/3`, and
`access_road_speed_sign` from `construction_site.sdf` (not by
re-guessing a new offset with only 2 data points) - see the world
file's own comment at that location for the full reasoning. Laydown/
office/waste-pad props are unaffected (all confirmed, both sessions,
well outside leg 4's observed or plausible reach). Access-road prop
placement needs leg 4's own open-loop drift either bounded or replaced
with something that has real lateral tracking before anything is
placed near that corridor again - a separate, not-yet-scoped follow-up,
not attempted this session.

Full data: `crosstrack_diagnosis_flatmode/` (the diagnostic script,
every pre/post-fix repro's CSV and log, both payload configs, and the
final clean full-round-trip reverification).

## Drift characterization: closing the crosstrack/drift work

The 0.76 m/-11.26 m pair above (and the single 0.58 m near-miss) is two
data points, not a distribution - not enough to size a real safety
margin or to decide, with any confidence, whether the removed gate-area
props (`gate_fence_north/south`, `gate_cone_1/2/3`,
`access_road_speed_sign`) could safely go back. This is the follow-up
that actually characterizes it: repeated, independent trials of the
leg-4 return leg, both payload configs, each on a genuinely fresh
`ros2 launch` + fresh sim (no persistent-session looping - see "Trial
protocol" below for why), min/max/mean/sample-stdev reported instead of
a single number, watching explicitly for convergence rather than
stopping at an arbitrary count.

### Trial protocol and infrastructure findings

Every trial: fresh `ros2 launch ramp_autonomy_bringup ... world:=
construction_site payload_mass:=<0 or 100> payload_cg_offset:=<0 or
0.1> headless:=true`, wait for `lifecycle_manager`'s "Managed nodes are
active", drive legs 1-2 (raw `/cmd_vel_nav` to the elevated platform),
leg 3 (`FollowPath` arc-turn + ramp descent, the same path-building code
as `arc_turn_followpath_test_ekf.py`), then leg 4 (raw `/cmd_vel_nav`
back to the gate) **only if leg 3's outcome is trustworthy** - see the
gate below - followed by one `gz topic -e -t /world/construction_site/
dynamic_pose/info` ground-truth sample. One CSV row appended per trial,
written only at the very end of a successful process (a killed/crashed
trial writes nothing, so partial data can't silently enter the set).
Script: `crosstrack_diagnosis_flatmode/drift_characterization/
drift_trials.py`; raw data: `drift_loaded.csv`/`drift_unloaded.csv` in
the same directory.

Three real infrastructure bugs were found and fixed while collecting
this data, each worth recording because they could silently corrupt
future trial-based characterization work in this repo:

1. **Looping many trials inside one persistent rclpy node/sim session**
   (to save relaunch overhead) self-corrupts: `gz sim` itself crashed
   mid-run on a real attempt at this, leaving the ROS graph alive and
   reporting `controller_server` "Optimizer reset" ~10,677 times over
   ~50 minutes and ground-truth positions that were physically
   impossible. Fixed by giving every trial a genuinely fresh launch -
   more expensive per trial, but the only version of this that produced
   trustworthy data.
2. **`pkill -9 -f "gz sim"`/`"ros2 launch"` intermittently fails to
   kill the target process** in this environment - repeatedly observed
   leaving a stale `gz sim` instance alive after a trial's own process
   exited, which, if a new launch starts before it's confirmed dead,
   produces two simultaneous simulation instances answering the same
   `gz topic` polls (both trials' ground truth then reads whichever
   instance happens to be alive at that instant - confirmed by two
   different "trials" reporting an identical, corrupted final
   position). Fixed by never trusting `pkill`'s exit status: always
   re-check with `ps aux | grep -iE "gz sim|ros2 launch"` after killing
   and fall back to killing exact PIDs, confirmed gone via `ps -p`,
   before starting the next launch. A related symptom of a launch
   racing a not-yet-dead previous instance: a flood of thousands of
   "Detected jump back in time. Clearing TF buffer." log lines with
   `lifecycle_manager` stuck mid-bringup - this needs to be killed and
   retried, not waited out.
3. **The leg-4 safety gate itself had a hole.** The original gate ran
   leg 4 whenever `not aborted and verified_fraction >= 0.5`. One trial
   produced `verified_fraction=3.186` (mathematically impossible - over
   100% of a single-pass path length) while `leg3_ok=False` and the
   `FollowPath` action itself never reported success - the turn had
   genuinely stalled (`trial_halt_count=220`, `min_margin=0.0`), but
   the inflated fraction passed the old gate anyway, and leg 4 then ran
   blind, assuming a heading the robot never actually reached (ended at
   gz_x=56.5, gz_y=4.36 - not a valid drift sample). Fixed: the gate now
   also requires `FollowPath`'s own action result to report
   `STATUS_SUCCEEDED` and `0 <= verified_fraction <= 1.5`, not fraction
   alone. That one contaminated row is still in `drift_unloaded.csv`
   (trial 6) for transparency but is excluded from every statistic
   below.

### Results: loaded (n=8, the binding case)

All 8 attempts completed leg 3 validly and produced a leg-4 drift
sample (Y-position at end of leg 4, ground truth):

`0.736, 1.403, 2.535, 0.565, 1.906, 0.526, 1.302, 1.130` m

| n | mean | stdev | min | max | range |
|---|---|---|---|---|---|
| 2 | 1.070 | 0.472 | 0.736 | 1.403 | 0.667 |
| 3 | 1.558 | 0.909 | 0.736 | 2.535 | 1.799 |
| 4 | 1.310 | 0.893 | 0.565 | 2.535 | 1.970 |
| 5 | 1.429 | 0.818 | 0.565 | 2.535 | 1.970 |
| 6 | 1.278 | 0.819 | 0.526 | 2.535 | 2.009 |
| 7 | 1.282 | 0.748 | 0.526 | 2.535 | 2.009 |
| 8 | **1.263** | **0.695** | **0.526** | **2.535** | 2.009 |

**Converged, not still swinging**: min, max, and range have been
identical since n=6; mean has moved by only 0.02 m over the last 3
trials; stdev has been monotonically shrinking (not oscillating) since
n=3. This is a real, characterized distribution as of n=8 - but it
converged to something inherently **wide**: a ~55% coefficient of
variation and a ~5x spread between the smallest and largest observed
trial (0.526 m to 2.535 m), all under identical commanded conditions.

### Results: unloaded (n=4 valid of 9 attempts)

`-0.171, 0.005, -0.203, -0.037` m -> min=-0.203, max=0.005,
mean=-0.102, stdev=0.101 (n=4). Tightly clustered near zero and
consistent in sign across every valid trial - unloaded leg-4 drift is
not the binding case for any margin decision.

**5 of the 9 unloaded attempts in this batch never produced a drift
sample** because leg 3 (the arc-turn) did not finish (`verified_fraction`
0.0, 0.067, 3.186 [the gate-bug row above], 0.013, 0.001;
`trial_halt_count` 0, 46, 220, 416, 695). **This was originally written
up here as a "50% leg-3 stall rate"; that reading was wrong and is
superseded by "Leg-3 completion recheck" below.** Every one of those
failures came from the later part of a session in which the cleanup
step killed only `gz sim`/`ros2 launch` and leaked every child ROS node,
so about 57 stale processes were running and host load average had
climbed to 71-76 on 12 cores. On a clean host the same maneuver
completed 8/8 unloaded and 7/8 loaded. The drift numbers above are
unaffected in kind (they came from completed legs), but they were
collected while that leak was building - see the caveat added under
"Decision" below.

### Decision: gate-area props stay removed - permanently, not pending further work

Applying this project's own established margining convention (used
elsewhere in `construction_site.sdf` for the laydown-area props: max
relevant excursion + 1.5 m safety buffer): **required clearance =
2.535 (loaded max, the converged worst case) + 1.5 = 4.035 m** from the
leg-4 path centerline. `access_road` is 6.1 m wide (`<size>8.000 6.100
0.1</size>`, Y in [-3.05, 3.05]) - its own half-width, 3.05 m, is less
than the required clearance. **No position within the drivable road
corridor can satisfy this margin**, regardless of how converged the
distribution is - the road itself isn't wide enough to hold a
safely-cleared fixed prop against this maneuver's own worst-case drift.

This is decision (b), not (a): the distribution IS real and converged
(not "still too wide/unpredictable to characterize" - it converged
cleanly by n=6-8), but what it characterizes is a worst case that
structurally doesn't fit the available road width. `gate_fence_north/
south`, `gate_cone_1/2/3`, and `access_road_speed_sign` stay removed
from `construction_site.sdf` permanently. This closes the crosstrack/
drift work opened in Session F: the open question was never "is the
fix good" (it is - loaded mean drift is down from an -11.26 m worst
case to a converged 1.263 m mean) but "is there a place gate-area props
could safely go," and the answer, backed by 8 independent trials per
condition, is no. Laydown/office/waste-pad props remain in place
(unaffected, per the separate 1197-point clearance check in Session E).

### Caveat on the decision above (added after the clean-host recheck)

Both drift sets above were collected while the trial harness was leaking
stale processes (next section). They were re-checked against clean-host
runs: loaded leg-4 drift on a clean host is n=7, mean 0.809 m, sd 0.423 m,
range 0.226-1.378 m (n=9 including the two diagnostic trials: mean
0.775, sd 0.390, same range) against the original n=8, mean 1.263 m, sd
0.695 m, range 0.526-2.535 m. The two sets are **not statistically
distinguishable at these sample sizes** (Welch t=1.55, df=11.7, p=0.148;
Mann-Whitney p=0.281); unloaded is indistinguishable (clean n=8 mean
-0.092 vs -0.102, p=0.89). So there is no evidence the original numbers
were contaminated, and **the decision stands - but it is more fragile
than the text above says**: it hangs on a max-of-n statistic. Under the
"max + 1.5 m" rule the original max (2.535 m) needs 4.035 m, which does
not fit the road's 3.05 m half-width, but the clean-host max alone
(1.378 m) needs 2.878 m, which would just fit. Under a tail-aware rule
(mean + 3 sd + 1.5 m) even the clean-host data alone needs 3.45-3.58 m
and does not fit. The props were left removed; a clean-host loaded drift
set of n>=16 would settle whether the 2.535 m value is a real tail or an
outlier, and has not been collected.

## Leg-3 completion recheck: the "50% stall rate" was a harness artifact

**Headline.** The arc-turn does not fail half the time. On a verified-
clean host it completed **15 of 16 planned trials (93.8%, Wilson 95%
interval 72-99%): unloaded 8/8, loaded 7/8**. Two further diagnostic
loaded trials also completed (17/18 overall). The earlier "4 of 9
unloaded attempts completed (~50% stall rate)" in the drift-
characterization section was measured while the harness was leaking
stale processes, and it is superseded by this section. The single clean-
host failure was **a harness timeout, not a physical stall**: loaded
trial 8 hit the script's 240.0 s cap while in normal `DESCENT` (margin
0.463, wheel slip ~0, position still advancing) about 33 s into a
descent that takes 36-48 s. This was measured at the old 240 s cap; the
cap is now 360 s (below), and that specific trial has not been re-run.

### 1. Where the false 50% came from

Between trials the drift-characterization runs killed only `gz sim` and
`ros2 launch` by name (`pkill -f`) and never checked the result. Every
child node survived every launch: `state_machine_node`,
`motion_stall_detector_node`, `cmd_vel_watchdog_node`,
`stability_monitor_node`, `ros_gz_bridge`'s `parameter_bridge`, and
`robot_state_publisher`. By the time this was noticed there were **57
stale processes and a load average of 71-76 on 12 cores**, and the
`halt_count` of successive unloaded trials climbed 46, 220, 416, 695.
After a full by-PID sweep the same maneuver completed 8/8 unloaded.
**This is strong circumstantial evidence, not a controlled A/B**: no
starved-vs-clean rerun of the same trial was done, the old runs had no
per-leg halt attribution, and the real-time factor was not recorded for
them. What was measured directly is that two simultaneous sim instances
cannot both keep real time on this host (below).

### 2. What a leg-3 "stall" actually is

Every clean trial's leg 3 has the same two-phase shape: a **platform
phase of 156-204 s** (state alternating `EXIT`/`HALT`; verified progress
climbs steadily throughout) followed by a **`DESCENT` of 36-48 s**. A
nominal 6.3 m, R=2 m arc at 0.4 m/s would take ~16 s, so the platform
phase is roughly ten times slower than it should be, and it sits right
against the old 240 s cap (clean n=16: mean 211.3 s, sd 16.5 s, max
240.0 s where that value is the cap itself).

Fast-rate telemetry (`stall_highrate_trial.py`, loaded trial 901,
1.0 M messages; `analyze_highrate.py`, output in
`analysis_highrate_loaded_901.txt`) shows where the time goes. In leg 3
(209.1 s) there were 15 HALT episodes: **11 LiDAR stall latches
totalling 100.7 s (48.2% of leg 3), median 10.00 s each**, and **4
wheel-slip HALTs totalling 0.4 s (0.2%)**, 0.1 s each. Across all 16
clean trials the FSM log has 693 slip-HALT entries (~43/trial, at most a
few seconds in total) against 346 stall latches (~22/trial, at least
~216 s of a 430-510 s trial).

**The hypothesis that brief slip HALTs stop the robot building speed
and thereby trip the stall latch is refuted.** Only 1 of 11 latches had
a slip HALT in the preceding 6 s; command was nonzero for 100% of the
5 s before each latch; wheel lag after a command resume is short
(median 0.20 s, p90 0.26 s). The slip metric does have a real flaw -
all 4 slip readings came from a near-zero commanded wheel rate (median
0.16 rad/s), a small-denominator artifact, and the published slip
reached 3+ - but it costs 0.2% of leg 3.

**The dominant cause is the stall detector reporting a stall on a robot
that is moving.** A second recording (`stall_scan_trial.py`, loaded
trial 902, raw `/scan` and `/scan_360`; `analyze_scan.py`, output in
`analysis_scan_loaded_902.txt`) replays
`motion_stall_detector_node.py`'s exact algorithm offline against real
motion:

- **All 16 latches in the trial fired with clear real motion in the
  prior 5 s** (1.0-2.15 m travelled, up to 50 deg turned).
- On the open platform `/scan_360` returns 0/360 finite beams, so the
  detector falls back to `/scan`. That sensor's ~87 finite beams are
  (a) the robot's own rear structure at 0.09-0.31 m and (b) the flat
  floor ahead as a smooth arc (0.59 m at -45 deg rising to 2.86 m at
  -8 deg). Both are invariant to translation and rotation on a flat
  plane; per-beam variation over time is ~0.01 m. The metric therefore
  reads a mean change of ~0.011 m against a 0.05 m threshold whatever
  the robot does.
- Of the samples where the robot had truly moved (>0.5 m or >20 deg in
  5 s), the detector said "no movement" for **93% on the platform
  (123/132)**, 37% in leg 2 (ramp + platform), 12% in leg 1 (flat drive,
  `/scan_360` moving along walls), and **0% on the descent slope and in
  leg 4** where the scan geometry changes.
- Each 5 s of commanded motion therefore latches a >=10 s HALT, a ~33%
  motion duty cycle. That is the "wander".

The scan sensors are the only motion evidence the detector uses because
wheel-derived odometry (and the EKF fed by it) is exactly what lies
during the physical freeze this detector was built to catch. So this is
a genuine sensor-observability limit on a featureless flat platform,
not a threshold typo.

### 3. Why it was not caught earlier, and what did not cause it

The maneuver always completed; the cost is time. The same cap-vs-slow-
run effect had already forced the cap up once (180 s to 240 s, see the
comment in `arc_turn_followpath_test_ekf.py`), and the wander was
already recorded as "HALT-thrash roughness" in the crosstrack diagnosis
above. It is **not** a regression from the `flat_mode` `path_align`
change or the leg-4 gate: neither can influence wheel-slip or scan
behaviour by code dependency (PathAlignCritic reweights tracking cost;
the gate runs after leg 3). That is a dependency argument only - the
workspace is not a git repository, so no before/after run was possible.

### 4. Decisions

1. **Slip threshold / hysteresis: not changed.** The telemetry says it
   would not speed the maneuver (0.2% of leg 3), and this is a hardened
   safety threshold. If it is ever revisited the concrete change is to
   compare against a lag-compensated command and ignore wheels whose
   commanded rate is below a floor; it would need a low-friction run to
   show real slip still trips HALT.
2. **Stall detector: not changed at that point, documented as the narrower remaining
   investigation - since fixed for rotation, see "Stall detector: positive control, fix and validation" below.** Removing the false latches is a design change, not a
   tune: options are (a) treat the scan verdict as "unobservable" when
   the recent scans are flat-floor/own-body only, which removes false
   latches but also blinds the detector to a real freeze on the
   platform; (b) add a gyro cross-check, which observes real rotation
   independent of the wheels and would cover the turning phase but not
   straight travel; (c) a sensor that can see translation over a plain
   floor. Any of them needs a **positive control** (a real freeze, for
   example the robot wedged against a wall on the platform, must still
   latch within ~7 s) that does not exist yet, plus a clean-host
   completion campaign. Expected benefit if solved: platform phase
   ~160-200 s down to roughly 20-30 s. Deliberately not done blind.
3. **Completion-time cap 240 s to 360 s** in `drift_trials.py`,
   `stall_diagnosis_trials.py` and `arc_turn_followpath_test_ekf.py`
   (mean 211.3 s, sd 16.5 s, the 240 s value being censored; 360 s is
   about mean + 9 sd; mean + 3 sd would be 260.8 s). Trial 902 ran at
   the new cap and finished in 180.2 s.
4. **Harness hardening** (`validation/phase8/harness/harness_guard.py`).
   All checks are by PID from `/proc`: `preflight` (zero stale sim/ROS
   processes and an instantaneously idle host), `verify` (after launch,
   before any motion: exactly one sim and one launch process, no
   duplicated node executables, load under 16, simulator real-time
   factor >= 0.95 measured over a 5 s window), `cleanup` (kill by PID
   and prove they are gone), and a `monitor` that records min RTF and
   max load during a trial so a starved trial identifies itself. Every
   failure prints a banner and exits 2 - the caller stops. The three
   trial scripts call `verify` before any motion and write `guard_*`
   columns (and refuse to append to a CSV with a different header);
   `run_matrix_regression_cleanhost.sh` gates every matrix cell the same
   way and aborts the whole run on a failure. Tested with a decoy stale
   process (preflight fails, cleanup removes it) and a decoy duplicate
   `state_machine_node` (verify fails); a first decoy with the wrong
   command-line shape passed, which was a flaw in the test, not the
   guard. `kill_instance.sh` (by Gazebo partition) exists for isolated
   instances.
5. **Parallel trials: tried and rejected.** One instance holds a
   real-time factor of 1.000 using ~412% of the 1200% CPU available;
   two isolated instances (separate `ROS_DOMAIN_ID` and `GZ_PARTITION`)
   swung between 0.29 and 1.17, and pinned to disjoint core sets between
   0.49 and 1.26. Trial timeouts are wall-clock, so that would recreate
   the artifact above. Trials stay sequential.

Data and tools in `crosstrack_diagnosis_flatmode/drift_characterization/`:
`stall_diag_unloaded.csv`, `stall_diag_loaded.csv` (16 planned trials),
`stall_diag_loaded_highrate.csv` / `highrate_loaded_901.csv`,
`stall_diag_loaded_scan.csv` / `scan_loaded_902.npz`, the analysis
outputs, `stall_diagnosis_trials.py`, `stall_highrate_trial.py`,
`stall_scan_trial.py`, `analyze_highrate.py`, `analyze_scan.py`.

## Clean-host regression pass (one run of each, harness-guarded)

Run once each on a host verified clean by `harness/harness_guard.py` (zero
stale processes and an idle CPU before every launch; one sim instance, no
duplicated nodes and real-time factor >= 0.95 verified before any motion;
RTF/load monitored throughout, and a starved run would have aborted the
whole pass). Every run below finished with `starved: False` (minimum RTF
0.963-0.995, maximum load 3.8-8.2). These are **single runs**: MPPI is
stochastic, so a one-grade change in a verdict is within run-to-run
variation and not evidence of a regression or improvement.

### `validation_test_ekf_groundtruth.py`, 7 cells (`run_matrix_regression_cleanhost.sh`)

| World | Friction | Payload | Ascent | Descent | HALTs | Ground truth | Overall | Prior trusted matrix |
|---|---|---|---|---|---|---|---|---|
| sharp | normal | unloaded | PASS | PASS | 2 | PASS (max\|y\| 0.10 m) | MARGINAL | PASS (0 HALTs) |
| sharp | normal | loaded | FAIL: DID_NOT_CLIMB | N/A | 7 | N/A | FAIL | FAIL: DID_NOT_CLIMB |
| sharp | 0.45 | loaded | FAIL: DID_NOT_CLIMB | N/A | 8 | N/A | FAIL | FAIL: DID_NOT_CLIMB |
| curved | normal | unloaded | PASS | PASS | 11 | PASS (max\|y\| 1.01 m) | FAIL: HALT_THRASH | FAIL: edge departure (2.68 m, 4 HALTs) |
| curved | 0.45 | unloaded | PASS | PASS | 3 | PASS (max\|y\| 0.31 m) | MARGINAL | MARGINAL (1 HALT, 0.36 m) |
| curved | normal | loaded | PASS | PASS | 4 | PASS (max\|y\| 0.94 m) | MARGINAL | MARGINAL (4 HALTs, 0.97 m) |
| curved | 0.45 | loaded | PASS | PASS | 3 | PASS (max\|y\| 0.93 m) | MARGINAL | FAIL: HALT_THRASH (9 HALTs) |

Two cells are the known, unrelated physical constraint (the sharp world's
loaded robot cannot climb) and reproduce exactly. Ground truth passed in
every cell that reached the descent in this run, including `curved/normal/unloaded`
(1.01 m against 2.68 m before). **Correction, added later the same night: that
1.01 m pass was an outlier, not an improvement** - the cell fails ground truth in
7 of 8 runs (see "Open finding: curved / normal / unloaded" at the end of this
file); the original sentence read it as evidence of run-to-run variation. That
cell's overall FAIL in this run was the HALT count (11), and the HALT
counts move up and down between the two runs (4 to 11, 9 to 3, 1 to 3,
0 to 2) in a way that fits run-to-run variation. No cell got a worse
ground-truth result than the prior matrix.

### Construction-site round trip (`arc_turn_followpath_test_ekf.py`, populated site)

| Payload | Leg 3 | Verified progress | Leg 4 | HALTs (total / descent) | Min margin | Max slip | Final leg-4 position (gz) |
|---|---|---|---|---|---|---|---|
| loaded | `ok=True` | 102% (28.85 of 28.25 m) | reached gate | 24 / 12 | 0.463 | 5.17 | (2.00, +1.57) |
| unloaded | `ok=True` | 102% (28.72 of 28.25 m) | reached gate | 40 / 26 | 0.872 | 8.53 | (1.96, -0.10) |

No safety abort in either run, correct 16 deg pitch profile, both
completed at the new 360 s leg-3 cap. The loaded run's 1.572 m leg-4
drift is a further clean-host loaded drift sample (clean set now n=10:
mean 0.855 m, sd 0.446, range 0.226-1.572). It **strengthens** the prop
decision: the clean-host max alone now needs 1.572 + 1.5 = 3.072 m, just
over the road's 3.05 m half-width, so the props stay removed under the
"max + 1.5 m" rule on clean data as well as under the tail-aware rule
(3.69 m).

## Stall detector: positive control, fix and validation

**Result.** The stall detector was reporting a stall on a robot that was moving. Against Gazebo ground truth,
100% of its latches in normal runs were false (35 of 35), and it kept the robot latched for 50.2% of leg 3. A
gyro-integrated rotation check (`motion_stall_detector_node.py`) removes **every** false latch that occurs
while the robot is rotating (14 -> 0), cuts leg-3 time spent stall-latched from 50.2% to 27.4% and mean leg-3
duration from 185 s to 123 s, and detected all 18 of 18 forced genuine freezes (95% interval 82-100%).
**It does not fix false latches during straight travel**, which remain: no inertial sensor can tell steady
straight motion from a blocked chassis (constant velocity is indistinguishable from rest), so this is a
sensor-suite limit, not an untuned threshold.

### 1. The positive control (`validation/phase8/stall_detector_control/`)

A test that produces a genuine, verifiable freeze on demand, independent of the detector under test.
`freeze_session.py` runs the normal maneuver and, at up to three points while the chassis is being commanded to
move, spawns a **low four-walled cage** around it through Gazebo's create service: `straight_raw` (leg 2, raw
straight drive, 3.5 m onto the platform), `arc` (leg 3, once the turn is >= 30 deg) and `taper` (leg 3,
near-straight travel, turn >= 170 deg). It stays 20 s, is removed, and the maneuver continues.

Why a cage, and why low. The original loaded freeze was wheels spinning at the commanded rate while the chassis
stayed put against a 5 cm step. A cage reproduces that mechanism with real contact physics, on demand, with a
ground-truth-verifiable outcome, and touches neither the robot nor the detector. It is 0.30 m high on purpose:
the horizontal ring LiDAR sits 0.5 m above the base and cannot see it, exactly as it could not see the step.
Rejected: a single frontal wall (a skid-steer robot can still pivot, so it is not a clean freeze) and resetting
the robot's pose (injects fake IMU impulses into the very signal being tested).

Calibration (one dry run, 5 baseline sessions, 12 validation sessions). Spawn latency 0.30 s; predicting the pose
0.30 s ahead placed the walls correctly (in the dry run, with 0.10 m gaps, the robot stopped 0.6-0.8 s after the
call and 0.18-0.22 m later, as predicted). Clearances front 0.06 / back 0.08 / side 0.04 m. Measured stillness after the 2 s settle:
`straight_raw` **0.000 m drift, 0.0 deg**; `taper` <= 0.05 m, <= 5.5 deg; `arc` up to 0.11 m and 6 deg (the
robot rocks in the wedge rather than jamming solid), which is why blocked is defined on **net** motion.

The fixture is the right kind of freeze. In 23 episodes where the chassis was provably blocked but commanded to
turn:

| Observer | yaw rate while blocked (median, p10-p90) | Truth |
|---|---|---|
| Commanded | 0.095 (0.083-0.113) rad/s | - |
| **Raw wheel odometry** | **0.095 (0.086-0.113)** | reports the command: lies |
| **IMU gyro** | **0.001 (0.000-0.044)** | true |
| **EKF** | **0.001 (0.000-0.045)** | true (it fuses IMU yaw and yaw rate, only wheel *linear* velocity) |
| wheel speed / commanded | 1.01 (0.99-1.21) | wheels spin at the commanded rate |

That is the original freeze signature, and it confirms that rotation is observable from the IMU/EKF but never
from wheel odometry.

**Scoring** (`analyze_events.py`, all on ground truth, never robot sensors). *Blocked*: net displacement < 6 cm
and net yaw < 0.10 rad over 2 s. *Moving*: >= 0.20 m or >= 0.10 rad over 2 s. A **commanded-and-blocked
episode** is a maximal interval where the actuation command is nonzero and the chassis is blocked, so HALT
pauses (which zero the command) never count. A latch is **false** if the robot was moving, **justified** if it
was commanded and blocked for the preceding 2 s. Detection is scored per episode long enough to require a latch
(detected, or blocked >= 8 s). `replay_detectors.py` reruns the node's exact latch logic on recorded inputs; for
the old rule it reproduces the live node's latch count exactly (16/16, 19/19).

### 2. The fix

`motion_stall_detector_node.py` now integrates the IMU's gyro z over a 2 s window and, if |yaw change| >= 0.10
rad, overrides the scan's "no movement" verdict (`YAW_EVIDENCE_WINDOW_S`, `YAW_EVIDENCE_MIN_RAD`). It uses the
gyro directly (the node already subscribes to `/imu`); the EKF gave the same answer but adds a dependency.
Nothing else in the detector changed, and the LiDAR check, 5 s window, 10 s latch and cooldown are as before.

The threshold was chosen from labelled windows, not guessed (`gyro_threshold_study.py`). |Integrated yaw| over
2 s: blocked arc fixture p95 0.096 / max 0.103, blocked taper max 0.090, blocked straight exactly 0.000, against
moving-and-rotating p5 0.115 / median 0.223. At 0.10 rad, 99.8% of rotating windows are suppressed while 0.9% of
blocked windows are vetoed (2.3% in the rocking arc fixture). Caveat: "rotating" is itself defined at 0.10 rad,
so the suppression figure is partly by construction; the blocked-side numbers are the independent evidence.

**Gyro noise caveat.** The simulated IMU has no noise model. `gyro_bias_robustness.py` adds a worst-case bias to
the recorded signal: at 0.005-0.02 rad/s (typical MEMS) 4-10% of blocked windows are masked per evaluation and
4-12% of rotating windows still latch; at 0.05 rad/s every freeze is masked. Because the check is re-evaluated
20 times a second that mostly delays detection at realistic bias, but on real hardware the gyro bias should be
subtracted using standstill (observable at every HALT). Not implemented: it cannot be validated in this sim.

### 3. Why straight travel is not covered

A rotation check cannot help when the robot is not rotating, and there is nothing else to use.
`straight_features.py` scored 13 IMU features (accelerometer/gyro spread, pitch, integrated acceleration...)
on ground-truth-labelled steady straight travel (1414 windows) against blocked windows: **none separates them**
(the best single feature, accelerometer y spread, reaches |AUC - 0.5| = 0.25, which the script itself labels
"weak", from only 27 blocked windows, several containing the contact transient; the perfectly frozen `straight_raw`
fixture has exactly the same accelerometer statistics as steady motion). That is Galilean invariance, not a tuning gap. Integrating
acceleration from rest is not viable either (gravity leakage from a 1 deg pitch error is 0.17 m/s^2, and the ramp
is 16 deg). Only an exteroceptive sensor that sees the floor move (or structure on the platform) closes it.

### 4. Validation (live detector, fresh guarded sim per session, none starved; min RTF 0.962-0.995, max load 7.0)

Normal sessions (no fixture), every latch scored against ground truth:

| | old detector (n=2) | fixed detector (n=6) |
|---|---|---|
| Latches on a rotating robot (leg-3 platform) | **14** (7 per session) | **0** |
| False latches in leg 3, per session | 10.5 (sd 0.7) | **4.3 (sd 1.2)** |
| False latches per session, all legs | 17.5 (sd 2.1) | 12.2 (sd 2.4) |
| Share of latches that are false | 35/35 | 73/73 |
| Leg-3 time spent stall-latched | 50.2% | **27.4%** |
| Leg-3 duration | 185.1 s (sd 7.2) | **123.2 s (sd 7.3)** |
| Leg 3 completed | 2/2 | 6/6 |

Fixed-detector false latches by phase (all straight): leg 1: 23, leg 2: 24, leg-3 descent: 8, leg-3 platform
(taper): 18. The old n=2 baseline is small; it is consistent with the earlier independent recording (48.2%
stall-latched in leg 3) and with the FSM logs of the 16 clean-host trials (~22 stall latches each).

Positive control, forced freezes (commanded-and-blocked episodes):

| Event | old (n=3 sessions) | fixed (n=6 sessions) | latency, fixed (median / max) |
|---|---|---|---|
| straight_raw | 3/3 | 6/6 | 5.0 / 5.0 s |
| arc (mid-turn) | 6/6 | 6/6 | 4.2 / 5.0 s |
| taper | 5/5 | 6/6 | 4.8 / 5.0 s |
| **all** | **14/14 (78-100%)** | **18/18 (82-100%)** | |

Turning freezes are detected later than before (median 4.2 s vs 2.4 s) because the check waits for rotation to
stop; all are within the 8 s deadline. **Stricter reading:** counting any blocked episode of >= 5 s that was not
latched as a miss gives 14/15 (old) and 18/20 (fixed, 90%, interval 70-97%). Both misses are arc episodes in the
rocking fixture where the veto was inactive (gyro integral <= 0.067 rad); replaying the *old* rule over the same
inputs also never fires there. The 5 s commanded window only completes at the very end of a 5.2 s rocking
episode, when the scan already shows 0.076 m of movement, so it is a pre-existing timing edge case, not an effect
of the fix. Sample sizes are modest: this supports "detects the forced freezes", not a claim above ~95%.

### 5. Status and what is left

**Known adverse effect (found afterwards): the fix regresses the curved-world cell `curved / normal / unloaded` from 0 of 6
to 10 of 10 ground-truth failures - see "Controlled detector comparison" at the end of this file.** The false-latch problem
is fixed for rotation and the platform's turning phase; the maneuver is ~33% faster and still completes 6/6. Left open, and physical rather than tunable: false latches during straight travel (~12 per
session, each a >= 10 s HALT) and gyro-bias handling on real hardware. Candidate remedies for straight travel:
an exteroceptive sensor that observes floor motion, or geometry on the platform for the horizontal LiDAR. Tools
and data: `validation/phase8/stall_detector_control/` (`freeze_session.py`, `run_session.sh`, `analyze_events.py`,
`replay_detectors.py`, `aggregate.py`, `gyro_threshold_study.py`, `gyro_bias_robustness.py`,
`straight_features.py`, `data/` recordings and scores), and the original detector is kept as
`data/motion_stall_detector_node.ORIGINAL.py`.

### Regression rerun on the fixed stall detector (one run each, harness-guarded, none starved)

The pass above ran on the old detector; the detector was then changed, so the same matrix and both round trips
were rerun on the fixed one (logs in `matrix_regression_newdetector_logs/` and `roundtrip_newdet_*`; the earlier
results were not overwritten). Minimum RTF 0.969-0.999 throughout. Single runs of a stochastic controller.

| World / friction / payload | Prior trusted matrix | Run on old detector | **Run on fixed detector** |
|---|---|---|---|
| sharp / normal / unloaded | PASS (0 HALTs) | MARGINAL (2; 0.10 m) | **MARGINAL (3; 0.08 m)** |
| sharp / normal / loaded | FAIL: DID_NOT_CLIMB | FAIL (7) | **FAIL: DID_NOT_CLIMB (6)** |
| sharp / 0.45 / loaded | FAIL: DID_NOT_CLIMB | FAIL (8) | **FAIL: DID_NOT_CLIMB (9)** |
| curved / normal / unloaded | FAIL: edge departure (4; 2.68 m) | FAIL: HALT_THRASH (11; GT PASS 1.01 m) | **FAIL: HALT_THRASH (14); GT FAIL edge departure 2.49 m** |
| curved / 0.45 / unloaded | MARGINAL (1; 0.36 m) | MARGINAL (3; 0.31 m) | **MARGINAL (4; 0.23 m)** |
| curved / normal / loaded | MARGINAL (4; 0.97 m) | MARGINAL (4; 0.94 m) | **MARGINAL (3; 1.08 m)** |
| curved / 0.45 / loaded | FAIL: HALT_THRASH (9; 0.93 m) | MARGINAL (3; 0.93 m) | **MARGINAL (2; 1.07 m)** |

Cells in parentheses: HALT count; ground-truth max |y|. Six of seven cells match the earlier results in kind (same
verdict class, HALT counts within a few). **`curved/normal/unloaded` failed ground truth (edge departure, 2.49 m)**,
where the old-detector run had passed (1.01 m). This is not evidence against the fix: the cell has now failed
edge departure in 2 of 3 runs overall (2.68 m before, 2.49 m now) and passed once, the edge departure is the
seam-stumble mechanism documented under "Crosstrack diagnosis", and the stall-latch HALT count in that cell is
**3 in both the old-detector and fixed-detector runs** (the rest are wheel-slip HALTs, 24 vs 42). One run per
condition cannot separate a detector effect from run-to-run variation, so it was reported as unresolved-but-
unattributed rather than dismissed. **Superseded by the five-run repeat below**: the cell fails in 7 of 8 runs, and
the stall-latch count comparison above rested on a single run per condition (across six fixed-detector runs it
ranges 2-7), so it does not rule the detector in or out.

Construction-site round trip, populated site:

| Payload | Detector | Leg 3 | Verified | Leg 4 | HALTs (total / descent) | Min margin | Max slip | Final leg-4 pos (gz) |
|---|---|---|---|---|---|---|---|---|
| loaded | old | `ok=True` | 102% | reached gate | 24 / 12 | 0.463 | 5.17 | (2.00, +1.57) |
| loaded | **fixed** | `ok=True` | **103%** | reached gate | **14 / 4** | 0.463 | 2.11 | (2.02, +1.17) |
| unloaded | old | `ok=True` | 102% | reached gate | 40 / 26 | 0.872 | 8.53 | (1.96, -0.10) |
| unloaded | **fixed** | `ok=True` | **103%** | reached gate | **30 / 15** | 0.872 | 4.61 | (2.06, +0.00) |

Both round trips complete on the fixed detector with no abort and the same margins; HALT counts are lower in both
(the direction the fix predicts, but a single run each). The extra loaded leg-4 drift point (1.169 m) brings the
clean-host loaded set to n=11, mean 0.883 m, sd 0.433, max 1.572 m, so the prop decision is unchanged.

## Open finding: curved / normal / unloaded fails ground truth in 7 of 8 runs

Status: **the detector question is now RESOLVED by the controlled comparison in the next section (the stall-detector fix is implicated). The mechanism was not investigated. Text below is the state of knowledge before that comparison.**

Question asked: is the cell's failure a stable rate, or a rare event that has been unlucky twice? Five more runs of
this exact cell (`run_cell_repeat_curved_unloaded.sh`: verified cleanup, preflight, fresh launch, one-instance and
RTF >= 0.95 verify, RTF/load monitor, the unchanged `validation_test_ekf_groundtruth.py`, 220 s cap), one at a time
on the guard-verified clean host, on the currently installed (fixed) detector. None starved (min RTF 0.995-0.998,
max load 3.1-4.7). Ground-truth metric as before: max |y| during the descent against the 1.5 m panel half-width.

| # | Run | Detector | Host verified clean? | GT max \|y\| | GT verdict | HALTs | Stall / slip / margin HALT entries | Overall |
|---|---|---|---|---|---|---|---|---|
| 1 | earlier trusted matrix | old | no (pre-guard) | 2.68 m | FAIL: edge departure | 4 | n/a | FAIL |
| 2 | clean-host regression | old | yes | 1.01 m | **PASS** | 11 | 3 / 24 / 0 | FAIL (HALT_THRASH) |
| 3 | fixed-detector regression | fixed | yes | 2.49 m | FAIL: edge departure | 14 | 3 / 42 / 1 | FAIL (HALT_THRASH) |
| 4 | repeat r1 | fixed | yes | 1.74 m | FAIL: edge departure | 6 | 2 / 34 / 2 | FAIL |
| 5 | repeat r2 | fixed | yes | 2.00 m | FAIL: edge departure | 16 | 7 / 46 / 1 | FAIL (HALT_THRASH) |
| 6 | repeat r3 | fixed | yes | 2.01 m | FAIL: edge departure | 9 | 7 / 35 / 0 | FAIL (HALT_THRASH) |
| 7 | repeat r4 | fixed | yes | 2.92 m | FAIL: edge departure | 13 | 3 / 29 / 2 | FAIL (HALT_THRASH) |
| 8 | repeat r5 | fixed | yes | 2.34 m | FAIL: edge departure | 16 | 5 / 43 / 0 | FAIL (HALT_THRASH) |

**Count: ground truth FAIL 7 of 8, PASS 1 of 8 (Wilson 95% interval for the failure rate 53-98%); overall verdict
FAIL 8 of 8.** Edge-departure magnitudes of the seven failures: 1.74, 2.00, 2.01, 2.33, 2.49, 2.68, 2.92 m (mean
2.31 m, i.e. 0.24-1.42 m past the 1.5 m half-width). On the fixed detector alone: 6 of 6 fail, 1.74-2.92 m (mean
2.25 m). So this is a **stable failure with one outlier pass**, not a rare event; the earlier "failed 2 of 3, passed
once" undercounted it, and the 1.01 m pass in the clean-host regression should not have been read as improvement.

What this does and does not show:

- It does not show the stall-detector change is the cause. The old detector has only two runs (one fail on an
  unverified host, one pass); 6 of 6 versus 1 of 2 failing is not distinguishable at these sizes (Fisher one-sided
  p = 0.25). It does not show the detector is innocent either: the earlier "identical stall-latch HALT count (3)"
  argument compared one run with one, and across the six fixed-detector runs that count ranges 2-7. Wheel-slip HALTs
  dominate every run's HALT count (24-46 entries against 2-7 stall entries).
- The eight runs are not one homogeneous sample (run 1 predates the harness guard; only run 2 is a clean-host run on
  the old detector), which is a further reason the detector question stays open.
- The mechanism was not investigated. The documented seam-stumble edge departure ("Crosstrack diagnosis") is a
  hypothesis for this world, untested here.

Next step to separate a detector effect from the world: repeat this cell 5 or more times on the ORIGINAL detector
(`stall_detector_control/data/motion_stall_detector_node.ORIGINAL.py`, reinstalled and rebuilt). Not done tonight
because it is a code change. Raw per-run output: `validation/phase8/curved_unloaded_repeat_logs/`.

## Controlled detector comparison: curved / normal / unloaded

**Question:** is the stall detector a contributing factor in this cell's ground-truth failures (fixed detector: 6 of 6
failing; old detector: 1 of 2, p = 0.25)? **Answer: yes. The fixed detector is implicated; the original passes.**

Design. Same cell, same instrument (`run_cell_repeat_curved_unloaded.sh`: verified cleanup, preflight, fresh launch,
one-instance verify, RTF/load monitor, the unchanged `validation_test_ekf_groundtruth.py`, 220 s cap), one sim at a
time on the guard-verified clean host. Only `motion_stall_detector_node.py` was swapped, rebuilt with
`colcon build --packages-select ramp_autonomy_control` and checked by checksum (installed copy `346608b6...` =
`data/motion_stall_detector_node.ORIGINAL.py`; zero rotation-check symbols). Afterwards the fixed detector was
reinstalled and checked (`48625b76...` = source = `data/motion_stall_detector_node.FIXED.py`). The decision rule
(exact Fisher on failure counts, exact permutation test on departure magnitude, p < 0.05) was fixed before the runs
(`compare_detectors_curved_unloaded.py`). Ground-truth fail = max |y| beyond the 1.5 m panel half-width.

| Group | n | Ground-truth fails | max \|y\| per run (m) | mean (sd) | Overall verdict | Stall / slip HALT entries (mean) | Min RTF |
|---|---|---|---|---|---|---|---|
| **Original detector** (controlled) | 6 | **0 / 6** | 0.97, 0.98, 0.39, 1.01, 0.94, 1.11 | **0.90 (0.26)** | 3 MARGINAL, 3 FAIL (HALT_THRASH: 16, 12, 8 HALTs) | 3.2 / 17.7 | 0.996-0.999 |
| **Fixed detector**, before the swap | 6 | 6 / 6 | 2.48, 1.74, 2.00, 2.00, 2.92, 2.33 | 2.25 (0.42) | FAIL | 4.5 / 38.2 | 0.995-0.998 |
| **Fixed detector**, control after the swap | 4 | 4 / 4 | 3.00, 3.85, 2.02, 2.57 | 2.86 (0.83) | FAIL | | 0.991-0.998 |
| Fixed, all | 10 | **10 / 10** (Wilson 72-100%) | 1.74-3.85 | **2.49 (0.63)** | FAIL | 4.9 / 38.2 | |

Original vs all fixed: Fisher two-sided **p = 0.00012**; permutation test on departure magnitude **p = 0.00012**;
mean difference (original - fixed) **-1.59 m, bootstrap 95% CI -2.03 to -1.20**. Original failure rate 0/6 (Wilson
0-39%). Order and host drift: the fixed detector reinstalled *after* the original runs failed 4/4 again, against 6/6
before (Fisher p = 1.0; magnitude permutation p = 0.12, the later runs being if anything larger). Real-time factor:
two control runs dipped to 0.991 and 0.994, under the requested 0.995 bar; restricting to runs that meet it gives
fixed 8/8 failing vs original 0/6, p = 0.00033, same conclusion. No run was starved.

**Conclusion, stated plainly:** the failure is *not* purely a curved-world path-tracking problem independent of the
stall detector. With the original detector this cell passes (6/6, all within the panel); with the fixed detector it
fails every time (10/10), by 0.24-2.35 m past the panel edge. The stall-detector change is a contributing factor,
and the earlier ambiguity (6/6 vs 1/2) was a sample-size problem. The earlier single old-detector failure (2.68 m)
was on a host never verified clean; pooled, the old detector is 1 failure in 8.

**How it differs** (association, not a demonstrated mechanism; the mechanism was not investigated):
- **Corrected:** every run in every group, including all 10 fixed-detector runs, has a wheel-slip HALT 0.3-0.65 s BEFORE the first `DESCENT` (the robot stops at the crest in all of them; an earlier version of this note claimed the fixed runs had no crest HALT, which used a misaligned time window and was wrong). What the original detector adds is a SECOND stop inside `DESCENT`: a "Motion stall confirmed" latch 0.4-0.8 s after `DESCENT` begins (6 of 6 original runs, 0 of 10 fixed), a 1.4-2.2 s pause, a wheel-slip HALT on resume at +2.2-2.7 s, and `descent_brake_node`'s cautious restart at 40% of the acceleration ceiling (6 of 6 vs 0 of 10).
- Wheel-slip HALT entries per run are ~2x more frequent with the fixed detector (mean 38 vs 18, p = 0.002 on the 6-vs-6
  comparison); stall-latch entries are similar (4.9 vs 3.2).
- This is perfectly confounded with the detector version: it cannot distinguish "the pause is what keeps the descent on
  the panel" from something else the change does. Whether the crest latch in the original is itself a false latch of
  the kind the fix removes is unverified (the matrix test records no ground truth before the descent).

**Scope and what to decide.** The other curved cells on the fixed detector stayed within their earlier verdicts (0.23,
1.08 and 1.07 m), so the effect is not universal. The same change removes every rotation false latch and cuts leg 3 on
the construction site by ~33%. (Superseded: the decision was made to restrict the rotation check to non-`DESCENT` states; see "State-scoped detector: causal test and final validation" below.) The fixed detector was left installed pending that decision; keeping it, restricting the rotation
check (for example to non-ramp states) and re-running both validation sets, or reverting for the curved world was an open
decision, and testing "restore the crest pause with the fixed detector" needs a code change that was not made. Raw data:
`curved_unloaded_origdet_logs/`, `curved_unloaded_repeat_logs/`, `curved_unloaded_fixedctl_logs/` (each
`*_result.log`, `*_launch.log`, `*_health.json`), `curved_unloaded_detector_comparison_final.txt`.

## State-scoped detector: causal test and final validation

**Decision implemented:** the gyro rotation check is switched off during `DESCENT`; the original scan-only stall logic
governs there. This is a **workaround for an unsolved problem, not a fix** (the curved-world descent edge departure has no
known cause). Final installed state (checksum-verified): `motion_stall_detector_node.py` = the scoped version
(`ea1bcee5...`, `data/motion_stall_detector_node.SCOPED.py`); `cmd_vel_watchdog_node.py` = the original
(`45428ae7...`, the temporary test hook was removed); the fixed and original detectors are kept as
`data/motion_stall_detector_node.FIXED.py` (`48625b76...`) and `.ORIGINAL.py` (`346608b6...`).

### 1. The boundary (`ROTATION_CHECK_EXCLUDED_STATES`, documented in the code)

| `/ramp_state` | Rotation check | Stall logic that governs |
|---|---|---|
| `APPROACH`, `ASCENT`, `TRANSITION`, `EXIT` | ACTIVE | scan verdict, overridden by the gyro check (as validated on the platform, every cell that is not a ramp descent, and the construction site) |
| `DESCENT` | OFF | ORIGINAL scan-only logic, unchanged |
| `HALT` | inherits the state it interrupted | a HALT that interrupted `DESCENT` -> original; interrupting `EXIT` -> gyro check |
| unknown (no message yet) | OFF | original logic (fail-safe) |

`test_state_scope.py` checks all 11 boundary cases (including HALT inheritance and the fail-safe). The node cannot tell a
curved world from the construction site, so the exclusion is by FSM state; that loses nothing validated because the
rotation check suppressed 14 false latches on the platform and none during the ramp descent.

### 2. Correcting the description of what differs (earlier wording was wrong)

Anchoring every run on its first `DESCENT` profile application (`nav2_param_switcher`), **all 32 runs analysed, including all 10
fixed-detector runs, have a wheel-slip HALT 0.3-0.65 s before `DESCENT`**; the earlier claim that the fixed runs had no
crest HALT used a window misaligned by ~1 s. What the original adds is a second stop **inside** `DESCENT`: a "Motion stall
confirmed" latch 0.4-0.8 s after `DESCENT` begins (6 of 6 original, 0 of 10 fixed), a 1.4-2.2 s pause, a slip HALT on
resume at +2.2-2.7 s, and `descent_brake_node`'s cautious restart (post-HALT re-entry pushes MPPI `ax_max`/`ax_min`/`az_max`
to 40% of the 2.0 m/s^2 / 3.0 rad/s^2 ceiling, +5% per 0.5 s). Stall latches *before* `DESCENT` are the same in every group
(mean 2.2 original, 2.3 scoped, 2.5 fixed).

### 3. Causal test (fixed detector, `curved / normal / unloaded`; each arm fired once at the first `DESCENT`, none a stall latch)

A stop cannot be delivered alone: any stop-and-go trips a wheel-slip HALT at resume (the slip metric divides by a near-zero
command), and every HALT also pushes MPPI parameters (each push resets its optimizer: verified as a burst of "Optimizer reset"
lines) and triggers the cautious restart. So the ingredients were tested separately (`crest_pause_injector.py`; the hold
uses a temporary watchdog hook that performs the same ramp-to-zero as a HALT without touching `/ramp_state`, since removed):

| Arm | What is injected | n | GT fails | mean max \|y\| (m) | gap to original closed | p vs fixed (Fisher / perm.) | p vs original |
|---|---|---|---|---|---|---|---|
| Original detector | (reference) | 6 | 0 | 0.90 | 100% | 0.0001 / 0.0001 | - |
| **Hold** | 1.75 s pause, 0.4 s after `DESCENT`, no stall latch | 6 | **2** | **1.59** | **57%** | **0.008 / 0.033** | 0.45 / 0.10 |
| Ceiling | cautious limits (40%, +5%/0.5 s), no pause | 5 | 5 | 2.19 | 19% | 1.0 / 0.34 | 0.002 |
| Reset | unchanged parameters re-pushed at +0.4 s and +2.15 s, no pause | 5 | 5 | 2.23 | 17% | 1.0 / 0.45 | 0.002 |
| Fixed detector | (baseline) | 10 | 10 | 2.49 | 0% | - | 0.0001 |

Hold vs fixed: mean difference -0.90 m, bootstrap 95% CI -1.62 to -0.13. Hold runs (max |y|): 1.02, 1.00, 0.93, 1.12 pass;
2.57, 2.88 fail. In every hold run the robot was already stationary when the hold began (speed 0.000 m/s) and the hold
produced wheel-slip HALTs at +0.5 s and +2.2 s, as feared (`states_during` shows HALT), so the hold *is* a HALT sequence.
Real-time factor 0.991-0.998, none starved.

**Result, stated plainly.** The **stop** is the only injected ingredient that moves the outcome; reduced acceleration limits
and MPPI optimizer resets alone do not (neither is distinguishable from the fixed detector). That argues against "the pause
helps because MPPI is reconfigured or re-anchored" and for a physical effect of stopping and restarting at the top of the
descent. It is only a **partial** confirmation: the hold closes 57% of the gap (2 of 6 still fail), n is 5-6 per arm, and the
stop cannot be separated from the slip-HALT/resume sequence it inevitably triggers. The result does not identify *why* a
stop helps, and it does not explain the underlying edge departure at all.

### 4. Final validation on the scoped detector (guarded clean host, one sim at a time)

`curved / normal / unloaded` x6: **4 pass, 2 fail.**

| # | GT max \|y\| (m) | Verdict | HALTs | Stall / slip / margin entries | In-`DESCENT` stall latch | Min RTF |
|---|---|---|---|---|---|---|
| 1 | 0.57 | PASS | 9 | 3 / 15 / 0 | yes | 0.994 |
| 2 | **2.54** | **FAIL** | 6 | 2 / 30 / 1 | **no** | 0.998 |
| 3 | 0.86 | PASS | 4 | 3 / 10 / 0 | yes | 0.999 |
| 4 | **2.10** | **FAIL** | 21 | 6 / 58 / 1 | yes | 0.996 |
| 5 | 0.38 | PASS | 7 | 4 / 13 / 0 | yes | 0.997 |
| 6 | 0.92 | PASS | 4 | 4 / 13 / 0 | yes | 0.998 |

Scoped: mean 1.23 m (sd 0.88), failure rate 2/6 (Wilson 10-70%). Versus the fixed detector (10/10): Fisher p = 0.008,
permutation p = 0.005, mean difference -1.26 m. **Versus the original (0/6): not distinguishable at these sizes** (Fisher
p = 0.46, permutation p = 0.44, mean difference +0.33 m, bootstrap 95% CI -0.30 to +1.02), so the workaround is shown to
recover most of the effect, **not shown to restore the original behaviour**; a larger sample could still reveal a residual
gap. Excluding run 1 (min RTF 0.994, just under the requested 0.995): 2 of 5 fail, mean 1.36 m. The in-`DESCENT` latch fired
in 5 of 6 scoped runs (original 6/6, fixed 0/10); across original + scoped + fixed (22 runs) it predicts the outcome strongly
(with the latch: 10 of 11 pass; without: 0 of 11 pass; Fisher p = 0.000034), consistent with the causal test but still an
association. **(This 6-run sample was extended by the closing sample in section 6; the pooled result supersedes these
6-run comparisons, in particular "not distinguishable at these sizes".)**

Full 7-cell matrix (one run each), against the earlier runs of the same cells:

| World / friction / payload | Prior trusted | Old detector | Fixed detector | **Scoped detector** |
|---|---|---|---|---|
| sharp / normal / unloaded | PASS (0) | MARGINAL (2; 0.10 m) | MARGINAL (3; 0.08 m) | **MARGINAL (6; 0.09 m)** |
| sharp / normal / loaded | FAIL: DID_NOT_CLIMB | FAIL (7) | FAIL (6) | **FAIL: DID_NOT_CLIMB (11)** |
| sharp / 0.45 / loaded | FAIL: DID_NOT_CLIMB | FAIL (8) | FAIL (9) | **FAIL: DID_NOT_CLIMB (7)** |
| curved / normal / unloaded | FAIL: edge dep. (4; 2.68 m) | FAIL: HALT_THRASH (11; GT PASS 1.01) | FAIL (14; GT FAIL 2.49 m) | **FAIL: HALT_THRASH (11); GT PASS 0.93 m** |
| curved / 0.45 / unloaded | MARGINAL (1; 0.36 m) | MARGINAL (3; 0.31 m) | MARGINAL (4; 0.23 m) | **FAIL: HALT_THRASH (13); GT PASS 0.14 m** |
| curved / normal / loaded | MARGINAL (4; 0.97 m) | MARGINAL (4; 0.94 m) | MARGINAL (3; 1.08 m) | **MARGINAL (5; 0.95 m)** |
| curved / 0.45 / loaded | FAIL: HALT_THRASH (9; 0.93 m) | MARGINAL (3; 0.93 m) | MARGINAL (2; 1.07 m) | **FAIL: HALT_THRASH (8); GT PASS 1.02 m** |

Ground truth passed in every cell that reached the descent (0.09-1.02 m), including `curved/normal/unloaded` this time.
**Nothing regressed on ground truth, but "nothing regressed" cannot be confirmed:** two cells' *overall* verdicts moved from
MARGINAL to FAIL purely on HALT count (`curved/0.45/unloaded` 13 HALTs; `curved/0.45/loaded` 8 HALTs). Their earlier runs
spanned 1-4 and 2-9 HALTs respectively, so this may be run-to-run variation, but each is a single run and the scoped
detector deliberately restores an in-`DESCENT` HALT, so an effect is not excluded. **Resolved in section 6 by repeating both
cells: `curved/0.45/loaded` is variation; `curved/0.45/unloaded` is probably a genuine added cost.**

Construction-site round trips (populated site):

| Payload | Detector | Leg 3 | Verified | HALTs (total / descent) | Min margin | Final leg-4 pos (gz) |
|---|---|---|---|---|---|---|
| loaded | old | ok | 102% | 24 / 12 | 0.463 | (2.00, +1.57) |
| loaded | fixed | ok | 103% | 14 / 4 | 0.463 | (2.02, +1.17) |
| loaded | **scoped** | **ok** | **103%** | **15 / 4** | 0.463 | (1.98, +1.42) |
| unloaded | old | ok | 102% | 40 / 26 | 0.872 | (1.96, -0.10) |
| unloaded | fixed | ok | 103% | 30 / 15 | 0.872 | (2.06, +0.00) |
| unloaded | **scoped** | **ok** | **102%** | **31 / 15** | 0.872 | (2.04, +0.01) |

Both complete with no abort, and the HALT counts match the fixed detector's, so the rotation-check gain on the construction
site is preserved. The extra loaded drift point (1.424 m) makes the clean-host loaded set n=12, mean 0.928 m, sd 0.442, max
1.572 m; the prop decision is unchanged. Minimum real-time factor 0.967-0.999 across the pass; none starved (three runs
under 0.995 but above the 0.95 guard: scoped repeat 1, and matrix cells `curved/normal/unloaded` 0.990 and
`sharp/0.45/loaded` 0.983).

### 5. What remains open (after the closing sample in section 6)

1. **The underlying curved-world descent edge departure is unresolved.** With the workaround about a third of
   `curved/normal/unloaded` runs (5 of 14) still leave the panel; the original detector was 0 of 6.
2. Whether the scoped detector is *equivalent* to the original on that cell: still not established, because the original's
   sample is only 6 (section 6). Settling it needs ~10-15 more original-detector runs (a detector swap).
3. Why a stop at the top of the descent helps (the causal test rules out MPPI reset and reduced accel limits as sufficient,
   and shows only partial reproduction by a bare hold), and why the in-`DESCENT` stall latch fires in only 10 of 14 scoped runs.
4. Where the extra HALTs in `curved/0.45/unloaded` come from (not traced).
5. Removal: see the final structural experiment below (`../path_structural/README.md`): the cause was found, and the workaround is unnecessary if the
   curved-cell procedure turns the robot on the flat platform. Removal awaits that procedure decision, then a 10+ run confirmation on the unscoped
   detector, the positive control, the matrix and both round trips.

### 6. Closing sample: larger scoped samples for the two loose ends (guarded clean host, one sim at a time)

Instrument, detector and code unchanged (scoped detector `ea1bcee5...`, watchdog original); guard verified 0 stale processes
and an idle host before every run; none starved. Per-run minimum real-time factor 0.991-0.999 (some runs dipped below the
requested 0.995, so every result below is also given with those runs excluded). Full output: `closing_samples_analysis.txt`,
produced by `analyze_closing_samples.py`.

**6a. `curved / normal / unloaded`, scoped detector, 6 earlier + 8 new = 14 runs.** New runs (max |y| m / verdict / HALTs /
min RTF): 2.01 F / 9 / 0.992; 1.93 F / 32 / 0.996; 0.98 P / 9 / 0.995; 1.00 P / 12 / 0.997; 1.15 P / 9 / 0.997; 2.36 F / 5 /
0.997; 0.59 P / 11 / 0.993; 1.07 P / 6 / 0.999 (P = ground-truth pass at the 1.5 m half-width; F = fail).

| Group | n | GT failures | Wilson 95% | max \|y\| mean (sd) | median |
|---|---|---|---|---|---|
| Original detector | 6 | 0 | 0-39% | 0.90 (0.26) | 0.98 |
| Fixed detector (rotation check everywhere) | 10 | 10 | 72-100% | 2.49 (0.63) | 2.41 |
| Scoped, first batch | 6 | 2 | 10-70% | 1.23 (0.88) | 0.89 |
| Scoped, new batch | 8 | 3 | 14-69% | 1.39 (0.63) | 1.11 |
| **Scoped, all** | **14** | **5 (36%)** | **16-61%** | **1.32 (0.72)** | **1.03** |
| Scoped, all runs with RTF >= 0.995 | 11 | 4 (36%) | 15-65% | 1.39 (0.71) | 1.07 |

| Comparison | Failure-rate difference (Newcombe 95% CI) | Fisher p | Mean max \|y\| difference (bootstrap 95% CI) | Rank-test p |
|---|---|---|---|---|
| Scoped (14) vs **fixed** (10/10) | -64 points (-84 to -27) | 0.0020 | -1.17 m (-1.69 to -0.67) | 0.0031 |
| Scoped (14) vs **original** (0/6) | +36 points (-8 to +61) | 0.26 | +0.42 m (+0.03 to +0.84) | 0.44 |
| Scoped, RTF >= 0.995 (11) vs fixed | -64 points (-85 to -24) | 0.0039 | -1.10 m (-1.66 to -0.56) | 0.0067 |
| Scoped, RTF >= 0.995 (11) vs original | +36 points (-8 to +65) | 0.24 | +0.49 m (+0.06 to +0.94) | 0.30 |

Reading it plainly. *Scoped vs fixed:* clearly better; that conclusion is robust. *Scoped vs original:* the data do **not**
show the scoped detector matches the original, and do **not** show it is meaningfully worse. The point estimate (36% vs 0%)
favours the original, the interval on the difference reaches +61 points at its upper end and only -8 at the lower, and the
scoped rate (36%) lies inside the original's own 6-run interval (0-39%). The mean-difference bootstrap barely excludes
zero while the rank test does not, which is the signature of an under-powered comparison. The limiting factor is the
original's sample of 6; more scoped runs cannot fix that. (0/10 original vs 5/14 scoped would give Fisher p = 0.053, 0/15
gives 0.017.) Best estimate: the workaround recovers most but not all of the original's behaviour. In-`DESCENT` stall latch,
30 runs across all three detectors: with the latch 15 of 16 pass, without it 0 of 14 pass (Fisher p = 2e-7); the latch fired
in 10 of 14 scoped runs, and the 4 scoped runs without it all failed. Post-hoc association only.

**6b. The two curved cells whose HALT-count verdict moved MARGINAL -> FAIL.** Every run shown (HALTs / overall / GT max
|y| m / min RTF); "matrix" = the run from the scoped 7-cell matrix.

`curved / 0.45 / unloaded` (9 scoped runs):

| Run | HALTs | Overall | GT max \|y\| | Min RTF |
|---|---|---|---|---|
| earlier: first matrix / old detector / fixed detector | 1 / 3 / 4 | MARGINAL x3 | 0.36 / 0.31 / 0.23 | - / 0.994 / 0.996 |
| scoped matrix | 13 | FAIL | 0.14 | 0.995 |
| scoped repeats 1-8 | 3, 7, 8, 7, 17, 9, 7, 10 | MARGINAL, MARGINAL, FAIL, MARGINAL, FAIL, FAIL, MARGINAL, FAIL | 0.35, 0.39, 0.30, 0.27, 0.05, 0.18, 0.41, 0.19 | 0.997, 0.991, 0.991, 0.998, 0.993, 0.995, 0.997, 0.998 |

Scoped HALTs: median 8, mean 9.0, range 3-17; `HALT_THRASH` (>= 8) in 5 of 9 (Wilson 27-81%); ground truth 0/9 fail (max |y|
0.05-0.41 m). Versus the three earlier runs (1, 3, 4): Mann-Whitney p = 0.032 (0.068 restricted to the six scoped runs with RTF
>= 0.995, of which 3 thrash); bootstrap 95% CI of the median difference +3 to +9 HALTs; Fisher on the thrash fraction (5/9 vs 0/3)
p = 0.20. Stall-latch HALT entries are 3-5 per run in every group (no change); the rise is in total HALTs.

`curved / 0.45 / loaded` (5 scoped runs): HALTs 8 (matrix), 1, 1, 2, 7 (median 2, mean 3.8), `HALT_THRASH` in 1 of 5 (Wilson
4-62%); ground truth 0.94-1.08 m, 0/5 fail; min RTF 0.997-0.999. Earlier: 9 (first matrix, before any detector work, itself a
`FAIL: HALT_THRASH`), 3 (old), 2 (fixed). Scoped vs those: Mann-Whitney p = 0.45.

**Verdict.** `curved/0.45/loaded`: **run-to-run variation**; the 8-HALT run was the tail of a distribution that produced 9 HALTs
before any of this work. `curved/0.45/unloaded`: **probably a genuine added cost of the workaround**, not just noise: 8 of 9
scoped runs had >= 7 HALTs against 1-4 in the three earlier runs. It is not proven to be caused by the scoped detector rather
than by day-to-day variation of the cell (the comparison arms are single runs, and two of them are other detectors), and the
source of the extra HALTs was not traced. It is physically harmless (robot stays on the panel, 0/9 ground-truth fails) but
costs extra stop-and-go, and the cell will read `FAIL: HALT_THRASH` on the >= 8 rule about half the time.

### 7. Final structural experiment (supersedes items 1-3 and 5 above where they say the cause is unknown)

Full report: `../path_structural/README.md` (pre-registration with four dated amendments, all arms, CIs). Summary: path densification
(0.4 -> 0.05 m, n=12) and an explicit cross-track critic (n=12) produce 0/12 edge departures each only because the robot never descends
(0/12 genuine descents); the departure is caused by the harness turning the robot on the curved crest, which leaves it facing the panel edge (open-loop
turn achieves ~90 of 180 degrees) and displaced 0.2-1.3 m sideways while the EKF's y stays 0.00; a closed-loop turn on the flat platform gives 0/6
departures and 6/6 full descents on the scoped detector and 0/6 and 6/6 on the unscoped detector. The "second mechanism" seam-stumble explanation and the
"structural limitation of MPPI" reading are refuted for this cell. Also: at normal friction every recorded curved-cell run moved only 0.1-2 m of the 6 m path,
so the earlier ground-truth PASSes meant "did not leave the panel before the leg ended".

### 8. Round 2: corrected turn procedure adopted everywhere, scoping removed, final matrix (supersedes sections 4-7 wherever they conflict)

Full report: `../path_structural/README.md` section 7 (pre-registration with six dated amendments in `PREREGISTRATION_ROUND2.md`). Summary: the corrected procedure (ground-truth-measured 2.4 m drive onto the flat platform, closed-loop turn on the
EKF yaw) is the default for all worlds (the sharp-world open-loop turn also fell short: 62-67 of 180 degrees, robot never descended). Larger samples on curved / normal / unloaded: scoped 0/18 and unscoped 0/18 failures (Wilson 0-18%), all 36
reaching the ramp foot (max |y| <= 0.056 m). Scoping decision: REMOVED (unscoped fixed detector 48625b76 installed; workaround kept as `.SCOPED.py`). Positive control on it: forced freezes 18/18, false latches while rotating 0, leg 3 6/6 (26.8% stall-latched vs
27.4% originally). Both round trips complete (loaded 103%, 22 / 9 HALTs; unloaded 102%, 36 / 19). The corrected 7-cell matrix (two runs, old-vs-new verdicts for every cell) is in the path_structural README section 7.4: curved ground truth is clean whenever the robot reaches
the platform; loaded curved cells often cannot clear the crest (curved / normal / loaded 6 of 8, curved / 0.45 / loaded 0 of 7) - a capability limit the old harness hid. Items 1-5 of section 5 are closed except: single-run HALT verdicts are noisy, and the cause of the crest-turn displacement is optional future work.

### 9. Crest freeze root-caused and fixed; ascent gate added (2026-09-20; supersedes the "loaded curved cells cannot clear the crest" limit in section 8)

Full report: `../crest_freeze/README.md`. The loaded-robot freeze at the ramp crest (curved / 0.45 / loaded 0/10 reached the platform, curved / normal / loaded 6/8, and the sharp-world loaded cells' `DID_NOT_CLIMB` in every earlier matrix) was a 0.05 m geometry authoring
step at the crest -> platform join (and the mirror -0.05 m at the ramp foot) in every `ramp_world_*.sdf` - the same bug class as the construction-site freeze, never fixed in these worlds. Evidence: exact corner math (all other seams <= 1.4 mm), two wheel-level recordings (identical frozen pose, front wheel within 3 mm of the
step corner, wheels spinning at 0.09 m/s at 0.0003 m/s chassis speed, `/wheel_slip` ~0.01, EKF +4.5 m ahead), and the old sharp-world freeze pose (gz x 7.119, within 1.1 mm of the same contact). Ramp slabs raised 0.050 / 0.052 m in all 7 ramp worlds. Reverification: loaded ascent + descent 48/48 (Wilson 93-100%), unloaded regression 6/6 x 2;
final 7-cell matrix: all cells ascend and descend fully (1 PASS, 5 MARGINAL, 1 FAIL on HALT count). Ascent gate: neither driver checked the ascent physically; the descent's `/verified_progress_m` pattern would not have caught this freeze (96% of raw wheel distance accrued while the chassis was frozen), so both drivers now compare Gazebo ground-truth
displacement with the odometry displacement (0.5 m tolerance, calibrated on 24 healthy leg ends <= 0.09 m). Both round trips complete with all ascent legs physically verified (disagreement <= 0.035 m; loaded 13 / 3 HALTs, unloaded 31 / 15).
