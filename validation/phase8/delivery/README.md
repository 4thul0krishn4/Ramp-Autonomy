# Detach mechanism decision and RTF dry-run (2026-09-26 / 2026-09-28)

Prerequisites for the attach/detach plugin session (`fsm_multiramp` 7.5 / `halt_loop` flagged both as open before writing real mission code). Resolved in two passes: 2026-09-26 built the
opt-in detachable payload and found the default detach geometry left the payload on the chassis; 2026-09-28 (AC power) fixed that and re-measured RTF clean, to convergence.

## 1. Detach mechanism - what "payload rests on chassis" meant and the fix

The URDF's `payload_link` sits directly above `base_footprint` (the payload joint puts its centre at +0.395 m). A `gz-sim-detachable-joint-system` fixed joint releasing there just stops constraining a body
that is already sitting on top of the robot - nothing pushes it off, so `detach_smoke.py`'s first test showed the payload staying at a constant offset from the chassis through a drive, attached or not.

**Decision: a delivery dock, not a drop or a recessed pad.** A pad recessed enough for the payload to fall into would also be a step the wheels would have to cross (contradicts the "no step for the wheels"
design already committed for `floor2_delivery_zone`); a pure vertical drop needs the payload's landing footprint to already be clear of the chassis, which it is not. A **dock beside the pad, at deck height**
mirrors how a real AMR would deliver to a fixed table (a roller-conveyor or a slide-off transfer) - the payload moves sideways off the robot onto a receiving surface, not down.

* **Geometry** (`floor2_delivery_dock`, `validation/phase8/floor2/generate_floor2.py`): 1.6 x 1.0 m, top at 0.365 m above Floor 2 - 5 mm below the payload's measured resting underside (settled robot origin
  0.125 m + payload joint 0.395 m payload centre − 0.15 m half payload height = 0.370 m; not assumed, read from `detach_smoke.py`'s settled pose). Centred at y = 0.94 (dock spans y 0.44-1.44), 0.14 m clear
  of the chassis side (0.30 m half-width + 0.05 m wheel clearance) so the payload is over the dock before its centre passes the chassis edge. Placement re-checked with the project's own layout tooling
  (`check_floor2_layout.py`, extended with a docking exception: the dock is allowed inside the normal 1.5 m lane buffer because it is *part of* the delivery point, same treatment the marker post already had -
  everything else on Floor 2 still holds the 2.0 m rule).
* **Transfer** (`payload_unloader.py`): a small script using gz-sim's `ApplyLinkWrench` **world** system (an environment actuator, run alongside the sim - not a robot node, not part of the autonomy stack under
  test) watches `/payload/state`; on `detached` it applies a persistent sideways force to the payload link, releases it once the payload has moved past the dock edge. Payload friction lowered to mu 0.3 (from
  the default's implicit 1.0) so it can be pushed at all; the dock surface is also mu 0.3 so it comes to rest rather than sliding indefinitely. Tuned by direct trial against the recorded trajectory
  (`m1.json` .. `m9.json`, `*.unloader.log.traj.json`): too little force leaves it on the deck edge, too much overshoots past the dock into open air - the final constant-force setting lands within the dock
  footprint with margin in every repetition (below).
* **Verified, not just the mechanism working once:** every one of the 7 clean RTF-dry-run repetitions of the detached states independently exercised the full release -> push -> rest sequence.
  Landing pose in all 7: **z = 7.065 exactly** (dock height) in every run, **y = 0.79-0.93** (dock spans 0.44-1.44, so 0.09-0.14 m margin from either dock edge and never overlapping the chassis at y = 0), and
  it stayed there through the `mechanism_test.py` drive-away check (robot reverses 1.5 m then drives forward 3 m; payload pose unchanged to the mm).

## 2. RTF baseline - real-time, repeated to convergence, contamination checked

`rtf_state_run.py`: whole stack up, robot spawned already on the Floor 2 delivery pad, real-time (`real_time_factor` 1.0, the shipped physics setting - not the max-speed mode used for the first, partial pass),
`/clock` vs wall time in 5 s windows over a 100 s measurement period (after a settle, and after release + unload for the detached states). Every run records its own host state - `/sys/class/power_supply/ACAD/online`,
`powerprofilesctl get`, the `energy_performance_preference` sysfs value, and per-CPU MHz - at start, every 10 s during, and at end; `rtf_states_chain.sh`/`rtf_states_topup.sh` also block before *launching* any run
until all three read correctly (not just once "on AC" is assumed for the whole session - the 2026-09-26 pass was contaminated by the laptop going onto battery mid-chain without that check). `analyze_rtf_states.py`
additionally cross-checks each run's own guard `_health.json`: two `detached_idle` and two `detached_away` repetitions in the first 6-rep chain were flagged **STARVED by the harness guard itself** (a real host
contention event mid-run, not a power-state issue - confirmed the host's AC/profile/EPP read correctly throughout those same runs) and are excluded even though the naive host check alone would have passed
them. Discarding them left only 4 clean reps for those two states, below this project's 5-run minimum, so 3 more repetitions were run once the laptop was back on AC (`rtf_states_topup.sh`) - all 3 clean, bringing
both to 7 valid reps each, converged (run-to-run std 0.002-0.003).

| state | valid reps (of attempted) | mean of run-means | min / max of run-means | std | min / max of any 5 s window | avg CPU (MHz) |
|---|---|---|---|---|---|---|
| unloaded (reference) | 5 / 6 | 0.9647 | 0.9577 / 0.9682 | 0.0041 | 0.821 / 0.980 | 3604 |
| attached (today's validated config) | 6 / 6 | 0.9587 | 0.9269 / 0.9674 | 0.0159 | 0.809 / 0.986 | 3599 |
| attached, plugin declared + unused (no-op) | 6 / 6 | 0.9552 | 0.9380 / 0.9638 | 0.0115 | 0.816 / 0.976 | 3590 |
| **detached, resting on the dock** | 7 / 9 | **0.9701** | 0.9676 / 0.9730 | 0.0021 | 0.951 / 0.984 | 3571 |
| **detached, robot driven away** | 7 / 9 | **0.9544** | 0.9518 / 0.9597 | 0.0026 | 0.935 / 0.970 | 3594 |

(2 discarded reps of `detached_idle`/`detached_away` each were the harness-guard-starved ones above; `unloaded_r6` failed a launch-readiness check once and was not retried since 5 reps already met the minimum.)

## 3. Verdict against the existing ~0.93 baseline

**Affordable - no lighter collision/physics setup is needed.** `attached` (0.959) is itself slightly above the project's long-standing whole-stack mean of ~0.93 on this world (host has been faster this
session; the guard's RTF gate is relative, not this exact number). Both detached states sit in the same band as `attached` and `unloaded` - **0.970 resting on the dock, 0.954 driven away** - neither is a cost
that eats into the ~7 % real-time margin the earlier (uncorrected-mechanism, max-speed) dry-run flagged as a risk. That earlier concern (-7 % to -15 % at max simulated speed) does not reproduce in real-time
mode with the corrected mechanism; the difference is examined below, not hand-waved.

**Why the two measurements disagree, honestly:** the first pass used the physics engine's *max-speed* mode (`real_time_factor: 100`, meant to expose raw per-step cost) with the payload landing ON the
chassis - continuous stacked-body contact solving at every step, at whatever rate the CPU can push steps through. The corrected mechanism's *real-time* numbers here settle the payload once and then have no
sustained new contact to solve. A same-conditions re-check (max-speed, corrected mechanism, resting on the dock) was not run this session - the real-time numbers are what the guarded trial harness and every
other RTF figure in this project are reported in, so they are the ones that matter for whether a guarded trial will run cleanly, and on that standard the detached states are clearly affordable.

## Files

`payload_unloader.py` (the sim-side transfer actuator), `mechanism_test.py` / `run4_mechanism/` (mechanism iteration, m1-m9), `detach_smoke.py` (original smoke test that found the "rests on chassis" problem),
`rtf_state_run.py` / `rtf_states_chain.sh` / `rtf_states_topup.sh` / `analyze_rtf_states.py` / `run5_rtf_states/` (the RTF baseline), `rtf_dryrun.sh` / `rtf_dryrun2.sh` / `run2_rtf_dryrun*.txt` /
`run3_rtf_dryrun_window.txt` (the superseded first-pass max-speed dry-run, kept for the record - the last 4 lines of `run3_rtf_dryrun_window.txt` are marked invalid, on-battery). World/description changes:
`floor2_delivery_dock` in `construction_site.sdf` (via `generate_floor2.py`), `payload_box.sdf.xacro`, `detachable_payload` xacro/launch arg (`robot.urdf.xacro`, `ramp_world.launch.py`,
`ramp_autonomy_bringup.launch.py`, all default off - the default URDF is verified byte-identical to before), `spawn_x`/`spawn_y`/`spawn_z` launch args (needed to spawn the test robot directly on Floor 2).

## Not yet done (deferred at the user's request, 2026-09-28)

The real mission - `payload_delivery` trigger node, ROS-side bridge of `/payload/detach`/`/payload/state`, `stability_monitor` payload-mass update on detach, and the full loaded
spawn -> ramp 1 -> ramp 2 -> dock -> detach -> empty return run with ground-truth verification - was explicitly deferred to a later session once this RTF check cleared the plugin as affordable.

## 4. The real mission (2026-09-30)

Once the mechanism and the RTF baseline both cleared (sections 1-3), the mission itself: a `payload_delivery_node.py` ROS node (new, `ramp_autonomy_control`) watches the
robot's own EKF pose and, with the payload still attached, stopped inside the Floor 2 delivery pad zone (odom x 77.3-79.7, y +-1.2) for 1 s, publishes `/payload/detach`
(bridged to gz's own DetachableJoint topic - see `ramp_world.launch.py`) - no test-script command triggers it, it is a real autonomy-stack event, launched conditionally
whenever `detachable_payload:=true`. Once gz confirms `detached` on `/payload/state`, the node also zeroes `stability_monitor_node.py`'s `payload_mass_kg` parameter (a
`set_parameters` call, the same pattern `nav2_param_switcher_node.py` already uses) so the tip-over margin reflects the robot's now-real unloaded mass; the physics itself
is already correct the instant the joint releases (the payload's own mass leaves with it), this only fixes the ROS-side parameter. `/payload/detach` and `/payload/state`
are bridged unconditionally in `ramp_world.launch.py` (harmless when the payload/plugin don't exist).

`full_mission_test.py` (imports `Probe` from `validation/phase8/fsm_multiramp/multiramp_nav2_test.py` unchanged, so every leg gets the identical Nav2/MPPI FollowPath
driving and ground-truth-vs-odometry gate as the rest of this project): loaded spawn at the gate (`detachable_payload:=true`, the separate payload model, not the lumped
one) -> ramp 1 up -> Floor 1 -> ramp 2 up -> Floor 2 -> the delivery pad -> stop and wait (the payload_delivery_node's own trigger, not a script command) -> confirm the
payload landed on the dock (ground truth) -> closed-loop 180 deg turn -> ramp 2 down -> Floor 1 -> ramp 1 down -> gate, empty. `payload_unloader.py` (section 1) runs
alongside as the environment actuator.

**3 of 3 clean guarded runs, all pass** (1 additional attempt was discarded by the harness guard as starved and retried, per this project's standard protocol - not counted):

| run | all 6 legs (GT-vs-odometry, gate 0.5 m / 4 % of the leg) | delivery status | payload landing (world y, dock spans 0.44-1.44) | HALTs | mean RTF |
|---|---|---|---|---|---|
| 1 | 6/6, disagreement -0.049..-0.000 m | WAITING -> DWELLING -> RELEASED | y = 0.717, z = 7.065 (exact dock height) | 10 | 0.905 |
| 2 | 6/6, disagreement -0.047..-0.000 m | WAITING -> DWELLING -> RELEASED | y = 0.798, z = 7.065 | 9 | 0.902 |
| 3 | 6/6, disagreement -0.047..-0.001 m | WAITING -> DWELLING -> RELEASED | y = 0.883, z = 7.065 | 9 | 0.900 |

Every leg's ground-truth-vs-odometry disagreement is inside the leg's own gate (worst case -0.049 m on a leg with 27 m of intended travel, 0.18 %); the delivery-pad leg
(2.9 m, precision approach) disagrees by <= 0.001 m in all three. The payload lands within the dock footprint every time, never overlapping the chassis (world y = 0 at
the pad centreline; the dock's near edge is at y = 0.44). HALT counts (9-10) and mean RTF (0.90) match the range already established for the loaded full-trip
(`fsm_multiramp` 7.3: HALTs 8-12, RTF ~0.92) - carrying an active payload-release/mass-zero event through the loop changed neither.
