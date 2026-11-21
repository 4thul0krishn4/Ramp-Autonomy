# Validation methodology

This project's validation approach was not fixed from the start - it changed shape after being fooled more than once by signals that looked
healthy while the robot was not. The current methodology is the result of that history, not a plan written up front. The specific incidents are
in `retrospective.md`; this document is the general method on its own terms.

## Why wheel odometry and FSM state are not enough

A `FollowPath` goal can report success, and a state machine can report a clean state transition, while the robot's real position is something
else entirely. This project hit that failure mode twice, in two different forms: a controller that trusted wheel odometry's yaw during a turn
and walked the robot off a physical ledge while the odometry-frame position still looked plausible, and a chassis genuinely wedged against a
0.05 m geometry step with its wheels still spinning at commanded rate, while the fused position estimate kept integrating that wheel rotation
and reported a complete round trip that never actually happened past the freeze point.

Neither failure was a bug in the signal being read - `/odom` and `/ramp_state` were both doing exactly what they're built to do. The bug was in
trusting them as a complete picture of the robot's physical state. The fix in both cases was the same shape: add an independent signal that
cannot be fooled by the same mechanism, and gate the test's own success criterion on it.

## Ground truth as the independent signal

Every test that involves turning, lateral motion, or a `FollowPath` goal-completion check now polls Gazebo's own ground-truth model pose
directly (`gz topic -e -t /world/<world>/dynamic_pose/info`), bypassing `/odom` and the EKF entirely. This is simulation-only - it has no
hardware equivalent, which is itself a documented limitation (see `known-limitations.md`) - but within simulation it is the one signal that
cannot be corrupted by the same wheel-slip or sensor-fusion mechanisms that can corrupt everything built on top of wheel rotation.

Two independent ground-truth checks exist, catching different failure shapes, not one superseding the other:

- **Lateral departure**: has the robot's real Y position exceeded the ramp or platform panel's own half-width. Catches a robot that drifted off
  the edge while believing it was centered.
- **Verified progress**: `/verified_progress_m`, a wheel-rotation distance signal that only counts while a LiDAR scan-consistency check has not
  latched a stall. Catches a robot that is wedged in place while its wheels keep turning and the fused position keeps advancing anyway.

A robot can fail one of these and pass the other - a lateral check does not catch a stall that stays within bounds, and a progress check does
not catch a departure that still covers real distance. Both run on tests where either failure mode is plausible.

## The positive control: proving a detector can actually detect

Adding a safety detector is not enough on its own - a detector that never fires because its gate is too strict is indistinguishable, from the
outside, from a detector protecting against a failure mode that never happens. Before trusting any new HALT trigger, this project tests it
against a scenario constructed specifically to make the failure happen, scored against ground truth, not against the detector's own output.

The motion stall detector's positive control is a direct example: a low invisible cage spawned around the moving chassis, forcing it to stop
while its wheels kept turning, the exact signature the detector is meant to catch. Run against ground truth: forced freezes correctly detected
18 of 18 times, with zero false latches while the robot was genuinely just turning (a case that used to produce false stalls before a gyro
rotation check was added). Only after that result does a detector's silence during a normal run mean "nothing was wrong" rather than "the
detector probably doesn't work."

## The clean-host guard

A test that looks like it failed because of real physics can instead be an artifact of a loaded test host - a leaked background process
pushing real-time factor below 1.0, or load average high enough that the simulator itself behaves differently. This project hit exactly that:
an apparent "50% stall rate" turned out to be a trial harness leaking roughly 57 stale ROS/Gazebo processes across repeated launches, not a
property of the robot or world being tested, and did not reproduce on a clean host.

`validation/phase8/harness/harness_guard.py` is the standard answer: before any guarded run, clean up every leftover sim/ROS process by PID
(never a broad pattern match that could catch the test's own shell), confirm the host is otherwise idle, and check real-time factor is close to
1.0 before accepting the launch as valid. A health monitor samples real-time factor and load throughout the run and flags the result as
"starved" if the host degraded mid-run. Runs flagged starved are discarded and rerun, not kept with a caveat - a test run under contention does
not tell you about the system under test.

## Classify before you cite

Once the ground-truth gap was understood, this project went back through its own prior results and classified each one rather than assume the
newer methodology silently applies everywhere: unaffected (no turning, no `FollowPath` goal-completion ambiguity - safe to trust as reported),
affected but already reverified (could have been fooled, but a later ground-truth-checked rerun stands in its place), or genuinely unverified
(would need a real rerun before citing). `results.md`'s numbers are drawn from the reruns and the unaffected category; anything flagged
unverified in that audit is noted as such rather than quietly dropped or presented as equivalent.

## Statistical honesty on small samples

Several of this project's findings rest on samples too small for a strong statistical claim - a workaround evaluated against 6 original-method
runs, a drift distribution characterized from 8 trials. The convention used throughout is to report the actual interval (Wilson confidence
intervals on pass/fail rates, bootstrap intervals on continuous measurements like lateral drift) rather than a bare point estimate, and to say
plainly when two conditions are "not distinguishable" because the sample is too small to tell, rather than present that as "equivalent." Where
a comparison matters enough to resolve, the fix was to collect more data, not to round the uncertainty away.
