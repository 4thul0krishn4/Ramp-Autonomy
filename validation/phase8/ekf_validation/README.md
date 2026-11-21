# EKF fusion validation (robot_localization)

Fix for the root cause identified in `validation/phase8/construction_site_descent/`:
`FollowPath`/MPPI closes its control loop against wheel odometry, and wheel
odometry's YAW diverges from ground truth under any turning motion (not
just fast open-loop spins - reproduced on flat ground, a gentle closed-loop
arc, zero fillet seams). Fixed with `robot_localization`'s `ekf_node`
(`ramp_autonomy_nav/config/ekf.yaml`), fusing wheel odometry's position +
linear velocity with the IMU's orientation + angular velocity into
`/odometry/filtered`, which now feeds both `controller_server`'s velocity
subscription and (via the `odom -> base_footprint` TF, which `ekf_node` now
owns instead of gz-sim's DiffDrive plugin) the robot's pose as Nav2 sees it.

## Isolated validation (this directory) - BEFORE trusting the real round trip

Per the task's own instruction not to assume the EKF works just because
it's running: this test drives straight, does the exact same open-loop
180 deg spin (0.5 rad/s, 6.6 s) that originally exposed the problem, then
drives straight again - all on flat, open ground (construction_site.sdf's
access road, nowhere near the ramp or any wall) - comparing
`/odometry/filtered` directly against Gazebo ground truth at every sample.

### First attempt caught a real bug in the EKF config itself

`ekf_validation_run1_position_bug.csv/log`: initial `ekf.yaml` fused wheel
odometry's ABSOLUTE x/y POSITION as well as its velocity. Yaw tracked
ground truth almost exactly (as intended), but position error still grew
to 11.3 m during the post-spin straight leg. Root cause: wheel odometry's
own x/y position is itself integrated using wheel odometry's OWN internal
yaw estimate - the exact quantity being fused specifically because it's
unreliable. Fusing that position directly re-imports the same corruption
one step removed, even though the fused ORIENTATION was already accurate.
**Fixed** by changing `odom0_config` to fuse only wheel odometry's
body-frame linear velocity (vx, vy), not its position - letting the EKF
integrate its own position using its own fused (IMU-corrected) yaw. This
is the standard, textbook-correct robot_localization pattern for this
failure mode.

### Clean result, after the fix (`ekf_validation_clean_run.csv/log`)

A second attempt at the corrected config also caught a real, but
unrelated, test-harness problem (not a second EKF bug): driving to a
fixed X target after an open-loop spin whose actual achieved angle varies
run to run (135-180 deg, never precise - the whole reason the EKF exists)
meant the robot could spend much longer than expected covering the
diagonal distance still needed to reach that X, drifting far enough
sideways to physically hit the office zone's wall (Y=8) - wheels then
kept "reporting" motion into the wall, which no pure dead-reckoning
approach can be expected to see coming without an obstacle sensor. Fixed
in the test itself by driving for a fixed, short TIME instead of to a
position target, keeping the whole test inside open space regardless of
the actual spin outcome.

With both fixed, the clean run:

| phase | n | pos_err_m (min/max/mean) | yaw_err_deg (min/max/mean) |
|---|---|---|---|
| straight1 (pre-turn) | 22 | 1.000 / 1.058 / 1.044 | 0.00 / 0.00 / 0.00 |
| spin (0.5 rad/s, 6.6s) | 14 | 0.999 / 1.001 / 1.000 | 0.00 / 3.25 / 1.97 |
| straight2 (post-turn) | 13 | 0.964 / 0.998 / 0.972 | 0.00 / 0.16 / 0.01 |

The ~1.0 m `pos_err_m` baseline present in ALL three phases (including
before any motion at all) is not error - it's the constant, expected
difference between Gazebo's world frame (robot spawns at world x=1) and
the `odom` frame's own convention (initialized at 0,0 at the robot's own
start pose). What matters is that it does NOT grow: contrast this with
the original raw-wheel-odom finding, where the SAME 180 deg spin produced
a true heading error of 35-40+ deg that then compounded into meters of
lateral drift over subsequent straight-line travel. Here, yaw error peaks
at 3.25 deg during the spin itself and settles under 0.2 deg immediately
after - and critically, position error does not grow with distance
traveled post-turn (0.97-1.0 m at the end of straight2, same range as
before the robot ever moved).

**Conclusion: the EKF fusion is genuinely accurate for the specific
problem it was built to fix (yaw estimation under turning), not just
"running and plausible-looking."**
