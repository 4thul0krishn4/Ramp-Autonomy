# Recorded round trips (2026-09-20) - real simulation, full current stack, Gazebo GUI live

> **Where the files are.** The videos below are attached to the
> [latest GitHub release](https://github.com/skunal3318/Ramp-Autonomy/releases/latest) rather than committed to the repository, and the
> per-run `data/` folders (rosbags, ground-truth CSV, logs) are not tracked at all - they are several hundred MB per run and are regenerated
> by `visualization/record_round_trip.sh`. The paths in the table below are where each file lands once you download or re-record it.

Each recording is a REAL run of the complete stack (EKF, stall detector, goal-verification gate, ascent physical gate, dual LiDAR, descent braking, arc-turn FollowPath, unscoped fixed detector 48625b76, corrected geometry)
on `construction_site.sdf`, one full round trip: gate -> laydown -> up the ramp -> arc turn on the elevated level -> down the ramp -> back to the gate. The video is a screen capture of the Gazebo GUI window (the camera follows the robot).

| | video | data | result |
|---|---|---|---|
| **Loaded** (100 kg, +0.1 m CG) | `round_trip_loaded/round_trip_loaded.mp4` (6.6 min, 1280x720, 20 fps, 103 MB) | `round_trip_loaded/data/` | all 4 legs ok, no abort; leg-3 verification 107%; 21 HALT events (7 in descent); min stability margin 0.46; ends at (2.03, +0.85) |
| **Unloaded** | `round_trip_unloaded/round_trip_unloaded.mp4` (7.3 min, 111 MB) | `round_trip_unloaded/data/` | all 4 legs ok, no abort; leg-3 verification 106%; 40 HALT events (22 in descent); min margin 0.91; ends at (1.98, -0.10) |

## What is in each `data/` folder
`roundtrip.log` / `roundtrip.csv` (the test driver's own log and 2 Hz sampled state incl. ground-truth pose), `ground_truth_pose.csv` (Gazebo pose, 20 Hz: x, y, z, roll, pitch, yaw), `rosbag/` (ROS 2 bag, zstd-compressed mcap, ~68 MiB, ~2.5 M messages:
`/odometry/filtered /odom /imu /joint_states /wheel_slip /stability_margin /motion_stalled /verified_progress_m /raw_wheel_distance_m /ramp_state /ramp_perception /scan /scan_360 /cmd_vel_nav /model/ramp_robot/cmd_vel /clock /tf`;
replay with `ros2 bag play round_trip_loaded/data/rosbag`), `events.json` (leg start/finish times as offsets into the video, e.g. loaded: leg 1 at 0:11, leg 2 at 1:39, leg 3 (arc turn + descent) at 3:02, leg 4 at 5:21),
`health.json` + `verify.txt` (guard measurements), `launch.log` (full stack log), `gst_record.log`, `rosbag_record.log`.

## Honest notes
* **These are demonstration recordings, not validation data.** Screen-encoding + bag + GUI add load: real-time factor over each run averaged 0.94 (min 0.915 loaded / 0.914 unloaded), guard flag `starved: false`, but below the 0.995 bar used for the validation matrix. The validated numbers are in the validation READMEs.
* A first loaded take (`take1_starved_loaded/`, video + logs, bag removed to save disk) ran under a heavier recorder and was flagged STARVED (min RTF 0.875, mean 0.92); it also completed all four legs. It is kept only as a record of why the recorder was lightened; use the newer take.
* Video timing: the video starts ~11 s before the test driver's leg 1 (guard check + recorder start-up); `events.json` already accounts for that.
* Playback: standard H.264 mp4 (plays in VLC, browsers, mpv). The Gazebo window chrome (entity tree, RTF readout) is part of the capture; the readout in the corner shows the real-time factor.
* Tooling: `visualization/record_round_trip.sh <loaded|unloaded>` re-records; `visualization/run_live_gui.sh` runs it live without recording. The disk is ~97% full (2.3 GB free) - each new recording needs ~250 MB.
