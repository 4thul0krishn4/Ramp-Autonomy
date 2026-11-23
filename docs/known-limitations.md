# Known limitations

Current, not historical - the story behind each fixed item is in `retrospective.md`. This list reflects the project's own "closed for this
round" status as of 2026-09-20, plus later items opened since.

## Open investigations

- **Why turning in place on the curved world's crest geometry displaces the chassis 0.2-1.3 m sideways** is documented and understood well
  enough to route around (turn on flat ground instead - see `retrospective.md`'s turn-harness section), but the mechanism itself was never
  investigated further. Whether it still matters now that the crest geometry step is fixed was not re-tested.
- **No onboard ascent verification.** The ascent physical-reality gate (comparing Gazebo ground-truth displacement against odometry) exists only
  in the test harness. The robot itself has no equivalent check - it would need an exteroceptive floor-motion sensor this project does not
  model.
- **The HALT/recovery push loop.** Every state change while the robot is moving pushes a new MPPI parameter profile and resets the optimizer.
  Under load, a HALT recovery can trigger a state change that re-triggers the condition that caused the HALT, observed once as a 75-HALT thrash
  (84 `halt_mode`/`flat_mode` push pairs in one run). Understood, not fixed.
- **Real-time factor headroom is thin on the full autonomous stack**, roughly 7% (mean 0.93 whole-stack; a heavy measurement probe alone costs
  about 0.05 of it). The clean-host guard's low-real-time-factor threshold is configurable (`HG_LOW_RTF`) specifically because this margin is
  thinner than the guard's original calibration assumed.
- **The clean-host guard refuses guarded trials on commodity hardware, and the cost is this project's own nodes rather than the world.**
  `harness_guard.py`'s `MIN_RTF` is a hard gate of 0.95, checked before every guarded trial. Measured on a 12-core laptop with an RTX 3050, on
  mains power: roughly **0.91-0.93 headless** and roughly **0.70 with the Gazebo GUI running**, so a guarded trial aborts immediately with
  `HARNESS GUARD FAILED - DO NOT RUN THE TRIAL`. The ablation, server only unless stated:
  - world alone, no robot and no sensors: **0.99-1.00**
  - world plus the robot plus both LiDARs plus the bridge, no autonomy nodes: **0.998-1.000**
  - the full stack: **0.91-0.93**

  So neither the construction-site world nor the sensors cost anything measurable. The shortfall is entirely the nodes, and they are dominated by
  one unthrottled publisher. `gz-sim-joint-state-publisher-system` in `robot.urdf.xacro` carries no rate limit, so `/joint_states` publishes every
  1 ms physics step, measured at **936 Hz**. `stability_monitor_node.py` recomputes wheel slip on each of those callbacks and republishes
  `/wheel_slip` at **987 Hz**, which in turn wakes `state_machine_node.py` and `descent_brake_node.py`. Those four nodes plus the stall detector
  come to about 183% CPU between them, more than the simulator itself, against 8.5% for Nav2's `controller_server`. By comparison `/imu` runs at
  95.6 Hz and both LiDARs at about 9 Hz, exactly as declared.

  Note that `architecture.md` describes the HALT triggers as checked "every IMU callback (~20 Hz)". That holds for the margin path. The slip path
  runs roughly fifty times faster than that.

  Two things that do **not** fix it, both tried: `HG_LOW_RTF`, which relaxes only the health monitor's in-run clause while `MIN_RTF` is a separate
  module constant with no environment override; and adding `<update_rate>` to the joint state publisher plugin, which gz-sim appears to ignore
  (`/joint_states` stayed at 758 Hz and the real-time factor did not move). A real fix has to be subscriber side, and it cannot be a smaller number
  chosen by eye: `SLIP_HALT_PERSIST_S = 0.25` was calibrated against artefacts observed at the current rate, so changing how often slip is sampled
  has to be re-validated against `halt_loop`'s own measurements.

  **Power state matters more than any of this.** The same machine on battery, with the governor at `powersave` and the cores held near 1 GHz against
  a 4.4 GHz maximum, runs the full stack at **0.48-0.51**. If you report a real-time factor, say whether the machine was plugged in.

## Standing scope boundaries

Not open questions - documented properties of this project as scoped, with no plan to close them within this phase of work.

- **No sim-to-real bridge.** This has been a simulation project throughout; hardware validation is explicitly a later phase, not an oversight.
- **No `planner_server`, `bt_navigator`, AMCL, or map.** There is no waypoint-following goal source. Every test in this project drives the robot
  with a manual `/cmd_vel` publish or a hand-built `FollowPath` goal.
- **The stability margin model is pitch-axis-only.** A real 3D support-polygon model accounting for roll simultaneously is a documented next
  step, not implemented.
- **`/odom` drifts under real wheel slip or when the robot is stuck.** Fixed for the controller's own localization by EKF fusion (see
  `retrospective.md`), but raw wheel odometry itself remains unreliable under slip - anything that trusts `/odom` directly, rather than
  `/odometry/filtered`, inherits this.
- **No signal in this project can detect a chassis that's genuinely jammed while its wheels keep rotating near their commanded rate, except the
  motion stall detector's LiDAR scan-consistency check** - see `retrospective.md`'s slip-metric section for why `/wheel_slip` and
  `/stability_margin` both structurally cannot see this failure mode.
- **The motion stall detector false-latches on straight travel**, roughly 12 times per session. No purely inertial signal can distinguish
  steady straight motion from a genuinely blocked chassis, and this project's simulated IMU has no sensor noise to even partially compensate
  for with a bias-subtraction scheme (a real gyro would need one; a 0.05 rad/s simulated bias was shown to mask real freezes in testing).
- **`cmd_vel_watchdog_node.py` is a single point of failure for stopping the robot.** If it stops running, nothing publishes the robot's
  actuation topic at all - the robot holds its last command rather than actively stopping. A real answer needs a heartbeat/liveness pattern
  between it and whatever would need to replace it; not implemented.
- **`/cmd_vel_scale` (from `state_machine_node.py`) has no consumer.** The MPPI profile switch provides an equivalent, planner-enforced speed
  limit per state, so this may be redundant rather than needing its own wiring - an open question for whenever planning work resumes, not a gap
  in what exists today.
- **The loaded leg-4 return-drive lateral drift is wide and characterized, not bounded.** 8 independent trials gave a range of 0.526-2.535 m
  with no active lateral correction on that leg (open-loop once the arc-turn hands off). This is why three props near the gate were removed
  permanently rather than margined against a single worst case - see `results.md`.
- **No tuning pass on `ramp_mode`/`transition_mode`'s exact speed and heading-tolerance numbers** beyond what's needed to prove the
  profile-switching mechanism itself works. The numbers are reasonable placeholders, not validated against a stated requirement.
