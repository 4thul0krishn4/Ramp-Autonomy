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
- **The clean-host guard now refuses guarded trials on the construction-site world on commodity hardware.** `harness_guard.py`'s `MIN_RTF` is a
  hard gate of 0.95, checked before every guarded trial, and the Floor-2 construction site no longer clears it. Measured on a 12-core laptop
  with an RTX 3050: roughly **0.90-0.92 headless**, and roughly **0.70 with the Gazebo GUI running**, so a guarded trial on that world aborts
  immediately with `HARNESS GUARD FAILED - DO NOT RUN THE TRIAL`. Three things follow, and all three matter to anyone trying to reproduce
  `results.md`:
  - `HG_LOW_RTF` does **not** help. It relaxes only the health monitor's "two samples below this" clause during a run; `MIN_RTF` is a separate
    module constant with no environment override, so the pre-trial check fails before the monitor is ever reached.
  - This is a property of the world, not of the host being busy. `harness_guard.py`'s own comment records the Floor-2 world at a steady ~0.91 on
    an idle host with a load average of ~6, against ~0.98 for the Floor-1-only world the 0.95 figure was calibrated on. Shrinking the GUI window
    changes nothing measurably - the cost is physics and dual-LiDAR raycasting, not render resolution.
  - **No filmed run of this world can be validation-grade on hardware of this class.** The recorded round trips in `recordings/` sustained 0.94
    because they were captured on 2026-09-20, before the Floor-2 extension existed; the same capture on the current world costs substantially
    more. This is a sharper version of the caveat `recordings/README.md` already makes.

  Not fixed, and deliberately not papered over by lowering `MIN_RTF`: the gate is doing exactly what it was built to do, which is refuse to
  produce a number the project would then have to defend. The real fix is either a cheaper world (fewer collision shapes, cheaper LiDAR
  raycasting) or an explicit, recorded decision to recalibrate the threshold for the Floor-2 world. Neither has been done. If you hit this,
  please open an issue with the real-time factor your machine sustains - how widespread it is across hardware is not yet known.

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
