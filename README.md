# Ramp Autonomy

A ROS2/Gazebo simulation project for ramp ascent and descent autonomy on a loaded wheeled AMR - the class of problem flat-floor Nav2/MPPI
navigation doesn't handle well: IMU pitch bias corrupting odometry, a shifting payload center of gravity changing the effective tip-over margin
in real time, and wheel slip becoming a distinct failure mode from anything seen on flat ground. Built in simulation first, with hardware
validation as an explicit, separate later phase (see `docs/roadmap.md`).

Construction-site AMRs - the kind that move material between floors - cross temporary ramps constantly, and most published navigation work,
standard Nav2/MPPI tuning included, assumes flat ground. This project treats the ramp as its own control and safety problem rather than
something a flat-floor planner should just power through.

## See it run

Full Gazebo GUI captures of a complete round trip - gate, up the ramp, across the top, back down, back to the gate - with the entire stack
live: EKF, the motion stall detector, the ground-truth-verified goal check, dual LiDAR, descent braking.

| | | |
|---|---|---|
| **Loaded round trip** (100 kg, +0.1 m CG) | 6.6 min, 1280x720 | [download](https://github.com/skunal3318/Ramp-Autonomy/releases/latest/download/round_trip_loaded.mp4) |
| **Unloaded round trip** | 7.3 min | [download](https://github.com/skunal3318/Ramp-Autonomy/releases/latest/download/round_trip_unloaded.mp4) |
| **Loaded delivery mission** (payload released on the dock) | full mission | [download](https://github.com/skunal3318/Ramp-Autonomy/releases/latest/download/delivery_mission.mp4) |

The videos are attached to the [latest release](https://github.com/skunal3318/Ramp-Autonomy/releases/latest) rather than committed, since each
is around 100-230 MB. `recordings/README.md` has the exact numbers for each take, and `visualization/record_round_trip.sh` re-records them.

Screenshots of the realistic construction site and the two-floor building are in `docs/images/`.

## Architecture

```
ramp_autonomy_ws/
├── ramp_autonomy_description/   robot xacro, payload link, cargo demo mode
├── ramp_autonomy_gazebo/        world files, spawn launch
├── ramp_autonomy_nav/           5-state FSM, Nav2 MPPI parameter switching, cmd_vel watchdog
├── ramp_autonomy_perception/    LiDAR-based ramp slope + break-point classifier
├── ramp_autonomy_control/       stability margin, wheel slip, motion stall detector, descent braking, payload delivery
├── ramp_autonomy_critics/       custom nav2_mppi_controller critic plugins
├── ramp_autonomy_bringup/       single launch file for the whole stack
└── validation/phase8/           the test harness and every session's logged data
```

```
APPROACH -> ASCENT -> TRANSITION -> DESCENT -> EXIT
    ^                                              |
    +---------------- HALT (any state) <-----------+
```

Full package breakdown, node graph, and topic list: `docs/architecture.md`.

## Quickstart

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=100.0
```

Full build/run instructions, launch arguments, and how to run the validation suite: `docs/getting-started.md`.

## Documentation

| | |
|---|---|
| `docs/architecture.md` | packages, node graph, topics, the ramp state machine |
| `docs/getting-started.md` | build, run, launch arguments |
| `docs/simulation-world.md` | the construction site, its real-world grounding, the two-floor building |
| `docs/validation-methodology.md` | the ground-truth verification philosophy and the positive-control testing pattern |
| `docs/results.md` | the trusted matrix, round-trip numbers, the delivery mission |
| `docs/retrospective.md` | four findings that changed shape between first observation and root cause |
| `docs/known-limitations.md` | what's actually still open, and what's a standing scope boundary |
| `docs/roadmap.md` | sim-to-real and the rest of what isn't built yet |
| `docs/development-log.md` | the full phase-by-phase record this summary layer is drawn from |
| `docs/construction_site_design.md` | the construction site's full design record: every citation, the complete Fuel sourcing/placement tables |
| [`CONTRIBUTING.md`](CONTRIBUTING.md) | how to build, verify and submit a change |

## Highlights

The full ramp FSM - approach, ascent, a perception-driven transition, descent, exit, with HALT pre-emptable from any state - runs end to end
through real Nav2 `FollowPath`/MPPI control, verified against Gazebo ground truth rather than wheel odometry or FSM state alone. The trusted
7-cell world x friction x payload matrix, the construction-site round trips, and the full loaded delivery mission (3 of 3 clean runs, payload
landed on the dock every time) are in `docs/results.md`.

Four of this project's findings are worth reading in full rather than as a bullet point: a loaded-robot climb failure that looked like a
rigid-body breakover-angle problem and turned out to be a 5 cm geometry authoring bug; a controller that tracked its own plan correctly and
still drove the robot off a ledge because its localization was wrong; a curved-world descent failure that survived three rounds of MPPI tuning
before the real cause turned out to be the test harness's own turn maneuver; and a wheel-slip metric that was correctly validated against real
slip and still had a blind spot wide enough to miss a chassis stuck dead for over four minutes. `docs/retrospective.md` has all four, with the
evidence that separated the first explanation from the real one.

## Contributing

Contributions are welcome - bug reports, independent reproduction of the published results, new ramp geometries or payload configurations, and
work on anything in [`docs/known-limitations.md`](docs/known-limitations.md). A reproduction that *contradicts* a number in
[`docs/results.md`](docs/results.md) is more useful than one that confirms it.

Start with [CONTRIBUTING.md](CONTRIBUTING.md). The one thing it asks for above all else: say how you verified a change, against Gazebo ground
truth rather than against the FSM's own report of its state - [`docs/retrospective.md`](docs/retrospective.md) has two cases where the
difference mattered enormously.

Participation is covered by the [Code of Conduct](CODE_OF_CONDUCT.md).

## Licensing

Licensed under the Apache License 2.0 - see [LICENSE](LICENSE) and [NOTICE](NOTICE).

Third-party assets (the robot's chassis mesh, the Gazebo Fuel props in the construction site) are **not** covered by that license and remain
under their own terms, listed in [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
