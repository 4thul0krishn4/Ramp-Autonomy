# Roadmap

## Sim-to-real: explicitly out of scope for this phase

This has been a simulation project from the start, by design, not because hardware validation was deferred by accident. Everything in
`results.md` is a Gazebo result. Moving any of it to real hardware - a real MiR100 or comparable AMR, real IMU noise characteristics, real
wheel-ground contact instead of ODE's Coulomb friction model - is a distinct future phase with its own validation work, not an extension of the
current one.

## Slope-aware planning and control (phase 5, not started)

The project's own phase plan calls this out as the next phase once a waypoint-following goal source exists. Nothing here has been built: there
is no `planner_server`, `bt_navigator`, AMCL, or map, and every test in this project drives the robot with a manual `/cmd_vel` publish or a
hand-built `FollowPath` goal rather than a real navigation-stack goal. `ramp_mode` and `transition_mode`'s exact speed and heading-tolerance
numbers are untuned placeholders, proven to switch correctly but never validated against a stated performance requirement - that tuning pass
belongs here too, once there's a real goal source to tune against.

## A 3D, roll-aware stability margin

The current tip-over margin model is pitch-axis-only. A real support-polygon calculation accounting for roll simultaneously is a documented next
step - not started, and not needed for any ramp geometry this project has tested so far, since every ramp here is a straight incline with no
meaningful roll component during a clean traverse.

## A liveness check for the actuation-topic watchdog

`cmd_vel_watchdog_node.py` is the sole publisher of the robot's real actuation topic, and if it stops running, the robot holds its last command
rather than stopping. A heartbeat or liveness pattern between it and some independent monitor would close this; not designed or built.

## Pinning down the practical friction floor

The practical minimum climbable friction for this robot and ramp sits somewhere in (0.35, 0.45] - meaningfully above the theoretical
`tan(16°) ≈ 0.287` static-friction floor, because real acceleration and weight-transfer effects eat into the margin right above it, but not
pinned down more precisely than that bracket. A finer bisection between those two values wasn't run. This was measured on the pre-crest-fix
geometry and has not been re-measured since.

## Extending the delivery mission

The delivery mission currently handles one payload, one pad, one release. Multiple sequential deliveries, a return trip carrying a different
payload, or delivery to a chosen one of several pads are all natural extensions nothing in the current `payload_delivery_node.py` design
precludes, but none are built.
