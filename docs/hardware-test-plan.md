# Hardware test plan

This project is simulation only. [Issue #11](https://github.com/skunal3318/Ramp-Autonomy/issues/11)
asks for someone with a real robot and a ramp. This document is what that
actually involves, written out in advance so nobody has to guess what is being
asked for.

The aim of a first session is **not** to make the robot work. It is to find out
where the simulation is lying. The fastest way to do that is to start with the
robot doing almost nothing and add one variable at a time.

---

## What is needed from the robot

Five numbers. Most of this plan depends on them, and two cannot be guessed:

| | why it matters |
|---|---|
| mass, kg | sets the payload ratio, see [limitations](#what-this-cannot-test) |
| wheelbase, front to rear axle, m | with CG height, sets how much load shifts under braking |
| CG height above ground, m (rough is fine) | same |
| wheel radius, m | converts encoder counts to ground speed for slip |
| max payload, and where it mounts | whether the loaded case is testable at all |

Also useful: drive type and whether all wheels are driven, tyre material, whether
there are wheel encoders or only visual odometry, and the ROS 2 distro.

Wheelbase and CG height matter more than they look. On the simulated robot, CG
height over wheelbase is 0.49, which is tall, and that ratio dominates how much
braking the uphill wheels tolerate before slipping. A long, low robot will behave
very differently, in a good way.

---

## The ramp

**Shape.** For a first session, one straight constant slope.

What the full system eventually needs is flat ground, then ramp, then flat ground
again, because the two break points are where the state machine does most of its
work and where hardware is most likely to disagree with simulation. That needs a
raised platform at the top, which is a lot of building for a first day. A single
propped sheet, with the robot stopping short of the top edge, covers ascent and
descent, which is plenty to begin with.

**Surface.** Normal, and the highest grip readily available. Bare plywood or
painted concrete is ideal.

Low friction is the interesting case eventually, but deliberately, later, and as
a patch partway down rather than the whole ramp. Reducing grip on a first run
just means the robot slides, and nothing is learned except that it slid. Avoid
anything dusty or loose entirely on a first session.

**Angle.** Three stages, in order:

| stage | angle | prop height on a 2.44 m sheet | minimum friction to not slide |
|---|---|---|---|
| A | 6 deg | 0.255 m | 0.105 |
| B | 10 deg | 0.424 m | 0.176 |
| C | 16 deg | 0.673 m | 0.287 |

16 degrees is what the simulation uses, so stage C is the comparison that
matters. Only go there if stage B is clean. Stopping at 10 is a result, not a
failure.

That last column is a hard floor: a rigid body on a slope needs friction above
`tan(angle)` or it slides regardless of control quality. Rubber on dry plywood is
roughly 0.5 to 0.7, so 16 degrees should be comfortable, but the number is worth
knowing. [Issue #3](https://github.com/skunal3318/Ramp-Autonomy/issues/3)
brackets the practical floor in simulation.

**Build.** A single 2440 x 1220 mm plywood sheet propped at one end is the whole
ramp. Changing the prop height changes the angle, so nothing needs cutting. Check
that the sheet does not flex under the robot, and chock the bottom edge so it
cannot kick out.

---

## Stages

**Stage 0, no ramp.** Record the five numbers above. Drive a measured straight
line on flat ground under teleop and compare encoder distance against a tape
measure. This is the odometry baseline every later number is read against.

**Stage 1, flat ground, loaded.** Same line with the payload mounted. If odometry
error changes with payload on flat ground, that is worth knowing before a slope
is involved.

**Stage 2, 6 degrees, teleop only.** Up, stop, down, around 0.2 m/s. No autonomy.
What matters is whether IMU pitch reads the true ramp angle, and whether wheel
odometry starts over-reading against ground truth on the slope. Those two are the
strongest suspicions about the simulation.

**Stage 3, 10 then 16 degrees.** As stage 2.

**Stage 4, autonomy.** Only once the above is boring.

The reason for so much teleop first: if the full stack goes on at stage 0 and the
robot behaves badly, there is no way to tell whether the control is wrong, the
odometry is wrong, or the ramp is not what it was believed to be. In this order,
each stage has exactly one new unknown.

---

## Safety

There is a failure mode in this stack that matters far more on hardware than in
simulation. `cmd_vel_watchdog_node` is the only publisher of the robot's
actuation topic. If it stops running, nothing publishes commands at all, and the
Gazebo DiffDrive plugin holds its last one. A stopping mechanism whose failure
mode is "keep going" is the wrong way round. It is open as
[issue #4](https://github.com/skunal3318/Ramp-Autonomy/issues/4) and it should be
fixed before any hardware work, not during it.

Until it is, every run including teleop needs:

- a physical e-stop or held kill switch, not a software one
- nobody downhill of the robot, ever
- a tether or a hand on the robot for the first run at each new angle
- speed capped at 0.2 m/s, below the 0.35 m/s the simulation uses
- the top edge blocked, or a stop point marked well short of it

This is not about distrusting anyone's hardware. It is that this stack has a
documented fail-open path, and putting it on a real slope without saying so first
would be wrong.

---

## What this cannot test

The simulated robot is 48 kg carrying a 100 kg payload, so it carries more than
twice its own mass. That ratio is why the project exists: it is what moves the
centre of gravity far enough to matter on a grade.

Most small research rovers carry a fraction of their own mass. At a ratio near
0.1 to 0.25, the tip-over margin and shifting-CG behaviour are simply not
reproduced, and a token weight should not be presented as validating them.

What is genuinely testable at any payload ratio, and is most of the value:

- whether IMU pitch tracks a real slope well enough to drive the state machine
- whether wheel odometry degrades on a grade the way this project claims
- whether real wheel slip resembles the slip estimate at all
- whether LiDAR slope detection fires at the right place on a real ramp

Those four are the ones most worth being wrong about.

---

## What to record

Per run, including teleop: a rosbag of IMU, wheel encoders or joint states,
whatever odometry the robot publishes, and visual odometry pose if available.
Plus, written down by hand: angle, surface, payload mass, commanded speed, and
whether anything felt wrong.

Tape-measure ground truth at the start and end of every run. It is the only
measurement in a session that cannot be wrong in an interesting way, and it is
what everything else is compared against.

Photos or video of every run, especially the failures.
