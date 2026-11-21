# Third-party mesh assets

`mir_100_base.stl` and `wheel.stl` are visual mesh files (geometry only - STL carries no material/color; the real robot's own material colors are
reproduced as plain URDF `<material>` definitions in `robot.urdf.xacro`, not part of these files) taken unmodified from DFKI-NI's `mir_robot`
repository (community-maintained ROS/ROS2 description of MiR's mobile robots):

- Source: https://github.com/DFKI-NI/mir_robot
- Path: `mir_description/meshes/visual/mir_100_base.stl`, `mir_description/meshes/visual/wheel.stl`
- Branch/commit: `rolling` @ `7d9c72942d512b191f2b326ca89c066710db4e34` (fetched 2026-09-30)
- License: BSD 3-Clause, Copyright (c) 2018, DFKI GmbH — see `LICENSE-BSD-3-Clause-DFKI.txt` in this directory.

**These are MiR100 meshes, not MiR250.** The task asked for MiR250; `DFKI-NI/mir_robot` (the named source) only publishes MiR100 meshes, and no
properly-licensed, established open-source MiR250-specific mesh package could be found (several personal repos exist but ship no LICENSE file, so
they are not safely redistributable - checked, not assumed). Confirmed with the user before substituting: use the MiR100 meshes from this same
source, labeled honestly as MiR100 everywhere (this file, `robot.urdf.xacro`'s own comments, the design doc, screenshots) rather than mislabeled as
MiR250. Same manufacturer (Mobile Industrial Robots), same general AMR chassis family.

Only the visual mesh files were taken. No URDF/xacro structure, control plugins, sensor mounts, transmission config, or collision geometry from
`mir_robot` was used anywhere in this project - `robot.urdf.xacro`'s own link/joint/collision/inertial structure is unchanged and remains this
project's own. See `docs/construction_site_design.md` ("Chassis mesh swap: MiR100") for how these files are used (per-axis scale to this project's
own collision dimensions) and the before/after structural-identity verification.
