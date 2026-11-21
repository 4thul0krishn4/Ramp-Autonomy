# Third-party assets

The Apache License 2.0 in [LICENSE](LICENSE) covers this repository's original content - code, world/robot models authored for this
project, and documentation. It does not cover the assets listed below, which remain under their own licenses as stated by their sources. Licenses
were confirmed directly against each source (Gazebo Fuel's own model API, the mesh repository's own LICENSE file) at the time this document was
written, not assumed from the asset owner's typical practice.

## Robot chassis/wheel meshes

| File | Source | License |
|---|---|---|
| `src/ramp_autonomy_description/meshes/mir100/mir_100_base.stl` | [DFKI-NI/mir_robot](https://github.com/DFKI-NI/mir_robot), path `mir_description/meshes/visual/mir_100_base.stl`, branch `rolling` @ commit `7d9c72942d512b191f2b326ca89c066710db4e34` | BSD 3-Clause, Copyright (c) 2018, DFKI GmbH |
| `src/ramp_autonomy_description/meshes/mir100/wheel.stl` | same repository, path `mir_description/meshes/visual/wheel.stl` | BSD 3-Clause, Copyright (c) 2018, DFKI GmbH |

Full text: `src/ramp_autonomy_description/meshes/mir100/LICENSE-BSD-3-Clause-DFKI.txt`. These are visual geometry only, used by
`robot.urdf.xacro` for the robot's `<visual>` elements; no URDF structure, control plugins, or collision geometry was taken from `mir_robot`. Note:
these are MiR100 meshes. An earlier version of this project used Husky A200 meshes from Clearpath Robotics' `clearpath_common`
(BSD-3-Clause) - that mesh swap was fully superseded and removed; no Husky/Clearpath files remain in this repository.

## Gazebo Fuel models (construction site props)

Referenced by live `<uri>https://fuel.gazebosim.org/...</uri>` in `src/ramp_autonomy_gazebo/worlds/construction_site.sdf` - Gazebo fetches and
caches these from Fuel's own servers at simulation launch time. No model files are copied into this repository; only the reference and the
placement (position, count) are this project's own content.

| Model | Owner | License | Used for | Status |
|---|---|---|---|---|
| Jersey Barrier | OpenRobotics | CC0 1.0 Universal | 2x, Floor 2 ramp-2 exit lane divider | in the current world |
| Euro pallet | OpenRobotics | CC0 1.0 Universal | laydown area incoming pallets, Floor 2 block pallets | in the current world |
| Cinder Block | OpenRobotics | CC0 1.0 Universal | laydown area stock, 48x on Floor 2 pallets | in the current world |
| Large Crate | OpenRobotics | CC-BY 4.0 | laydown area staged crates, 2x Floor 2 staging | in the current world |
| Pallet_Rack_Section | OpenRobotics | CC-BY 4.0 | laydown area, nearest the ramp | in the current world |
| Dumpster_Large | OpenRobotics | CC-BY 4.0 | waste/debris staging pad | in the current world |
| std_cargo_container_40ft | jfrascon | CC-BY 4.0 | office pad, second structure beside the office building | in the current world |
| 55gal_Drum | OpenRobotics | CC-BY 4.0 | 2x, Floor 2 material staging | in the current world |
| Cable_Spool | OpenRobotics | CC-BY 4.0 | 1x, Floor 2 material staging | in the current world |
| fence | athackst | CC-BY 4.0 | gate-opening fence panels | removed (see note below) |
| Construction Cone | OpenRobotics | CC0 1.0 Universal | access road, near the gate | removed (see note below) |
| Speed limit sign | OpenRobotics | CC0 1.0 Universal | access road, near the gate | removed (see note below) |

**CC-BY 4.0 attribution**: each CC-BY-licensed model above is attributed to its listed owner on Gazebo Fuel (app.gazebosim.org), per the license's
own attribution requirement. CC0 models carry no attribution requirement; they are credited here anyway for a complete record of what this
project uses.

**Removed props**: `gate_fence_north/south`, `gate_cone_1/2/3`, and `access_road_speed_sign` were placed near the gate during site population,
then permanently removed once a later fix changed the leg-4 return drive's drift pattern enough to put them inside the robot's characterized
worst-case path (see `docs/construction_site_design.md`, "Closing this: leg-4 drift characterized, gate props stay removed permanently"). They
are listed here because they were part of this project's history and their removal is a safety finding, not a licensing one - they are not in the
current `construction_site.sdf` and nothing in this repository fetches them today.

## Citations, not redistributed content

Design documents (`docs/construction_site_design.md`, `docs/simulation-world.md`) cite real-world standards and sources - OSHA regulations,
industry site-layout guidance, manufacturer specifications - as justification for chosen dimensions. These are hyperlinks to external material,
not content copied into this repository, and carry no licensing obligation here.
