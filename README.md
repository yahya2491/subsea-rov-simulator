# Subsea ROV Simulator

ROS 2 + Gazebo simulation of an underwater ROV with a BlueROV2-style
8-thruster layout.

## Contents

| Path | What it is |
|------|------------|
| `src/SUBSEA-ROV-2026/` | `rov_description` package: SDF model of the `uissubsea_rov` (8 thrusters, hydrodynamics) |
| `src/SUBSEA-WORLD-2026/` | `underwater_world` package: world files and environment models (seabed, rocks, pipeline, `dummy_rov`) |
| `src/teleop_keyboard.py` | Keyboard teleop for `dummy_rov` in Gazebo |
| `src/plan.md` | Phase-by-phase plan for the ROS 2 control layer |
| `bluerov2-sim-plan.md` | Plan for the ArduSub SITL simulation stack |

The two packages under `src/` mirror the
[SUBSEA-ROV-2026](https://github.com/SUBSEA-SIMULATOR/SUBSEA-ROV-2026) and
[SUBSEA-WORLD-2026](https://github.com/SUBSEA-SIMULATOR/SUBSEA-WORLD-2026)
repos (branch `seabed-terrain-and-rov-inertia`).

## Build and run

```bash
colcon build --symlink-install
source install/setup.bash
gz sim src/SUBSEA-WORLD-2026/src/worlds/hydrodynamics_world.sdf

# In another terminal, drive the ROV
python3 src/teleop_keyboard.py
```
