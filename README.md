# Subsea ROV Simulator

UiS Subsea ROV team: a BlueROV2-style ROV simulated in ROS 2 Jazzy and Gazebo
Harmonic, with an IMU, pressure-depth sensor, DVL and acoustic positioning, a
navigation filter, and a closed-loop hover controller tested against
disturbances (currents, waves, pushes).

## Contents

| Path | What it is |
|------|------------|
| `src/SUBSEA-WORLD-2026/` | `underwater_world`: world, seabed and the `dummy_rov` vehicle with its sensors |
| `src/SUBSEA-ROV-2026/` | `rov_description`: SDF model of the `uissubsea_rov` |
| `src/subsea_rov_sensors/` | pressure and USBL sensor models, DVL relay |
| `src/subsea_rov_navigation/` | orientation filter + Kalman filter for position/velocity |
| `src/subsea_rov_control/` | hover controller and thruster allocation |
| `src/subsea_rov_bringup/` | launch files, config, test scenarios and results |
| `src/teleop_keyboard.py` | keyboard teleop straight to the thrusters |
| `src/plan.md`, `bluerov2-sim-plan.md` | project plans |

See [src/subsea_rov_bringup/README.md](src/subsea_rov_bringup/README.md) for
how it works and the test results.

## Build and run

```bash
colcon build --symlink-install
source install/setup.bash

# Simulator with GUI and hover controller
ros2 launch subsea_rov_bringup sim.launch.py

# Run all test scenarios headless and write a report
ros2 run subsea_rov_bringup run_validation --output ~/hover_validation
```
