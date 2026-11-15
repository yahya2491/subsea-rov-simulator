# Subsea ROV simulator

UiS Subsea ROV team: a BlueROV2-style ROV simulated in ROS 2 Jazzy and Gazebo
Harmonic, with sensors, navigation and a hover controller.

## What's in it

- **Sensors on the vehicle** (`dummy_rov` in `underwater_world`):
  - IMU and magnetometer
  - DVL (Gazebo ray-casts its beams against the seabed)
  - pressure/depth sensor
  - USBL acoustic positioning, with latency, dropouts and outliers
- **Navigation** (`subsea_rov_navigation`):
  - an AHRS filter for orientation (gyro + accelerometer + magnetometer)
  - a Kalman filter for position and velocity (DVL + depth + USBL)
- **Control** (`subsea_rov_control`):
  - a hover controller (PID per axis) that holds position and heading
  - thruster allocation onto the 8 thrusters
- **Testing** (`subsea_rov_bringup`):
  - disturbance scenarios: currents, waves, pushes, setpoint changes
  - a script that runs them all and writes a report with plots

The controller only uses the navigation estimate, never Gazebo's ground truth.
Ground truth is only used to drive the pressure and USBL models, and to score
the tests.

## Running it

```bash
cd ~/subsea_final
colcon build --symlink-install
source install/setup.bash

# Simulator with GUI; the ROV holds the position where it starts
ros2 launch subsea_rov_bringup sim.launch.py

# Move the hold point (world frame, z up, surface at z = 10 m)
ros2 topic pub --once /rov/hover/setpoint geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.5, z: 4.0}, orientation: {w: 1.0}}}"

# Add a 0.2 m/s current
ros2 topic pub --once /ocean_current geometry_msgs/msg/Vector3 "{x: 0.2, y: 0.0, z: 0.0}"

# Run all test scenarios headless (~12 min), writes report.md + plots
ros2 run subsea_rov_bringup run_validation --output ~/hover_validation

# Unit tests
colcon test && colcon test-result --verbose
```

Use `controller:=false` to run only the sensors and navigation, and
`gui:=false` to run headless.

## How it works (short version)

- **Orientation:** gyro integration, corrected by the accelerometer
  (roll/pitch) and magnetometer (heading only). The DVL-derived acceleration
  is subtracted, so pushes aren't mistaken for tilt.
- **Position:** the IMU predicts, then DVL velocity, pressure depth and USBL
  fixes correct the estimate. Bad measurements are rejected, and late USBL
  fixes are shifted to the current time.
- **Hover control:** one PID per axis, with position error in the world frame.
  The vehicle's +14.6 N buoyancy is fed forward. Gains come from pole
  placement on `m·a + d·v = F` (see `config/control.yaml`).
- **Allocation:** pseudo-inverse of the 6×8 thruster matrix. When thrusters
  saturate, the horizontal and vertical groups are scaled separately.

All settings are in `config/`. The thruster layout and sensor positions are
written in both the SDF and `vehicle.yaml`; a test checks that they match.

## Results

| scenario | disturbance | max drift | drift at end (RMS) | max depth error | back on station after | result |
|---|---|---|---|---|---|---|
| calm_hold | still water | 2 cm | 0.8 cm | 0.2 cm | – | pass |
| steady_current | 0.22 m/s current switches on | 12 cm | 1.1 cm | 0.2 cm | 3.6 s | pass |
| current_heading_change | 0.2 m/s cross current + 90° turn | 10 cm | 1.2 cm | 0.3 cm | – | pass |
| wave_orbital | wave-like water motion, 7 s period | 10 cm | 5.6 cm | 7.1 cm | – | pass |
| impulse_push | 40 N and 30 N shoves | 32 cm | 1.4 cm | 22.8 cm | 4.4 s | pass |
| setpoint_steps | 1 m moves, 1 m depth change, 45° turn | (moving to new setpoints) | 1.1 cm | (moving to new setpoints) | 8.2 s | pass |

The navigation position error was 1.2 cm RMS or less in every run. The full report with plots is in [validation/report.md](validation/report.md).

## Limitations

- **Drag:** Gazebo only has linear drag and no added mass, so the gains would
  need retuning on the real ROV.
- **Pressure and USBL:** these are noise models on top of ground truth, not
  acoustic simulations.
- **Water surface:** there isn't one. The surface is taken as the plane
  z = 10 m.
