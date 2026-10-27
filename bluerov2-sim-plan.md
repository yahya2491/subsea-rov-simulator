# BlueROV2 Simulation Stack: Implementation Plan

## Context

We own a BlueROV2 and will compete at TAC Challenge (Tau Autonomy Center, Norway, next edition expected June 2027). We need an indoor development simulator so mission software can be written and regression-tested without pool access.

The vehicle runs stock ArduSub on a Pixhawk. The simulator must therefore run the same ArduSub firmware in SITL so that mission code targets an identical MAVLink interface in sim and on hardware. Any design that requires different application code in sim versus on the vehicle is a failure of the objective.

## Target environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (installed via ROS 2 vendor packages, not the standalone osrfoundation packages)
- ArduSub from ArduPilot master, built for SITL
- `ardupilot_gazebo` (JSON model backend)
- mavros

Everything must run inside a Docker dev container with an `x11`/nvidia compose profile. Nobody on the team should need to install this by hand. A working `docker compose up` is part of the deliverable, not an afterthought.

## Architecture

```
ROS 2 mission nodes
        |  (ROS 2 topics/services)
      mavros
        |  (MAVLink over UDP)
   ArduSub SITL          <- real autopilot firmware, x86 build
        |  (JSON sensor/actuator interface)
 ardupilot_gazebo plugin
        |
  Gazebo Harmonic        <- world, vehicle model, sensors, hydrodynamics
```

Gazebo owns physics and sensing. ArduSub owns control, mixing and state estimation. Neither role moves.

## Repository layout

Single colcon workspace, one repo, these packages:

```
uis_rov_sim/
  uis_rov_description/    vehicle SDF/xacro, sensor mounts
  uis_rov_worlds/         TAC world, props, pool world, open water world
  uis_rov_bringup/        launch files, params, ArduSub param files
  uis_rov_missions/       mission logic (later phases)
  docker/                 Dockerfile, compose files
```

Vendored as git submodules or vcs repos, not copied in: `ardupilot_gazebo`, `bluerov2_gz` (model source), ArduPilot.

## Base packages: what to use and why

Use `clydemcqueen/bluerov2_gz` as the **source of the vehicle model only**. It targets Gazebo Harmonic and uses the built-in `Buoyancy`, `Hydrodynamics` and `Thruster` systems, and covers both base and heavy configurations. Take its model and adapt it into `uis_rov_description`. Do not depend on its worlds or scripts long term; we need our own.

Do **not** adopt `Robotic-Decision-Making-Lab/blue` as the framework. It is a good reference for how to wire ArduSub SITL, `ardusub_driver` and ros2_control together, and should be read, but it brings an abstraction layer the team would have to learn and maintain on top of the parts we actually need to customise. Read it, borrow from it, do not inherit from it.

Do not use `uuv_simulator`, `freebuoyancy_gazebo`, or anything targeting Gazebo Classic. All of it is dead for our ROS version.

---

## Phase 0: Environment

**Goal:** reproducible container, nothing else.

Tasks:
1. Dockerfile on `ros:jazzy-ros-base`, installing Gazebo Harmonic via `ros-jazzy-ros-gz`, ArduPilot build deps, MAVProxy, mavros and its geographiclib datasets.
2. Build ArduSub for SITL inside the image.
3. Build `ardupilot_gazebo` against Harmonic.
4. Compose files for nvidia and non-nvidia hosts, with X11 forwarding and host networking.
5. `README.md` with a single command to get a Gazebo window open.

**Acceptance:** a teammate with a clean Ubuntu 24.04 machine and Docker gets an empty Gazebo Harmonic window running from one command, with no host-side ROS or Gazebo installation.

---

## Phase 1: Vehicle in an empty underwater world

**Goal:** the BlueROV2 floats, sinks and responds to direct thrust, with no autopilot involved.

Tasks:
1. Port the `bluerov2_gz` base model into `uis_rov_description` as xacro so we can parameterise sensor mounts later.
2. Create `uis_rov_worlds/empty_underwater.sdf`: flat seafloor, water surface at Z=0, correct `Buoyancy` and `Hydrodynamics` configuration.
3. Launch file `sim.launch.py` that starts Gazebo with the world and spawns the vehicle.
4. A script that publishes thrust directly to the six `Thruster` topics.

**Acceptance:** vehicle is neutrally buoyant at rest with no thrust, does not drift or oscillate, and responds sensibly to direct thruster commands in surge, sway, heave and yaw. Confirm the hydrodynamic damping is not obviously wrong by checking that a thrust step settles to a steady velocity rather than accelerating indefinitely.

**Note for whoever does this:** the hydrodynamic coefficients shipped in every public BlueROV2 model are literature approximations, not measurements of our vehicle. Treat Phase 1 as "plausible", not "correct". Flag any coefficient you change and why, in a comment.

---

## Phase 2: ArduSub SITL in the loop

**Goal:** the real autopilot is flying the simulated vehicle. This is the critical phase.

Tasks:
1. Attach the `ardupilot_gazebo` plugin to the vehicle model, JSON backend.
2. Launch ArduSub SITL with frame `vectored` (use `vectored_6dof` if and only if our vehicle is the Heavy configuration; confirm before writing this).
3. Bring up mavros pointed at the SITL UDP endpoint.
4. Verify with MAVProxy first (`arm throttle`, `rc 3 1450`, `mode alt_hold`), then with ROS 2.
5. Write a `param/` directory and load our real vehicle's parameter file into SITL.

**Acceptance, and this is the one that matters:** export the parameter file from our physical BlueROV2, load it unmodified into SITL, and confirm `ALT_HOLD` and `STABILIZE` behave sanely. If parameters have to be changed to make SITL usable, stop and report which ones and why. Divergence here means we are simulating a different vehicle and every downstream result is suspect.

Second acceptance: a ROS 2 node arms the vehicle, commands a descent to 2 m, holds depth, and disarms, using only mavros. That node must be runnable against the real ROV later with no code change.

---

## Phase 3: TAC world and props

**Goal:** a world that resembles the competition, not a generic seabed.

Source material: TAC Challenge 2026 Mission Booklet and Rules and Regulations (`tacchallenge.com`), plus the shared Info Folder. Pull dimensions from those documents.

Tasks:
1. Model the docking station to real dimensions as an SDF model.
2. Model a pipeline made of connectable segments, so an "unknown route" can be assembled from a config file rather than hand-placed.
3. Model the detection targets (defined objects and images) with correct sizes and the actual image textures where we have them.
4. Model a valve panel geometry. Geometry only; we are not simulating valve actuation physics in this phase.
5. Build two worlds: an indoor pool world matching the TAC pool dimensions, and an open water world.
6. Make prop placement data-driven via a YAML scenario file, so a mission run can be randomised.

**Acceptance:** a scenario YAML produces a repeatable world, and changing a seed produces a different pipeline route without editing SDF by hand.

---

## Phase 4: Sensors

Add one at a time, each with its own acceptance test. Do not add all four then debug.

1. **Camera.** Gazebo camera plus `ros_gz_bridge`, published on the same topic name the real ROV's camera will use.
2. **Depth/pressure and IMU.** Already consumed internally by ArduSub, but expose them to ROS for logging.
3. **DVL.** No stock Harmonic plugin. Use Project DAVE's ROS 2 branch (targets Jazzy and Harmonic) or, if integration cost is high, write a lightweight node that derives body velocity from Gazebo ground truth and adds configurable noise, dropout and bottom-lock loss. The fake version is acceptable and probably preferable at this stage.
4. **Acoustic positioning (USBL).** No plugin exists. Write a node publishing ground truth position with injected noise, latency and dropout, matched to the update rate and accuracy figures of whatever unit we actually buy.
5. **Multibeam/imaging sonar.** Only if we commit to sonar on the vehicle. Project DAVE has a physics-based multibeam plugin migrated to Harmonic with CUDA acceleration. Requires an NVIDIA GPU. This is the highest-effort sensor by a wide margin; do not start it until it is confirmed to be on the real vehicle.

**Acceptance per sensor:** topic name, message type, frame_id and update rate all match what the real hardware will publish. Write this down in a table in the repo, because it is the contract between the sim team and the vehicle team.

---

## Phase 5: Mission logic

Only after Phases 2 to 4 are green.

Build the mission layer as behaviour trees or explicit state machines in `uis_rov_missions`, one per TAC task: docking approach, pipeline following, object detection and reporting, valve approach. Each mission must be launchable standalone against a scenario file.

Every mission node reads sensor topics and writes mavros setpoints. No mission node may touch a Gazebo topic. That rule is what keeps the code deployable.

---

## Phase 6: CI

1. Headless Gazebo runs in GitHub Actions.
2. A smoke test per phase: vehicle spawns and settles, SITL arms and holds depth, each mission runs to completion on a fixed scenario seed within a time budget.
3. Fail the build on regression.

This is not optional polish. With multiple students editing the same workspace across a semester, missing CI means silent breakage discovered a week before the competition.

---

## Explicit non-goals

State these clearly so nobody wastes time:

- **Tether dynamics are not modelled.** None of the available packages do this. Tether drag materially affects station keeping near the docking station. Station-keeping gains tuned in sim will need retuning in the pool. Do not chase this in simulation.
- **Underwater optics are not realistic.** No turbidity, backscatter, colour attenuation or light falloff from our own LEDs. Sim imagery is for developing the control loop that consumes a detection. A detector trained on sim images will not transfer. Real footage from a pool with the real props is required.
- **Contact physics for valve manipulation is unreliable** in Gazebo underwater. Use the sim for approach and reach geometry only.
- **Timing is not real.** SITL has no bus contention and can run off wall clock. Timing bugs will still appear on hardware.
- **Hardware failure modes do not exist** unless we inject them. Leaks, ESC dropout, tether comms loss, brownout. If mission code must handle these, add fault injection deliberately in a later phase.

## Reference material

- `github.com/clydemcqueen/bluerov2_gz`
- `github.com/ArduPilot/ardupilot_gazebo`
- `github.com/Robotic-Decision-Making-Lab/blue` (read only)
- `github.com/CentraleNantesROV/bluerov2` (useful sensor and control examples, non-ArduSub)
- Project DAVE ROS 2 port: `dave-ros2.notion.site`
- ArduSub SITL setup: `ardupilot.org/dev/docs/setting-up-sitl-on-linux.html`
- TAC Challenge 2026 mission booklet and rules: `tacchallenge.com`

## Order of work

Phases are sequential. Do not start Phase 3 before Phase 2 acceptance passes. The single highest-risk item in this plan is the Phase 2 parameter-file test; if that fails, everything after it is built on a vehicle we do not own.
