## Plan: 8-Thruster Gazebo + ROS2 Control Stack

Add a BlueROV2-style 8-thruster configuration to the active `uissubsea_rov` model, enable Gazebo Sim underwater dynamics (thruster + buoyancy + hydrodynamics), and provide ROS 2 control through a new Python package that supports both direct 8-channel thrust and high-level `Wrench`/`Twist` interfaces via `ros_gz_bridge`.

**Scope**
- Included: SDF model changes, underwater dynamics plugins, new ROS 2 control package, bridge config, launch orchestration, and validation/tuning workflow.
- Included: both direct per-thruster and high-level motion command paths.
- Excluded: hardware-in-the-loop, real thruster driver integration, advanced state estimation, mission autonomy, and final hydrodynamic identification from tank data.

**System Design (Target End State)**
- Gazebo model owns physical actuation and hydrodynamics.
- ROS 2 node owns command arbitration, allocation, saturation, and safety fallback.
- `ros_gz_bridge` maps ROS topics to Gazebo thruster command topics.
- A single launch entrypoint starts Gazebo world, bridge, and mixer in the correct order.

**Phase 0: Baseline and Conventions (Blocking)**
1. Define body-frame convention and sign rules to prevent later axis inversion bugs.
2. Freeze thruster indexing and naming convention: `thruster_0_joint` to `thruster_7_joint`, with matching link names.
3. Freeze interface naming under a stable ROS namespace: one direct 8-channel input, one `Wrench` input, one `Twist` input, and 8 per-thruster output topics.
4. Freeze safety rules: command timeout, max/min thrust clamp, startup zero-output behavior, and interface priority policy.
5. Document these conventions in the control package README and config comments.

**Phase 1: ROV SDF Actuation + Underwater Physics**
1. Edit `/home/yahya/subsea_final/src/SUBSEA-ROV-2026/src/models/uissubsea_rov/model.sdf` to add 8 thruster links with lightweight inertials and simple collision/visual geometry for debugging.
2. Add 8 joints connecting each thruster link to `base_link`; use continuous/revolute joints with axis direction aligned to intended thrust direction.
3. Arrange thrusters in BlueROV2-style pattern.
4. Configure 8 `gz::sim::systems::Thruster` plugins (`filename="gz-sim-thruster-system"`) with one plugin instance per thruster joint.
5. Use force command mode (`cmd_thrust`) initially for simpler high-level mapping and consistent units in Newtons.
6. Set initial plugin parameters per thruster: namespace, joint name, fluid density, propeller diameter, thrust coefficient, deadband, min/max command.
7. Add buoyancy configuration so neutral/near-neutral behavior is possible at zero thrust.
8. Add `gz::sim::systems::Hydrodynamics` (`filename="gz-sim-hydrodynamics-system"`) on `base_link` with initial diagonal damping/added-mass terms.
9. Remove or disable conflicting generic control systems in the same model (notably `VelocityControl`) so they do not fight thruster-based actuation.
10. Keep `UserCommands` only if needed for manual debug interactions.
11. Verify model still parses and spawns cleanly with no SDF/plugin warnings.

**Phase 2: New ROS 2 Control Package (`subsea_rov_control`)**
1. Create package at `/home/yahya/subsea_final/src/subsea_rov_control`.
2. Use Python (`rclpy`) node for rapid tuning and easy parameter iteration.
3. Add dependencies in `package.xml`: `rclpy`, `std_msgs`, `geometry_msgs`, `launch`, `launch_ros`, `ros_gz_bridge` runtime integration dependencies.
4. Add install rules in `CMakeLists.txt` for Python module, launch files, and YAML configs.
5. Implement `thruster_mixer_node.py` with three input modes.
6. Input mode A: direct 8-channel command (`Float64MultiArray` length must equal 8; reject and log on invalid length).
7. Input mode B: `geometry_msgs/Wrench` mapped through 8x6 allocation matrix to thruster forces.
8. Input mode C: `geometry_msgs/Twist` converted to desired body wrench via tunable gains, then passed through same allocation matrix path.
9. Implement source arbitration policy with explicit priority and freshness timeout.
10. Implement safety logic: stale input timeout to zero, output clamp, optional rate limiter, and startup zero.
11. Publish 8 scalar ROS outputs intended for bridge forwarding to each Gazebo thruster topic.
12. Publish optional debug topics: selected source, commanded wrench, post-saturation thruster vector.

**Phase 3: Parameterization and Bridge Mapping**
1. Add `/home/yahya/subsea_final/src/subsea_rov_control/config/thruster_allocation.yaml` for:
2. Thruster geometric parameters and lever arms.
3. 8x6 allocation matrix coefficients.
4. Per-thruster thrust limits and deadband values.
5. `Twist`-to-`Wrench` gains and max wrench limits.
6. Input arbitration and timeout settings.
7. Add `/home/yahya/subsea_final/src/subsea_rov_control/config/bridge.yaml` with 8 explicit ROS-to-Gazebo topic mappings from ROS outputs to `/model/uissubsea_rov/joint/<joint>/cmd_thrust`.
8. Optionally map feedback topics (`ang_vel`, pose, odom) if needed for diagnostics.
9. Keep bridge topic names deterministic and documented to reduce launch/debug friction.

**Phase 4: Unified Launch and Runtime Wiring**
1. Add `/home/yahya/subsea_final/src/subsea_rov_control/launch/rov_gz_control.launch.py`.
2. Launch Gazebo Sim with `/home/yahya/subsea_final/src/SUBSEA-WORLD-2026/src/worlds/hydrodynamics_world.sdf`.
3. Ensure `GZ_SIM_RESOURCE_PATH` includes installed model/world paths so `model://uissubsea_rov` resolves.
4. Start `ros_gz_bridge parameter_bridge` using `bridge.yaml`.
5. Start `thruster_mixer_node.py` with `thruster_allocation.yaml` and selected defaults.
6. Add launch arguments for mode defaults and optional simulation verbosity.
7. Ensure startup order avoids race conditions between bridge readiness and initial command publication.

**Phase 5: Verification and Tuning Workflow**
1. Build and source with `colcon build --symlink-install` and `source install/setup.bash`.
2. Spawn simulation and confirm no plugin load failures.
3. Verify all 8 Gazebo command topics exist and bridge endpoints are active.
4. Direct-mode validation: command each thruster independently and confirm expected thrust direction.
5. Wrench-mode validation: test pure surge, sway, heave, roll, pitch, yaw and check activation symmetry.
6. Twist-mode validation: test each velocity axis and verify reasonable translated wrench behavior.
7. Safety validation: stop sending commands and verify timeout-to-zero response.
8. Saturation validation: send oversized commands and confirm clamping behavior.
9. Hydrodynamics sanity: with zero command, vehicle should not accelerate indefinitely; with steady command, speed should asymptotically settle due to drag.
10. Iterative tuning loop: thrust coefficients, damping terms, buoyancy volume, and Twist gains until stable handling is achieved.

**Parallelism and Dependencies**
1. Parallel after conventions are fixed: Phase 1 SDF editing can run in parallel with Phase 2 package scaffolding.
2. Blocking relation: mixer logic must be complete before final bridge config and launch validation.
3. Parallel late-stage: verification script writing and README updates can run while tuning proceeds.

**Detailed Deliverables**
1. Updated ROV model SDF with 8 thrusters and underwater dynamics.
2. New `subsea_rov_control` package with mixer node.
3. Allocation and bridge YAML configs.
4. Unified launch file for reproducible bring-up.
5. Documentation section describing topic interfaces and control modes.

**Relevant files**
- `/home/yahya/subsea_final/src/SUBSEA-ROV-2026/src/models/uissubsea_rov/model.sdf` — add thruster links/joints/plugins and underwater dynamics; remove conflicting velocity control.
- `/home/yahya/subsea_final/src/SUBSEA-ROV-2026/src/models/simple_rov.sdf` — plugin style reference (`ApplyForce`/`VelocityControl` usage pattern).
- `/home/yahya/subsea_final/src/SUBSEA-WORLD-2026/src/worlds/hydrodynamics_world.sdf` — world entrypoint for launch and runtime validation.
- `/home/yahya/subsea_final/src/subsea_rov_control/package.xml` — dependencies and metadata for control package.
- `/home/yahya/subsea_final/src/subsea_rov_control/CMakeLists.txt` — install rules for scripts, launch, and config.
- `/home/yahya/subsea_final/src/subsea_rov_control/subsea_rov_control/thruster_mixer_node.py` — input arbitration, mixing, safety, and output publication.
- `/home/yahya/subsea_final/src/subsea_rov_control/config/thruster_allocation.yaml` — allocation matrix and all tuning parameters.
- `/home/yahya/subsea_final/src/subsea_rov_control/config/bridge.yaml` — per-thruster ROS↔Gazebo bridge declarations.
- `/home/yahya/subsea_final/src/subsea_rov_control/launch/rov_gz_control.launch.py` — one-command startup path.
- `/home/yahya/subsea_final/src/SUBSEA-ROV-2026/package.xml` — optional dependency updates if launch assets remain here.
- `/home/yahya/subsea_final/src/SUBSEA-ROV-2026/CMakeLists.txt` — optional install targets if ownership remains here.

**Acceptance Criteria**
1. ROV spawns with 8 thruster joints and associated thruster plugins active.
2. ROS direct 8-channel command can drive each thruster independently through bridge.
3. ROS high-level `Wrench` and `Twist` commands both produce stable thruster outputs.
4. Safety timeout guarantees zero-thrust fallback.
5. Underwater dynamics produce bounded, physically plausible motion.
6. End-to-end startup is reproducible via one launch command.

**Risks and Mitigations**
- Risk: axis/sign mismatches in early SDF can invert controls. Mitigation: lock conventions first and run single-thruster sign tests.
- Risk: poor initial hydrodynamic coefficients cause unrealistic acceleration/oscillation. Mitigation: start conservative and tune incrementally.
- Risk: bridge miswiring across 8 topics increases debugging cost. Mitigation: deterministic naming and explicit mapping table in config.
- Risk: mixed input modes conflict. Mitigation: explicit arbitration priority with freshness timeout.

**Decisions (Confirmed)**
- Layout: BlueROV2-style 8-thruster arrangement.
- Interfaces: direct 8-channel + high-level `Wrench` + high-level `Twist`.
- Hydrodynamics: in scope now.
- Control package: new dedicated package.
- Mixer language: Python (`rclpy`).

**Out-of-Scope for this iteration**
- Closed-loop autopilot controllers (depth/heading hold PID loops tied to sensors).
- Real hardware ESC/thruster calibration and actuator latency models.
- Full CFD-grade hydrodynamic parameter identification.
