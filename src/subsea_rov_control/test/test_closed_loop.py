"""Closed-loop tests: hover controller + allocation on the offline 6-DOF plant.

These use true state (no sensor noise), so they isolate the control design:
stability, sign conventions, integral action against a steady current, and
recovery from an impulse. Sensor-driven validation runs in Gazebo, see
subsea_rov_bringup (run_validation).
"""

import math

from conftest import hover_config
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from subsea_rov_control.hover import HoverController, HoverSetpoint, VehicleState
from subsea_rov_control.plant import RovPlant

CTRL_DT = 0.02      # 50 Hz, as in the node
PLANT_SUBSTEPS = 10


def run(plant, ctrl, allocator, setpoint, seconds, disturbance=None):
    """Simulate; return arrays of (t, position, yaw, roll, pitch, thrust)."""
    log = []
    steps = int(seconds / CTRL_DT)
    for k in range(steps):
        t = k * CTRL_DT
        if disturbance:
            disturbance(t, plant)
        state = VehicleState(
            position=plant.position.copy(),
            orientation=plant.rot.as_quat(),
            velocity_world=plant.velocity_world,
            angular_velocity_body=plant.omega.copy())
        wrench = ctrl.update(state, setpoint, CTRL_DT)
        thrust, _ = allocator.allocate(wrench)
        applied = allocator.wrench_from_thrust(thrust)
        for _ in range(PLANT_SUBSTEPS):
            plant.step(applied, CTRL_DT / PLANT_SUBSTEPS)
        yaw, pitch, roll = plant.rot.as_euler('ZYX')
        log.append((t, *plant.position, yaw, roll, pitch, np.max(np.abs(thrust))))
    return np.array(log)


def make(position=(0, 0, 5), yaw=0.0):
    return RovPlant(position=position, yaw=yaw), HoverController(hover_config())


def test_converges_to_setpoint_from_offset(allocator):
    plant, ctrl = make(position=(0.6, -0.4, 4.6), yaw=math.radians(40))
    sp = HoverSetpoint(position=np.array([0.0, 0.0, 5.0]), yaw=0.0)
    log = run(plant, ctrl, allocator, sp, 30.0)
    final = log[-1]
    assert np.hypot(final[1], final[2]) < 0.02
    assert abs(final[3] - 5.0) < 0.02
    assert abs(final[4]) < math.radians(1.0)
    # Stays level while doing it.
    assert np.max(np.abs(log[:, 5:7])) < math.radians(5.0)


def test_rejects_steady_current_with_integral_action(allocator):
    plant, ctrl = make()
    sp = HoverSetpoint(position=np.array([0.0, 0.0, 5.0]), yaw=0.0)
    run(plant, ctrl, allocator, sp, 10.0)          # settle first

    def current(t, p):
        p.current_world = np.array([0.25, 0.15, 0.0])
    log = run(plant, ctrl, allocator, sp, 40.0, current)
    horiz = np.hypot(log[:, 1], log[:, 2])
    assert horiz.max() < 0.5
    assert horiz[-50:].max() < 0.03                # last second: back on station


def test_recovers_from_impulse(allocator):
    plant, ctrl = make()
    sp = HoverSetpoint(position=np.array([0.0, 0.0, 5.0]), yaw=0.0)
    run(plant, ctrl, allocator, sp, 10.0)

    def push(t, p):
        p.force_world = np.array([0.0, 40.0, 0.0]) if t < 1.0 else np.zeros(3)
    log = run(plant, ctrl, allocator, sp, 20.0, push)
    horiz = np.hypot(log[:, 1], log[:, 2])
    assert horiz.max() < 0.4
    assert horiz[-50:].max() < 0.03


@pytest.mark.parametrize('yaw_sp_deg', [90.0, -150.0])
def test_heading_change_takes_short_way_round(allocator, yaw_sp_deg):
    plant, ctrl = make(yaw=math.radians(170.0))
    sp = HoverSetpoint(position=np.array([0.0, 0.0, 5.0]), yaw=math.radians(yaw_sp_deg))
    log = run(plant, ctrl, allocator, sp, 20.0)
    final_yaw = log[-1, 4]
    err = (math.radians(yaw_sp_deg) - final_yaw + math.pi) % (2 * math.pi) - math.pi
    assert abs(err) < math.radians(1.0)
    # Position held while turning.
    assert np.max(np.hypot(log[:, 1], log[:, 2])) < 0.15


def test_plant_free_ascent_matches_gazebo():
    """Sanity check on the plant: Gazebo measured 0.182 m/s free ascent."""
    plant = RovPlant()
    for _ in range(5000):
        plant.step(np.zeros(6), 0.002)
    assert plant.velocity_world[2] == pytest.approx(0.182, abs=0.005)
    assert Rotation.from_quat(plant.rot.as_quat()).magnitude() < 1e-6
