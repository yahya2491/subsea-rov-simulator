"""Lightweight 6-DOF model of dummy_rov for offline controller tests.

Mirrors what the Gazebo model does: rigid body, buoyancy acting at the
centre of the displaced volume, linear damping on the velocity relative to
the water (so an ocean current is a drag disturbance), thrust through the
same allocation geometry. It has no added mass or quadratic drag, because
the Gazebo model has neither. That makes it a close stand-in for the
simulator, good enough to catch sign errors, instability and windup in
unit tests without launching Gazebo.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

GRAVITY = 9.81


@dataclass
class PlantParams:
    # base_link 34.2 kg + 8 thruster links x 0.05 kg
    mass: float = 34.6
    inertia: np.ndarray = field(default_factory=lambda: np.array([1.974, 2.886, 3.705]))
    # Displaced volume 0.036191 m^3 in water of 997 kg/m^3 (see model.sdf)
    buoyancy: float = 997.0 * 0.036191 * GRAVITY
    # Centre of buoyancy relative to CoM: hull at z=0, floats at z=+0.15
    cob: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0375]))
    # Linear damping, body frame (positive numbers, applied as -d*v)
    lin_damping: np.ndarray = field(default_factory=lambda: np.array([40.0, 40.0, 80.0]))
    ang_damping: np.ndarray = field(default_factory=lambda: np.array([11.0, 9.0, 9.0]))


class RovPlant:
    def __init__(self, params: PlantParams = None, position=(0.0, 0.0, 5.0), yaw: float = 0.0):
        self.p = params or PlantParams()
        self.position = np.array(position, dtype=float)
        self.rot = Rotation.from_euler('z', yaw)
        self.vel_body = np.zeros(3)
        self.omega = np.zeros(3)
        self.current_world = np.zeros(3)
        self.force_world = np.zeros(3)   # external disturbance at CoM

    @property
    def velocity_world(self) -> np.ndarray:
        return self.rot.apply(self.vel_body)

    def step(self, body_wrench, dt: float) -> None:
        p = self.p
        r_inv = self.rot.inv()
        tau = np.asarray(body_wrench, dtype=float)

        weight_b = r_inv.apply([0.0, 0.0, -p.mass * GRAVITY])
        buoy_b = r_inv.apply([0.0, 0.0, p.buoyancy])
        rel_vel = self.vel_body - r_inv.apply(self.current_world)
        damp_f = -p.lin_damping * rel_vel
        dist_b = r_inv.apply(self.force_world)

        force = tau[:3] + weight_b + buoy_b + damp_f + dist_b
        acc = force / p.mass - np.cross(self.omega, self.vel_body)

        moment = tau[3:] + np.cross(p.cob, buoy_b) - p.ang_damping * self.omega
        inertia = p.inertia
        alpha = (moment - np.cross(self.omega, inertia * self.omega)) / inertia

        # Semi-implicit Euler: velocities first, then pose with new velocities.
        self.vel_body = self.vel_body + acc * dt
        self.omega = self.omega + alpha * dt
        self.position = self.position + self.rot.apply(self.vel_body) * dt
        self.rot = self.rot * Rotation.from_rotvec(self.omega * dt)
