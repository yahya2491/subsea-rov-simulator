"""Closed-loop hover (station-keeping) controller.

Holds a world-frame position and heading while keeping the vehicle level.

Structure
---------
* Position (x, y, z) is controlled in the world frame. Disturbances that
  matter for station keeping, such as a steady current or the vehicle's
  own net buoyancy, are fixed in the world, so world-frame integrators
  stay valid while the vehicle yaws.
* The world-frame force demand is rotated into the body frame through the
  full attitude, so a small roll or pitch does not bleed heave into surge.
* Heading is a PID on the wrapped yaw error. Roll and pitch are regulated
  to zero. The frame is passively stable (centre of buoyancy above the
  centre of mass), so those loops only add damping and stiffness.
* The known net buoyancy enters as a feedforward on world z. The z
  integrator only has to absorb the residual model error, which shortens
  the transient after the controller is engaged.

All gains are tuned by pole placement on the linearised per-axis model
m*a + d*v = F. Placing the closed-loop poles of the PID loop at a triple
real pole -w gives

    kp = 3*m*w^2,   ki = m*w^3,   kd = 3*m*w - d

See ``config/control.yaml`` in subsea_rov_bringup for the numbers.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from .pid import Pid, PidGains

AXES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pole_placement_gains(mass: float, damping: float, omega: float) -> Tuple[float, float, float]:
    """PID gains placing a triple pole at -omega for m*a + d*v = F."""
    kp = 3.0 * mass * omega ** 2
    ki = mass * omega ** 3
    kd = max(0.0, 3.0 * mass * omega - damping)
    return kp, ki, kd


@dataclass
class HoverConfig:
    gains: Dict[str, PidGains] = field(default_factory=dict)
    # Feedforward force along world z (N). Negative = push down, which is
    # what a positively buoyant vehicle needs.
    buoyancy_feedforward: float = 0.0
    # Per-axis body wrench limit [Fx, Fy, Fz, Mx, My, Mz].
    wrench_limit: Sequence[float] = (math.inf,) * 6


@dataclass
class VehicleState:
    position: np.ndarray            # world, m
    orientation: np.ndarray         # quaternion (x, y, z, w), body -> world
    velocity_world: np.ndarray      # m/s
    angular_velocity_body: np.ndarray  # rad/s


@dataclass
class HoverSetpoint:
    position: np.ndarray            # world, m
    yaw: float                      # rad


class HoverController:
    def __init__(self, config: HoverConfig):
        missing = [a for a in AXES if a not in config.gains]
        if missing:
            raise ValueError(f'missing gains for axes: {missing}')
        self.config = config
        self.pids = {a: Pid(config.gains[a]) for a in AXES}
        self.limit = np.asarray(config.wrench_limit, dtype=float)
        self.last_errors = dict.fromkeys(AXES, 0.0)

    def reset(self) -> None:
        for pid in self.pids.values():
            pid.reset()

    def update(self, state: VehicleState, sp: HoverSetpoint, dt: float) -> np.ndarray:
        rot = Rotation.from_quat(state.orientation)
        yaw, pitch, roll = rot.as_euler('ZYX')
        p_rate, q_rate, r_rate = state.angular_velocity_body

        e_pos = np.asarray(sp.position, dtype=float) - state.position
        v = state.velocity_world
        force_world = np.array([
            self.pids['x'].update(e_pos[0], v[0], dt),
            self.pids['y'].update(e_pos[1], v[1], dt),
            self.pids['z'].update(e_pos[2], v[2], dt,
                                  feedforward=self.config.buoyancy_feedforward),
        ])
        force_body = rot.inv().apply(force_world)

        # Euler-angle rates from body rates (ZYX convention).
        cos_pitch = max(math.cos(pitch), 1e-3)
        roll_rate = p_rate + math.tan(pitch) * (q_rate * math.sin(roll) + r_rate * math.cos(roll))
        pitch_rate = q_rate * math.cos(roll) - r_rate * math.sin(roll)
        yaw_rate = (q_rate * math.sin(roll) + r_rate * math.cos(roll)) / cos_pitch

        e_yaw = wrap_angle(sp.yaw - yaw)
        torque = np.array([
            self.pids['roll'].update(-roll, roll_rate, dt),
            self.pids['pitch'].update(-pitch, pitch_rate, dt),
            self.pids['yaw'].update(e_yaw, yaw_rate, dt),
        ])

        self.last_errors = {
            'x': e_pos[0], 'y': e_pos[1], 'z': e_pos[2],
            'roll': -roll, 'pitch': -pitch, 'yaw': e_yaw,
        }
        wrench = np.concatenate([force_body, torque])
        return np.clip(wrench, -self.limit, self.limit)
