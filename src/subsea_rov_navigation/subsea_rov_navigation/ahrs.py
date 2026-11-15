"""Attitude and heading reference: Mahony complementary filter.

Fuses gyro (propagation), accelerometer (roll/pitch reference) and
magnetometer (heading reference) into a body->world quaternion, and
estimates the gyro bias on line.

Conventions: world ENU, body FLU. The accelerometer measures specific force,
so a vehicle at rest reads +g along body "up". The magnetometer reads the
world field rotated into the body frame.

Two design choices matter underwater:
* The magnetometer only corrects heading. Its error is projected onto the
  world vertical before being applied, so a disturbed or poorly calibrated
  field can never tilt the roll/pitch estimate.
* Linear acceleration must not be mistaken for tilt. When the caller can
  supply the vehicle's own acceleration (the navigation node derives it
  from DVL velocity), it is removed from the specific force before the
  gravity direction is extracted. As a second line of defence the
  accelerometer correction is gated off whenever the remaining specific
  force departs from g by more than ``acc_gate``.
"""

import math
from typing import Optional, Sequence

import numpy as np


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, (w, x, y, z) order."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Rotation matrix (body -> world) for a (w, x, y, z) unit quaternion."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """(w, x, y, z) for ZYX Euler angles."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_euler(q: np.ndarray):
    """(roll, pitch, yaw), ZYX convention, from (w, x, y, z)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class MahonyAhrs:
    def __init__(self, mag_reference: Sequence[float], kp_acc: float = 0.5,
                 kp_mag: float = 0.5, ki: float = 0.02, gravity: float = 9.81,
                 acc_gate: float = 0.3, bias_limit: float = 0.05):
        ref = np.asarray(mag_reference, dtype=float)
        horiz = np.hypot(ref[0], ref[1])
        if horiz < 1e-9:
            raise ValueError('magnetic reference has no horizontal component')
        self.mag_ref_h = ref[:2] / horiz
        self.kp_acc = kp_acc
        self.kp_mag = kp_mag
        self.ki = ki
        self.gravity = gravity
        self.acc_gate = acc_gate
        self.bias_limit = bias_limit
        self.q: Optional[np.ndarray] = None
        self.gyro_bias = np.zeros(3)
        self.acc_used = False

    @property
    def initialised(self) -> bool:
        return self.q is not None

    def initialise(self, acc: Sequence[float], mag: Sequence[float]) -> None:
        """Align from rest: tilt from gravity, heading from the magnetic field."""
        f = np.asarray(acc, dtype=float)
        roll = math.atan2(f[1], f[2])
        pitch = math.atan2(-f[0], math.hypot(f[1], f[2]))
        level = quat_to_matrix(quat_from_euler(roll, pitch, 0.0))
        m_level = level @ np.asarray(mag, dtype=float)
        yaw = math.atan2(self.mag_ref_h[1], self.mag_ref_h[0]) - \
            math.atan2(m_level[1], m_level[0])
        self.q = quat_from_euler(roll, pitch, yaw)
        self.gyro_bias = np.zeros(3)

    def update(self, gyro: Sequence[float], acc: Sequence[float],
               mag: Optional[Sequence[float]], dt: float,
               linear_accel_body: Optional[Sequence[float]] = None) -> np.ndarray:
        """Advance by dt and return the bias-corrected body rate.

        ``linear_accel_body`` is the vehicle's inertial acceleration in the
        body frame, if known; it is subtracted from the specific force.
        """
        if self.q is None:
            raise RuntimeError('initialise() first')
        w = np.asarray(gyro, dtype=float)
        rot = quat_to_matrix(self.q)

        err_acc = np.zeros(3)
        f = np.asarray(acc, dtype=float)
        if linear_accel_body is not None:
            f = f - np.asarray(linear_accel_body, dtype=float)
        f_norm = np.linalg.norm(f)
        self.acc_used = abs(f_norm - self.gravity) < self.acc_gate
        if self.acc_used:
            up_est = rot.T @ np.array([0.0, 0.0, 1.0])
            err_acc = np.cross(f / f_norm, up_est)

        err_mag = np.zeros(3)
        if mag is not None:
            m_world = rot @ np.asarray(mag, dtype=float)
            h = np.hypot(m_world[0], m_world[1])
            if h > 1e-12:
                h_est = m_world[:2] / h
                # Heading error about world z, mapped back into the body frame.
                ez = h_est[0] * self.mag_ref_h[1] - h_est[1] * self.mag_ref_h[0]
                err_mag = rot.T @ np.array([0.0, 0.0, ez])

        if dt > 0.0:
            self.gyro_bias -= self.ki * (err_acc + err_mag) * dt
            self.gyro_bias = np.clip(self.gyro_bias, -self.bias_limit, self.bias_limit)

        rate = w - self.gyro_bias
        corrected = rate + self.kp_acc * err_acc + self.kp_mag * err_mag
        angle = np.linalg.norm(corrected) * dt
        if angle > 0.0:
            axis = corrected / np.linalg.norm(corrected)
            dq = np.concatenate([[math.cos(angle / 2)], math.sin(angle / 2) * axis])
            self.q = quat_multiply(self.q, dq)
            self.q /= np.linalg.norm(self.q)
        return rate

    def euler(self):
        return quat_to_euler(self.q)
