"""Position / velocity Kalman filter for an underwater vehicle.

State x = [p (3), v (3)], world ENU. Attitude comes from the AHRS and is
treated as known, which makes every measurement model below linear in x,
so this is an exact (time-varying) Kalman filter rather than an EKF.

    predict   p += v dt + a dt^2/2,  v += a dt
              a = R f_imu + g  (IMU specific force), or a = 0 if disabled
    DVL       R^T v = v_dvl - w x r_dvl            (body-frame velocity)
    depth     p_z   = z_sensor - (R r_p)_z          (pressure sensor)
    USBL      p     = p_transponder - R r_u         (acoustic fix)

Robustness
* Every update is gated on its normalised innovation squared (chi-square
  test), so acoustic outliers and DVL glitches are rejected.
* After ``max_consecutive_rejects`` USBL fixes in a row fail the gate, the
  next fix is accepted anyway. A filter that has drifted would otherwise
  reject every correct fix forever.
* USBL fixes arrive late (acoustic travel time + processing). The fix is
  carried forward to the present using the estimated displacement since its
  timestamp, from a short state history, before it is applied.
"""

from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np

# chi-square 99.9 % quantiles
CHI2_999 = {1: 10.83, 3: 16.27}


@dataclass
class UpdateResult:
    accepted: bool
    nis: float


class PositionVelocityKF:
    def __init__(self, accel_noise: float = 0.1, history_seconds: float = 5.0,
                 max_consecutive_rejects: int = 5):
        self.accel_noise = accel_noise
        self.history_seconds = history_seconds
        self.max_rejects = max_consecutive_rejects
        self.x = np.zeros(6)
        self.P = np.eye(6)
        self.t: Optional[float] = None
        self.history = deque()
        self.usbl_rejects = 0
        self.stats = {'dvl': [0, 0], 'depth': [0, 0], 'usbl': [0, 0]}  # [accepted, rejected]

    @property
    def initialised(self) -> bool:
        return self.t is not None

    def initialise(self, position, t: float, sigma_position=(1.0, 1.0, 1.0),
                   sigma_velocity: float = 0.3) -> None:
        self.x = np.concatenate([np.asarray(position, dtype=float), np.zeros(3)])
        self.P = np.diag(np.concatenate([np.square(sigma_position), [sigma_velocity ** 2] * 3]))
        self.t = t
        self.history.clear()
        self.history.append((t, self.x[:3].copy()))

    def predict(self, t: float, accel_world=None) -> None:
        dt = t - self.t
        if dt <= 0.0:
            return
        a = np.zeros(3) if accel_world is None else np.asarray(accel_world, dtype=float)
        p, v = self.x[:3], self.x[3:]
        self.x = np.concatenate([p + v * dt + 0.5 * a * dt * dt, v + a * dt])
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        G = np.vstack([np.eye(3) * 0.5 * dt * dt, np.eye(3) * dt])
        self.P = F @ self.P @ F.T + (self.accel_noise ** 2) * (G @ G.T)
        self.t = t
        self.history.append((t, self.x[:3].copy()))
        while self.history and self.history[0][0] < t - self.history_seconds:
            self.history.popleft()

    def _update(self, z, H, R, kind: str, gate: bool = True) -> UpdateResult:
        z = np.atleast_1d(np.asarray(z, dtype=float))
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        S_inv = np.linalg.inv(S)
        nis = float(y @ S_inv @ y)
        if gate and nis > CHI2_999[len(z)]:
            self.stats[kind][1] += 1
            return UpdateResult(False, nis)
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        # Joseph form keeps P symmetric positive definite.
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.stats[kind][0] += 1
        return UpdateResult(True, nis)

    def update_body_velocity(self, v_body, rot_body_to_world, cov) -> UpdateResult:
        H = np.zeros((3, 6))
        H[:, 3:] = np.asarray(rot_body_to_world).T
        return self._update(v_body, H, np.asarray(cov), 'dvl')

    def update_depth(self, z_position: float, variance: float) -> UpdateResult:
        H = np.zeros((1, 6))
        H[0, 2] = 1.0
        return self._update([z_position], H, np.array([[variance]]), 'depth')

    def position_at(self, t: float) -> np.ndarray:
        """Estimated position at a past time (linear interpolation)."""
        if not self.history or t >= self.history[-1][0]:
            return self.x[:3].copy()
        if t <= self.history[0][0]:
            return self.history[0][1].copy()
        prev = self.history[0]
        for item in self.history:
            if item[0] >= t:
                span = item[0] - prev[0]
                w = 0.0 if span <= 0 else (t - prev[0]) / span
                return (1 - w) * prev[1] + w * item[1]
            prev = item
        return self.x[:3].copy()

    def update_position(self, position, cov, stamp: Optional[float] = None) -> UpdateResult:
        z = np.asarray(position, dtype=float)
        if stamp is not None and self.t is not None and stamp < self.t:
            z = z + (self.x[:3] - self.position_at(stamp))
        H = np.zeros((3, 6))
        H[:, :3] = np.eye(3)
        force = self.usbl_rejects >= self.max_rejects
        result = self._update(z, H, np.asarray(cov), 'usbl', gate=not force)
        self.usbl_rejects = 0 if result.accepted else self.usbl_rejects + 1
        return result
