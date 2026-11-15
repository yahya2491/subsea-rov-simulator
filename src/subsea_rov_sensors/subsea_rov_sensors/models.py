"""Measurement models for sensors Gazebo does not provide.

Both models take the vehicle's true pose (body origin position + body->world
rotation) and produce what the real instrument would report, including its
mount offset on the vehicle.

PressureSensorModel
    Hydrostatic pressure at the sensor port: p_atm + rho * g * depth, with
    white noise and a constant offset (calibration error). Depth is measured
    from the surface plane z = surface_z.

UsblModel
    Ultra-short-baseline acoustic positioning. A transceiver fixed at a known
    world position (a vessel or pier) measures slant range and two angles to
    the vehicle's transponder. Noise is applied in that native spherical
    space (range noise grows with range, angle noise is constant), which is
    what gives USBL its characteristic error that grows with distance. Fixes
    can drop out or be outliers (multipath), and each one is delivered late:
    acoustic travel time plus processing delay.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np


@dataclass
class PressureSensorModel:
    surface_z: float = 10.0
    water_density: float = 997.0
    gravity: float = 9.81
    atmospheric_pressure: float = 101325.0
    noise_std: float = 50.0          # Pa  (about 5 mm of water)
    offset: float = 0.0              # Pa
    mount: Sequence[float] = (0.0, 0.0, 0.0)

    def true_depth(self, position, rotation: np.ndarray) -> float:
        z = position[2] + (np.asarray(rotation) @ np.asarray(self.mount))[2]
        return self.surface_z - z

    def measure(self, position, rotation: np.ndarray, rng: np.random.Generator) -> float:
        depth = max(0.0, self.true_depth(position, rotation))
        p = self.atmospheric_pressure + self.water_density * self.gravity * depth
        return p + self.offset + rng.normal(0.0, self.noise_std)


@dataclass
class UsblFix:
    position: np.ndarray        # transponder position, world, m
    covariance: np.ndarray      # 3x3, world, m^2
    latency: float              # s from ping to delivery
    outlier: bool


@dataclass
class UsblModel:
    transceiver_position: Sequence[float] = (0.0, 0.0, 9.5)
    range_std: float = 0.03          # m, constant part
    range_std_per_m: float = 0.002   # m per m of slant range
    angle_std: float = math.radians(0.5)
    sound_speed: float = 1500.0
    processing_delay: float = 0.15
    max_range: float = 150.0
    dropout_probability: float = 0.05
    outlier_probability: float = 0.02
    outlier_std: float = 1.5         # m, size of a multipath outlier
    mount: Sequence[float] = (0.0, 0.0, 0.0)

    def measure(self, position, rotation: np.ndarray,
                rng: np.random.Generator) -> Optional[UsblFix]:
        tx = np.asarray(self.transceiver_position, dtype=float)
        target = np.asarray(position, dtype=float) + np.asarray(rotation) @ np.asarray(self.mount)
        rel = target - tx
        rng_true = float(np.linalg.norm(rel))
        if rng_true > self.max_range or rng_true < 1e-6:
            return None
        if rng.random() < self.dropout_probability:
            return None

        az = math.atan2(rel[1], rel[0])
        el = math.asin(rel[2] / rng_true)
        sr = self.range_std + self.range_std_per_m * rng_true
        r_m = rng_true + rng.normal(0.0, sr)
        az_m = az + rng.normal(0.0, self.angle_std)
        el_m = el + rng.normal(0.0, self.angle_std)
        meas = tx + r_m * np.array([math.cos(el_m) * math.cos(az_m),
                                    math.cos(el_m) * math.sin(az_m),
                                    math.sin(el_m)])
        outlier = rng.random() < self.outlier_probability
        if outlier:
            meas = meas + rng.normal(0.0, self.outlier_std, 3)

        # First-order covariance: J diag(sr^2, sa^2, sa^2) J^T.
        ce, se, ca, sa = math.cos(el), math.sin(el), math.cos(az), math.sin(az)
        J = np.array([
            [ce * ca, -rng_true * ce * sa, -rng_true * se * ca],
            [ce * sa, rng_true * ce * ca, -rng_true * se * sa],
            [se, 0.0, rng_true * ce],
        ])
        cov = J @ np.diag([sr ** 2, self.angle_std ** 2, self.angle_std ** 2]) @ J.T
        latency = rng_true / self.sound_speed + self.processing_delay
        return UsblFix(position=meas, covariance=cov, latency=latency, outlier=outlier)
