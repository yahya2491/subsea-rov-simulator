import math

import numpy as np
import pytest

from subsea_rov_navigation.ahrs import MahonyAhrs, quat_from_euler, quat_to_euler, quat_to_matrix

FIELD = np.array([0.41e-6, 15.6e-6, -48.1e-6])
G = 9.81


def readings(roll, pitch, yaw):
    """Noise-free accelerometer and magnetometer for a static attitude."""
    rot = quat_to_matrix(quat_from_euler(roll, pitch, yaw))
    return rot.T @ np.array([0, 0, G]), rot.T @ FIELD


def angle_err(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def test_euler_roundtrip():
    for rpy in [(0.1, -0.2, 2.5), (-0.4, 0.3, -1.0), (0, 0, math.pi - 0.01)]:
        assert np.allclose(quat_to_euler(quat_from_euler(*rpy)), rpy, atol=1e-9)


@pytest.mark.parametrize('rpy', [(0, 0, 0), (0.2, -0.1, 1.3), (-0.3, 0.25, -2.8),
                                 (0, 0, math.pi / 2)])
def test_static_alignment(rpy):
    ahrs = MahonyAhrs(FIELD)
    ahrs.initialise(*readings(*rpy))
    est = ahrs.euler()
    for e, t in zip(est, rpy):
        assert angle_err(e, t) < 1e-9


def test_converges_and_estimates_gyro_bias_with_noise():
    rng = np.random.default_rng(1)
    ahrs = MahonyAhrs(FIELD, kp_acc=0.5, kp_mag=0.5, ki=0.05)
    true = (0.05, -0.04, 0.7)
    acc, mag = readings(*true)
    ahrs.initialise(*readings(0.0, 0.0, 0.0))  # deliberately wrong start
    bias = np.array([0.004, -0.003, 0.005])
    dt = 0.01
    for _ in range(int(120 / dt)):
        gyro = bias + rng.normal(0, 2e-3, 3)
        ahrs.update(gyro, acc + rng.normal(0, 0.02, 3), mag + rng.normal(0, 3e-7, 3), dt)
    est = ahrs.euler()
    for e, t in zip(est, true):
        assert angle_err(e, t) < math.radians(0.5)
    np.testing.assert_allclose(ahrs.gyro_bias, bias, atol=1e-3)


def test_magnetometer_cannot_tilt_the_estimate():
    ahrs = MahonyAhrs(FIELD)
    acc, _ = readings(0.0, 0.0, 0.0)
    ahrs.initialise(*readings(0.0, 0.0, 0.0))
    # A badly disturbed field (e.g. a nearby steel structure).
    bad_mag = np.array([30e-6, -5e-6, 10e-6])
    for _ in range(3000):
        ahrs.update(np.zeros(3), acc, bad_mag, 0.01)
    roll, pitch, _ = ahrs.euler()
    assert abs(roll) < 1e-6 and abs(pitch) < 1e-6


def test_accelerometer_is_gated_during_strong_acceleration():
    ahrs = MahonyAhrs(FIELD, acc_gate=0.3)
    acc, mag = readings(0.0, 0.0, 0.0)
    ahrs.initialise(acc, mag)
    heaved = acc + np.array([0.3, 0.0, 1.0])    # |f| = 10.8, outside the gate
    for _ in range(200):
        ahrs.update(np.zeros(3), heaved, mag, 0.01)
    assert not ahrs.acc_used
    _, pitch, _ = ahrs.euler()
    assert abs(pitch) < 1e-9


def test_known_linear_acceleration_is_removed():
    # A horizontal shove barely changes |f| (10.01 vs 9.81), so the gate alone
    # would let it through and the filter would read it as ~11 deg of pitch.
    acc, mag = readings(0.0, 0.0, 0.0)
    shove = np.array([2.0, 0.0, 0.0])
    naive = MahonyAhrs(FIELD)
    aided = MahonyAhrs(FIELD)
    for ahrs in (naive, aided):
        ahrs.initialise(acc, mag)
    for _ in range(300):
        naive.update(np.zeros(3), acc + shove, mag, 0.01)
        aided.update(np.zeros(3), acc + shove, mag, 0.01, linear_accel_body=shove)
    assert abs(naive.euler()[1]) > math.radians(3.0)
    assert abs(aided.euler()[1]) < 1e-9


def test_gyro_integration_tracks_a_turn():
    ahrs = MahonyAhrs(FIELD, kp_acc=0.0, kp_mag=0.0, ki=0.0)
    ahrs.initialise(*readings(0.0, 0.0, 0.0))
    for _ in range(100):
        ahrs.update([0.0, 0.0, 0.5], [0, 0, G], None, 0.01)   # 0.5 rad/s for 1 s
    assert ahrs.euler()[2] == pytest.approx(0.5, abs=1e-9)
