import math

import numpy as np
import pytest

from subsea_rov_sensors.models import PressureSensorModel, UsblModel


def rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def test_pressure_is_hydrostatic_at_the_sensor_port():
    m = PressureSensorModel(surface_z=10.0, noise_std=0.0, mount=(0.0, 0.0, 0.1))
    rng = np.random.default_rng(0)
    p = m.measure([0.0, 0.0, 5.0], np.eye(3), rng)
    depth = 10.0 - 5.1
    assert p == pytest.approx(101325.0 + 997.0 * 9.81 * depth)
    # Rolled upside down the port is 0.1 m below the origin instead of above.
    assert m.true_depth([0, 0, 5.0], rot_x(math.pi)) == pytest.approx(5.1)


def test_pressure_noise_statistics():
    m = PressureSensorModel(noise_std=50.0, offset=20.0)
    rng = np.random.default_rng(1)
    truth = m.atmospheric_pressure + m.water_density * m.gravity * 5.0
    samples = np.array([m.measure([0, 0, 5.0], np.eye(3), rng) for _ in range(20000)]) - truth
    assert samples.mean() == pytest.approx(20.0, abs=1.5)
    assert samples.std() == pytest.approx(50.0, rel=0.03)


def test_pressure_never_below_atmospheric():
    m = PressureSensorModel(noise_std=0.0)
    assert m.measure([0, 0, 12.0], np.eye(3), np.random.default_rng(0)) == pytest.approx(101325.0)


def clean_usbl(**kw):
    base = {'dropout_probability': 0.0, 'outlier_probability': 0.0}
    base.update(kw)
    return UsblModel(**base)


def test_usbl_noise_free_fix_is_exact_and_late():
    m = clean_usbl(range_std=0.0, range_std_per_m=0.0, angle_std=0.0,
                   transceiver_position=(0, 0, 9.5), mount=(0.0, 0.0, 0.25))
    fix = m.measure([3.0, -4.0, 5.0], np.eye(3), np.random.default_rng(0))
    np.testing.assert_allclose(fix.position, [3.0, -4.0, 5.25], atol=1e-12)
    slant = math.sqrt(9 + 16 + 4.25 ** 2)
    assert fix.latency == pytest.approx(slant / 1500.0 + 0.15)


def test_usbl_error_matches_reported_covariance():
    m = clean_usbl()
    rng = np.random.default_rng(2)
    pos = np.array([12.0, 5.0, 4.0])
    fixes = [m.measure(pos, np.eye(3), rng) for _ in range(4000)]
    err = np.array([f.position - pos for f in fixes])
    cov = fixes[0].covariance
    nis = np.einsum('ij,jk,ik->i', err, np.linalg.inv(cov), err)
    assert nis.mean() == pytest.approx(3.0, rel=0.1)   # chi-square, 3 dof


def test_usbl_error_grows_with_range():
    m = clean_usbl()
    near = m.measure([1.0, 0, 8.0], np.eye(3), np.random.default_rng(3)).covariance
    far = m.measure([60.0, 0, 8.0], np.eye(3), np.random.default_rng(3)).covariance
    assert np.trace(far) > 20 * np.trace(near)


def test_usbl_dropouts_and_range_limit():
    m = UsblModel(dropout_probability=0.3, outlier_probability=0.0, max_range=50.0)
    rng = np.random.default_rng(4)
    got = [m.measure([5, 0, 5], np.eye(3), rng) for _ in range(5000)]
    assert sum(f is None for f in got) / 5000 == pytest.approx(0.3, abs=0.02)
    assert m.measure([80, 0, 5], np.eye(3), rng) is None
