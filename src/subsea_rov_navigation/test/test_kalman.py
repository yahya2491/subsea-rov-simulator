import numpy as np
import pytest

from subsea_rov_navigation.kalman import PositionVelocityKF


def simulate(kf, seconds, velocity, rng, usbl_sigma=0.1, outlier_every=None, delay=0.0):
    """Constant-velocity truth, DVL at 8 Hz, depth at 20 Hz, USBL at 1 Hz."""
    dt = 0.01
    p = np.array([0.0, 0.0, 5.0])
    v = np.asarray(velocity, dtype=float)
    pending = []
    errors = []
    for k in range(1, int(seconds / dt) + 1):
        t = k * dt
        p = p + v * dt
        kf.predict(t)
        if k % 12 == 0:
            kf.update_body_velocity(v + rng.normal(0, 0.005, 3), np.eye(3), np.eye(3) * 0.005 ** 2)
        if k % 5 == 0:
            kf.update_depth(p[2] + rng.normal(0, 0.005), 0.005 ** 2)
        if k % 100 == 0:
            fix = p + rng.normal(0, usbl_sigma, 3)
            if outlier_every and (k // 100) % outlier_every == 0:
                fix = fix + np.array([3.0, -3.0, 0.0])
            pending.append((t + delay, t, fix))
        while pending and pending[0][0] <= t + 1e-9:
            _, stamp, fix = pending.pop(0)
            kf.update_position(fix, np.eye(3) * usbl_sigma ** 2, stamp=stamp if delay else None)
        errors.append(np.linalg.norm(kf.x[:3] - p))
    return np.array(errors), p


def make(p0=(0.5, -0.5, 5.0)):
    kf = PositionVelocityKF(accel_noise=0.05)
    kf.initialise(p0, 0.0, sigma_position=(1.0, 1.0, 1.0))
    return kf


def test_tracks_moving_vehicle_to_well_below_fix_noise():
    rng = np.random.default_rng(0)
    kf = make()
    err, _ = simulate(kf, 60, [0.2, -0.1, 0.0], rng)
    steady = err[2000:]
    assert np.sqrt(np.mean(steady ** 2)) < 0.05     # USBL alone is 0.17 m 3D RMS
    assert np.linalg.norm(kf.x[3:] - [0.2, -0.1, 0.0]) < 0.01


def test_outliers_are_rejected():
    rng = np.random.default_rng(2)
    kf = make()
    err, _ = simulate(kf, 60, [0.1, 0.0, 0.0], rng, outlier_every=7)
    assert kf.stats['usbl'][1] >= 7
    assert err[2000:].max() < 0.2


def test_delayed_fixes_are_carried_forward():
    rng = np.random.default_rng(3)
    lagged = make()
    err_comp, _ = simulate(lagged, 60, [0.5, 0.0, 0.0], rng, delay=0.4)
    rng = np.random.default_rng(3)
    naive = make()
    # Same delay, but applied as if it were current.
    err_naive, _ = simulate_naive(naive, 60, [0.5, 0.0, 0.0], rng, delay=0.4)
    assert np.sqrt(np.mean(err_comp[2000:] ** 2)) < 0.5 * np.sqrt(np.mean(err_naive[2000:] ** 2))


def simulate_naive(kf, seconds, velocity, rng, delay):
    original = kf.update_position
    kf.update_position = lambda pos, cov, stamp=None: original(pos, cov, None)
    return simulate(kf, seconds, velocity, rng, delay=delay)


def test_recovers_after_consecutive_rejections():
    kf = PositionVelocityKF(max_consecutive_rejects=3)
    kf.initialise([0, 0, 5], 0.0, sigma_position=(0.01, 0.01, 0.01))
    kf.P[3:, 3:] = np.eye(3) * 1e-6
    # The vehicle is really 5 m away; a confident filter rejects that at first.
    truth = np.array([5.0, 0.0, 5.0])
    accepted = []
    for k in range(1, 6):
        kf.predict(float(k))
        accepted.append(kf.update_position(truth, np.eye(3) * 0.01).accepted)
    assert accepted[:3] == [False, False, False]
    assert accepted[3] is True
    assert kf.x[0] > 4.0


def test_position_history_interpolates():
    kf = PositionVelocityKF()
    kf.initialise([0, 0, 0], 0.0, sigma_position=(1, 1, 1))
    kf.x[3] = 1.0
    for k in range(1, 11):
        kf.predict(k * 0.1)
    assert kf.position_at(0.55)[0] == pytest.approx(0.55, abs=1e-9)
