import pytest

from subsea_rov_control.hover import pole_placement_gains, wrap_angle
from subsea_rov_control.pid import Pid, PidGains


def test_proportional_and_derivative_on_measurement():
    pid = Pid(PidGains(kp=2.0, kd=0.5))
    # Derivative acts on the measured rate, not the error.
    assert pid.update(error=1.0, rate=0.4, dt=0.1) == pytest.approx(2.0 - 0.2)


def test_integral_accumulates_and_is_clamped():
    pid = Pid(PidGains(ki=1.0, i_limit=0.3))
    for _ in range(100):
        out = pid.update(error=1.0, rate=0.0, dt=0.01)
    assert out == pytest.approx(0.3)


def test_no_windup_while_output_is_saturated():
    pid = Pid(PidGains(kp=10.0, ki=1.0, out_limit=1.0))
    for _ in range(1000):
        assert pid.update(error=1.0, rate=0.0, dt=0.01) == pytest.approx(1.0)
    assert pid.saturated
    assert pid.integral == pytest.approx(0.0)
    # Error reverses: the output responds at once instead of unwinding first.
    assert pid.update(error=-0.05, rate=0.0, dt=0.01) < 0.0


def test_feedforward_is_added():
    pid = Pid(PidGains(kp=1.0))
    assert pid.update(error=0.0, rate=0.0, dt=0.01, feedforward=-14.6) == pytest.approx(-14.6)


def test_pole_placement_gives_a_triple_pole():
    m, d, w = 34.6, 40.0, 0.8
    kp, ki, kd = pole_placement_gains(m, d, w)
    # m s^3 + (d + kd) s^2 + kp s + ki == m (s + w)^3
    assert (d + kd) / m == pytest.approx(3 * w)
    assert kp / m == pytest.approx(3 * w ** 2)
    assert ki / m == pytest.approx(w ** 3)


TWO_PI = 6.283185307179586


@pytest.mark.parametrize('a, expected', [(0.0, 0.0), (3.5, 3.5 - TWO_PI),
                                         (-3.5, -3.5 + TWO_PI), (6.0, 6.0 - TWO_PI)])
def test_wrap_angle(a, expected):
    assert wrap_angle(a) == pytest.approx(expected)
