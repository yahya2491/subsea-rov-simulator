import numpy as np
import pytest

from subsea_rov_control.allocation import Thruster, ThrusterAllocator


def test_matrix_has_full_rank(allocator):
    assert np.linalg.matrix_rank(allocator.matrix) == 6


@pytest.mark.parametrize('axis', range(6))
def test_unsaturated_wrench_is_reproduced_exactly(allocator, axis):
    wrench = np.zeros(6)
    wrench[axis] = 5.0 if axis < 3 else 2.0
    thrust, scales = allocator.allocate(wrench)
    assert all(s == 1.0 for s in scales.values())
    np.testing.assert_allclose(allocator.wrench_from_thrust(thrust), wrench, atol=1e-9)


def test_signs_match_the_teleop_mixing_table(allocator):
    # Surge forward: all four horizontal thrusters positive and equal.
    thrust, _ = allocator.allocate([10, 0, 0, 0, 0, 0])
    assert np.all(thrust[:4] > 0)
    np.testing.assert_allclose(thrust[:4], thrust[0], rtol=1e-6)
    # Heave up: vertical thrusters positive.
    thrust, _ = allocator.allocate([0, 0, 10, 0, 0, 0])
    assert np.all(thrust[4:] > 0)
    # Yaw CCW: FR and BR positive, FL and BL negative.
    thrust, _ = allocator.allocate([0, 0, 0, 0, 0, 5])
    fl, fr, bl, br = thrust[:4]
    assert fr > 0 and br > 0 and fl < 0 and bl < 0


def test_saturation_scales_a_group_and_keeps_its_direction(allocator):
    wrench = np.array([200.0, 100.0, 5.0, 0.0, 0.0, 0.0])
    thrust, scales = allocator.allocate(wrench)
    assert np.max(np.abs(thrust)) <= 15.0 + 1e-9
    assert scales['horizontal'] < 1.0
    assert scales['vertical'] == pytest.approx(1.0)
    achieved = allocator.wrench_from_thrust(thrust)
    # Horizontal demand keeps its surge/sway ratio...
    assert achieved[1] / achieved[0] == pytest.approx(0.5, rel=1e-6)
    # ...and the vertical group still delivers the full heave demand.
    assert achieved[2] == pytest.approx(5.0, abs=1e-6)


def test_max_wrench_is_consistent_with_limits(allocator):
    limits = allocator.max_wrench()
    # Four horizontal thrusters at 45 deg: 4 * 15 * cos(45) = 42.4 N.
    assert limits[0] == pytest.approx(4 * 15 * np.sqrt(0.5), rel=1e-6)
    assert limits[2] == pytest.approx(60.0, rel=1e-6)
    for axis in range(6):
        w = np.zeros(6)
        w[axis] = limits[axis]
        thrust, scales = allocator.allocate(w)
        assert all(s == pytest.approx(1.0) for s in scales.values())


def test_rank_deficient_layout_is_rejected():
    only_surge = [Thruster(f't{i}', (0.0, 0.1 * i, 0.0), (1.0, 0.0, 0.0)) for i in range(4)]
    with pytest.raises(ValueError):
        ThrusterAllocator(only_surge)
