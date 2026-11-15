import math

import pytest

from subsea_rov_control.allocation import ThrusterAllocator, thrusters_from_arrays
from subsea_rov_control.hover import HoverConfig, pole_placement_gains
from subsea_rov_control.pid import PidGains

S = math.sqrt(0.5)

# dummy_rov layout (model.sdf); test_vehicle_config.py in subsea_rov_bringup
# checks the shipped YAML against the SDF, these tests check the maths.
NAMES = ['fl', 'fr', 'bl', 'br', 'fl_top', 'fr_top', 'bl_top', 'br_top']
POSITIONS = [0.40, 0.31, -0.02, 0.40, -0.31, -0.02, -0.40, 0.31, -0.02, -0.40, -0.31, -0.02,
             0.30, 0.30, 0.10, 0.30, -0.30, 0.10, -0.30, 0.30, 0.10, -0.30, -0.30, 0.10]
DIRECTIONS = [S, -S, 0, S, S, 0, S, S, 0, S, -S, 0,
              0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1]
GROUPS = ['horizontal'] * 4 + ['vertical'] * 4


@pytest.fixture
def allocator():
    return ThrusterAllocator(thrusters_from_arrays(
        NAMES, POSITIONS, DIRECTIONS, GROUPS, 15.0, -15.0))


def hover_config():
    """Gains used by the closed-loop tests, designed the same way as the YAML."""
    m, izz = 34.6, 3.8
    kp, ki, kd = pole_placement_gains(m, 40.0, 0.8)
    kpz, kiz, kdz = pole_placement_gains(m, 80.0, 1.0)
    kpy, kiy, kdy = pole_placement_gains(izz, 9.0, 1.2)
    return HoverConfig(
        gains={
            'x': PidGains(kp, ki, kd, i_limit=25.0, out_limit=40.0),
            'y': PidGains(kp, ki, kd, i_limit=25.0, out_limit=40.0),
            'z': PidGains(kpz, kiz, kdz, i_limit=30.0, out_limit=55.0),
            'roll': PidGains(8.0, 0.5, 2.0, i_limit=3.0, out_limit=15.0),
            'pitch': PidGains(8.0, 0.5, 2.0, i_limit=3.0, out_limit=15.0),
            'yaw': PidGains(kpy, kiy, kdy, i_limit=8.0, out_limit=25.0),
        },
        buoyancy_feedforward=-14.6,
        wrench_limit=[40.0, 40.0, 55.0, 15.0, 15.0, 25.0],
    )
