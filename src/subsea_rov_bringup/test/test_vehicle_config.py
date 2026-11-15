"""The ROS-side config must describe the same vehicle and world as the SDF.

Thruster geometry, sensor mounts, the DVL beam tilt, the magnetic field and
the net buoyancy are each written down twice: in the SDF (physics) and in
YAML (estimation and control). These tests fail the moment the two drift.
"""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config')
GZ_NS = '{http://gazebosim.org/schema}'


def world_share():
    # Prefer the sibling source checkout: another workspace on the path may
    # provide a different package with the same name.
    source = os.path.join(HERE, '..', '..', 'SUBSEA-WORLD-2026')
    if os.path.isdir(os.path.join(source, 'models', 'dummy_rov')):
        return source, os.path.join('src', 'worlds')
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('underwater_world'), 'worlds'


SHARE, WORLDS = world_share()
MODEL_SDF = os.path.join(SHARE, 'models', 'dummy_rov', 'model.sdf')
WORLD_SDF = os.path.join(SHARE, WORLDS, 'hydrodynamics_world.sdf')


def params(name, node='/**'):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def vec(text):
    return np.array([float(v) for v in text.split()])


@pytest.fixture(scope='module')
def model():
    return ET.parse(MODEL_SDF).getroot().find('model')


@pytest.fixture(scope='module')
def vehicle():
    return params('vehicle.yaml')


def test_thruster_geometry_matches_sdf(model, vehicle):
    names = vehicle['thruster_names']
    pos = np.array(vehicle['thruster_positions']).reshape(-1, 3)
    axis = np.array(vehicle['thruster_directions']).reshape(-1, 3)
    plugin_joints = [p.findtext('joint_name') for p in model.findall('plugin')
                     if p.get('filename') == 'gz-sim-thruster-system']
    assert vehicle['thruster_joints'] == plugin_joints

    for i, (name, joint) in enumerate(zip(names, vehicle['thruster_joints'])):
        j = model.find(f"joint[@name='{joint}']")
        assert j is not None, joint
        link = model.find(f"link[@name='{j.findtext('child')}']")
        assert link.findtext('pose') is not None
        assert link.find('pose').get('relative_to') == 'base_link'
        np.testing.assert_allclose(vec(link.findtext('pose'))[:3], pos[i], atol=1e-9,
                                   err_msg=f'{name} position')
        sdf_axis = vec(j.find('axis').findtext('xyz'))
        np.testing.assert_allclose(sdf_axis / np.linalg.norm(sdf_axis),
                                   axis[i] / np.linalg.norm(axis[i]),
                                   atol=1e-4, err_msg=f'{name} axis')


def test_thrust_limits_match_sdf(model, vehicle):
    for p in model.findall('plugin'):
        if p.get('filename') == 'gz-sim-thruster-system':
            assert float(p.findtext('max_thrust_cmd')) == vehicle['max_thrust']
            assert float(p.findtext('min_thrust_cmd')) == vehicle['min_thrust']


@pytest.mark.parametrize('sensor, key', [('imu', 'imu_mount'), ('dvl', 'dvl_mount')])
def test_gazebo_sensor_mounts_match(model, vehicle, sensor, key):
    s = model.find(f"link[@name='base_link']/sensor[@name='{sensor}']")
    pose = vec(s.findtext('pose'))
    np.testing.assert_allclose(pose[:3], vehicle[key], atol=1e-9)
    np.testing.assert_allclose(pose[3:], 0.0, atol=1e-9)   # frames are axis-aligned


@pytest.mark.parametrize('frame, key', [('pressure_sensor', 'pressure_mount'),
                                        ('usbl_transponder', 'usbl_mount')])
def test_ros_sensor_mounts_match(model, vehicle, frame, key):
    f = model.find(f"frame[@name='{frame}']")
    assert f.get('attached_to') == 'base_link'
    np.testing.assert_allclose(vec(f.findtext('pose'))[:3], vehicle[key], atol=1e-9)


def test_dvl_settings_match(model):
    dvl = model.find("link[@name='base_link']/sensor[@name='dvl']")
    assert dvl.get(GZ_NS + 'type') == 'dvl'
    tilts = {float(b.findtext('tilt')) for b in dvl.find(GZ_NS + 'dvl').iter('beam')}
    assert tilts == {params('sensors.yaml', '/**/dvl_bridge')['beam_tilt_deg']}
    bridge = params('sensors.yaml', '/**/dvl_bridge')
    assert bridge['gz_topic'] == dvl.findtext('topic')


def test_world_magnetic_field_matches():
    world = ET.parse(WORLD_SDF).getroot().find('world')
    np.testing.assert_allclose(vec(world.findtext('magnetic_field')),
                               params('environment.yaml')['magnetic_field'], rtol=1e-9)


def test_buoyancy_feedforward_matches_sdf(model):
    """Feedforward = -(rho g V - m g) from the SDF's collision volume and masses."""
    env = params('environment.yaml')
    volume = 0.0
    for col in model.find("link[@name='base_link']").findall('collision'):
        geom = col.find('geometry')
        if geom.find('cylinder') is not None:
            c = geom.find('cylinder')
            volume += math.pi * float(c.findtext('radius')) ** 2 * float(c.findtext('length'))
        elif geom.find('box') is not None:
            volume += float(np.prod(vec(geom.find('box').findtext('size'))))
        else:
            pytest.fail(f'unhandled collision geometry in {col.get("name")}')
    mass = sum(float(link.findtext('inertial/mass')) for link in model.findall('link'))
    net = env['water_density'] * env['gravity'] * volume - mass * env['gravity']
    ff = params('control.yaml', '/**/hover_controller')['buoyancy_feedforward']
    assert ff == pytest.approx(-net, abs=0.5)


def test_bridge_covers_every_thruster(vehicle):
    with open(os.path.join(CONFIG, 'bridge.yaml')) as f:
        bridge = yaml.safe_load(f)
    gz_topics = {e['gz_topic_name']: e for e in bridge}
    for name, joint in zip(vehicle['thruster_names'], vehicle['thruster_joints']):
        e = gz_topics[f'/model/dummy_rov/joint/{joint}/cmd_thrust']
        assert e['ros_topic_name'] == f'/rov/thrusters/{name}/cmd'
        assert e['direction'] == 'ROS_TO_GZ'
