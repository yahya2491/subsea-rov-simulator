"""Full simulation stack: Gazebo world, bridge, sensors, navigation, control.

    ros2 launch subsea_rov_bringup sim.launch.py               # with GUI
    ros2 launch subsea_rov_bringup sim.launch.py gui:=false    # headless
    ros2 launch subsea_rov_bringup sim.launch.py scenario:=steady_current gui:=false
    ros2 launch subsea_rov_bringup sim.launch.py controller:=false   # sensors + nav only

With ``scenario`` set, the scenario runner plays that disturbance scenario,
writes its results to ``output_dir`` and shuts the whole launch down.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NS = 'rov'


def setup(context, *args, **kwargs):
    bringup = get_package_share_directory('subsea_rov_bringup')
    world_pkg = get_package_share_directory('underwater_world')
    cfg = lambda name: os.path.join(bringup, 'config', name)  # noqa: E731
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    gui = arg('gui').lower() in ('true', '1')
    world = arg('world') or os.path.join(world_pkg, 'worlds', 'hydrodynamics_world.sdf')
    seed = int(arg('seed'))
    scenario = arg('scenario')
    controller = arg('controller').lower() in ('true', '1')
    log_level = arg('log_level')

    resource_path = os.pathsep.join(
        p for p in (os.path.join(world_pkg, 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', ''))
        if p)
    gz_args = f'-r --seed {seed} {world}' if gui else \
        f'-r -s --headless-rendering --seed {seed} {world}'

    common = [cfg('environment.yaml'), cfg('vehicle.yaml'), {'use_sim_time': True}]
    ros_args = ['--ros-args', '--log-level', log_level]

    def node(executable, package, params, **kw):
        return Node(package=package, executable=executable, namespace=NS, output='screen',
                    parameters=common + params, arguments=ros_args, **kw)

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             output='screen', arguments=ros_args,
             parameters=[{'config_file': cfg('bridge.yaml'), 'use_sim_time': True}]),
        # Sensors
        node('dvl_bridge', 'subsea_rov_sensors', [cfg('sensors.yaml')]),
        node('pressure_sensor', 'subsea_rov_sensors', [cfg('sensors.yaml'), {'seed': seed + 1}]),
        node('usbl', 'subsea_rov_sensors', [cfg('sensors.yaml'), {'seed': seed + 2}]),
        # Navigation
        node('navigation', 'subsea_rov_navigation', [cfg('navigation.yaml')]),
    ]
    if controller:
        actions += [
            node('thruster_allocator', 'subsea_rov_control', [cfg('control.yaml')]),
            node('hover_controller', 'subsea_rov_control', [cfg('control.yaml')]),
        ]
    if scenario:
        runner = node('scenario_runner', 'subsea_rov_bringup', [{
            'scenario': scenario,
            'scenarios_file': arg('scenarios_file') or cfg('scenarios.yaml'),
            'output_dir': arg('output_dir'),
        }])
        actions += [runner, RegisterEventHandler(OnProcessExit(
            target_action=runner, on_exit=[EmitEvent(event=Shutdown(reason='scenario done'))]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Run the Gazebo GUI'),
        DeclareLaunchArgument('world', default_value='',
                              description='World SDF (default: hydrodynamics_world.sdf)'),
        DeclareLaunchArgument('controller', default_value='true',
                              description='Start the hover controller and thruster allocator'),
        DeclareLaunchArgument('seed', default_value='1',
                              description='Seed for Gazebo sensor noise and the sensor models'),
        DeclareLaunchArgument('scenario', default_value='',
                              description='Scenario to run (config/scenarios.yaml)'),
        DeclareLaunchArgument('scenarios_file', default_value=''),
        DeclareLaunchArgument('output_dir', default_value='validation_results'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=setup),
    ])
