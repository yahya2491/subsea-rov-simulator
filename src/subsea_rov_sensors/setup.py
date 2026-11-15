from setuptools import find_packages, setup

package_name = 'subsea_rov_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahya',
    maintainer_email='yahya@todo.todo',
    description='Pressure-depth and USBL acoustic positioning sensor models, and a DVL '
                'relay from Gazebo to ROS 2.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pressure_sensor = subsea_rov_sensors.pressure_sensor_node:main',
            'usbl = subsea_rov_sensors.usbl_node:main',
            'dvl_bridge = subsea_rov_sensors.dvl_bridge_node:main',
        ],
    },
)
