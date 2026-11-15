from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'subsea_rov_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahya',
    maintainer_email='yahya@todo.todo',
    description='Launch, configuration and disturbance-rejection validation for the subsea '
                'ROV simulator.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_runner = subsea_rov_bringup.scenario_runner:main',
            'run_validation = subsea_rov_bringup.run_validation:main',
        ],
    },
)
