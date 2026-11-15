from setuptools import find_packages, setup

package_name = 'subsea_rov_control'

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
    description='Thruster allocation and closed-loop hover controller for the subsea ROV.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_allocator = subsea_rov_control.thruster_allocator_node:main',
            'hover_controller = subsea_rov_control.hover_controller_node:main',
        ],
    },
)
