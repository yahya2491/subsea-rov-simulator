from setuptools import find_packages, setup

package_name = 'subsea_rov_navigation'

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
    description='AHRS and Kalman-filter navigation fusing IMU, magnetometer, DVL, '
                'pressure depth and USBL.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation = subsea_rov_navigation.navigation_node:main',
        ],
    },
)
