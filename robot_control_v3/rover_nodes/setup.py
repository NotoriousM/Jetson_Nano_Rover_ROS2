from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rover_nodes'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover',
    maintainer_email='rover@robot.com',
    description='Rover control v3 — STM32 hardware-flag safety + Ackermann + odometry',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_controller_node   = rover_nodes.keyboard_controller_node:main',
            'dualshock4_controller_node = rover_nodes.dualshock4_controller_node:main',
            'flag_safety_node           = rover_nodes.flag_safety_node:main',
            'ackermann_calculator_node  = rover_nodes.ackermann_calculator_node:main',
            'serial_controller_node     = rover_nodes.serial_controller_node:main',
            'odometry_node              = rover_nodes.odometry_node:main',
            'straight_trajectory_node   = rover_nodes.straight_trajectory_node:main',
            'rover_status_node          = rover_nodes.rover_status_node:main',
        ],
    },
)
