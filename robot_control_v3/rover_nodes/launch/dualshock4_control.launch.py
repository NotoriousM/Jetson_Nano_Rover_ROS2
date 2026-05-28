#!/usr/bin/env python3
"""
dualshock4_control.launch.py — управление с PlayStation 4 геймпада.

Перед запуском:
  ros2 launch rover_nodes robot_bringup.launch.py
  ros2 topic pub /safety/clear std_msgs/msg/Bool "{data: false}" --once

Запуск:
  ros2 launch rover_nodes dualshock4_control.launch.py

Подключение геймпада:
  USB:        просто подключить кабелем
  Bluetooth:  bluetoothctl → scan on → pair MAC → connect MAC

Проверка:
  evtest    # выберите устройство 'Wireless Controller'
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_nodes')
    params    = os.path.join(pkg_share, 'config', 'robot_params.yaml')

    args = [
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('params_file', default_value=params),
    ]

    log_level = LaunchConfiguration('log_level')
    prms      = LaunchConfiguration('params_file')

    ds4 = Node(
        package='rover_nodes',
        executable='dualshock4_controller_node',
        name='dualshock4_controller_node',
        parameters=[prms],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        emulate_tty=True,
    )

    log_start = LogInfo(msg=(
        '\n'
        '────────────────────────────────────────────\n'
        '  DUALSHOCK 4 CONTROLLER\n'
        '────────────────────────────────────────────\n'
        '  Left stick Y  — speed\n'
        '  Right stick X — steering\n'
        '  X (cross)     — Manual safety ON\n'
        '  O (circle)    — Manual safety OFF\n'
        '  △ (triangle)  — max speed +\n'
        '  □ (square)    — max speed −\n'
        '  OPTIONS       — center steering\n'
        '────────────────────────────────────────────'
    ))

    return LaunchDescription([
        *args,
        log_start,
        ds4,
    ])
