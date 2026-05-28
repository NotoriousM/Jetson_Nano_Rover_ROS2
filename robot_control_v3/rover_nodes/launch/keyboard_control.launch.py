#!/usr/bin/env python3
"""
keyboard_control.launch.py — управление с клавиатуры.

Запускается ОТДЕЛЬНО от основной системы (robot_bringup.launch.py).

Перед запуском:
  ros2 launch rover_nodes robot_bringup.launch.py
  ros2 topic pub /safety/clear std_msgs/msg/Bool "{data: false}" --once

Запуск:
  ros2 launch rover_nodes keyboard_control.launch.py

Управление: W/S, A/D, C, SPACE, X (safety on), R (safety clear), Q (выход).
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

    keyboard = Node(
        package='rover_nodes',
        executable='keyboard_controller_node',
        name='keyboard_controller_node',
        parameters=[prms],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        emulate_tty=True,
        # IMPORTANT: keyboard needs interactive terminal
        prefix=['xterm -e'] if os.environ.get('DISPLAY') else None,
    )

    log_start = LogInfo(msg=(
        '\n'
        '────────────────────────────────────────────\n'
        '  KEYBOARD CONTROLLER\n'
        '────────────────────────────────────────────\n'
        '  W/S — speed forward/back\n'
        '  A/D — steer left/right\n'
        '  C   — center steering\n'
        '  SPACE — full stop\n'
        '  X   — Manual safety block ON\n'
        '  R   — Manual safety block OFF\n'
        '  Q   — quit\n'
        '────────────────────────────────────────────'
    ))

    return LaunchDescription([
        *args,
        log_start,
        keyboard,
    ])
