#!/usr/bin/env python3
"""
trajectory_control.launch.py — автономное движение по прямой.

Перед запуском:
  ros2 launch rover_nodes robot_bringup.launch.py

Запуск:
  ros2 launch rover_nodes trajectory_control.launch.py

Использование:
  # Проехать 2 метра вперёд со скоростью 0.5 м/с:
  ros2 service call /start_straight_trajectory \
       rover_interfaces/srv/StartStraightTrajectory \
       "{distance: 2.0, speed: 0.5}"

  # Проехать 1 метр назад со скоростью 0.3 м/с:
  ros2 service call /start_straight_trajectory \
       rover_interfaces/srv/StartStraightTrajectory \
       "{distance: -1.0, speed: 0.3}"

  # Мониторинг:
  ros2 topic echo /trajectory/status
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

    trajectory = Node(
        package='rover_nodes',
        executable='straight_trajectory_node',
        name='straight_trajectory_node',
        parameters=[prms],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        emulate_tty=True,
    )

    log_start = LogInfo(msg=(
        '\n'
        '────────────────────────────────────────────────────────\n'
        '  STRAIGHT TRAJECTORY CONTROLLER\n'
        '────────────────────────────────────────────────────────\n'
        '  Service: /start_straight_trajectory\n'
        '  Topic:   /trajectory/status\n'
        '\n'
        '  Example:\n'
        '    ros2 service call /start_straight_trajectory \\\n'
        '      rover_interfaces/srv/StartStraightTrajectory \\\n'
        '      "{distance: 2.0, speed: 0.5}"\n'
        '────────────────────────────────────────────────────────'
    ))

    return LaunchDescription([
        *args,
        log_start,
        trajectory,
    ])
