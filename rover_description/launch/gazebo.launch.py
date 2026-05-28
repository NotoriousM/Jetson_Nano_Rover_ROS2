#!/usr/bin/env python3
"""
gazebo.launch.py — Запускает Gazebo и спавнит робота.

НЕ запускает движение. Для запуска восьмёрки используйте отдельно:
  ros2 run rover_description figure_eight

Использование:
  ros2 launch rover_description gazebo.launch.py
  ros2 launch rover_description gazebo.launch.py gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg    = get_package_share_directory('rover_description')
    xacro  = os.path.join(pkg, 'urdf',   'rover.urdf.xacro')
    world  = os.path.join(pkg, 'worlds', 'empty.world')
    gz_ros = get_package_share_directory('gazebo_ros')

    # ── Аргументы ─────────────────────────────────────────────────
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Показывать Gazebo GUI (true/false)')

    robot_description = ParameterValue(
        Command(['xacro ', xacro]), value_type=str)

    # ── 1. robot_state_publisher ──────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ── 2. Gazebo Classic ─────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world':   world,
            'verbose': 'false',
            'gui':     LaunchConfiguration('gui'),
        }.items()
    )

    # ── 3. Спавн робота ───────────────────────────────────────────
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rover',
            '-topic',  'robot_description',
            '-x', '0.1',
            '-y', '0.1',
            '-z', '2.0',
            '-Y', '0.1',
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_gui,
        rsp,
        gazebo,
        spawn,
    ])
