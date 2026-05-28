#!/usr/bin/env python3
"""
robot_description.launch.py — Минимальный запуск для rosbridge / Foxglove.

Запускает только:
  • robot_state_publisher — публикует топик /robot_description (URDF как строка)
                             и /tf на основе /joint_states

Без RViz, без GUI, без Gazebo. Этот launch предназначен для:
  • Foxglove Studio через rosbridge_websocket
  • Удалённой визуализации в браузере
  • Интеграции с любым внешним инструментом, который читает /robot_description

Связка с реальным роботом (robot_control_v3):
  • На Jetson: ros2 launch rover_nodes robot_bringup.launch.py
  • На Jetson: ros2 launch rover_description robot_description.launch.py
  • На Jetson: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  • С браузера/Foxglove: ws://<jetson-ip>:9090

Источники /joint_states (нужны чтобы /tf обновлялся):
  • На реальном роботе — отдельный узел (не входит в этот пакет!),
    публикует joint_state на основе данных от STM32.
  • Для тестов URDF — можно запустить joint_state_publisher_gui отдельно:
      ros2 run joint_state_publisher_gui joint_state_publisher_gui
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share  = get_package_share_directory('rover_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Использовать симуляционное время /clock'
    )

    robot_description = {
        'robot_description': Command(['xacro ', xacro_path]),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    return LaunchDescription([
        use_sim_time_arg,
        rsp_node,
    ])
