#!/usr/bin/env python3
"""
display.launch.py — Визуализация ровера в RViz2 с интерактивными ползунками.

Запускает:
  • robot_state_publisher        — публикует /robot_description и /tf
  • joint_state_publisher_gui    — окно с ползунками для всех joints
  • rviz2                        — визуализация (фрейм fixed_frame=base_footprint)

Использование:
  ros2 launch rover_description display.launch.py
  ros2 launch rover_description display.launch.py use_gui:=false   # без ползунков

Что увидишь:
  • Полная 3D-модель робота
  • Ползунки для 4 серво (steer_FL, FR, RL, RR) — диапазон ±35°
  • Ползунки для 6 моторов (drive_FL..RR) — continuous (вращение)
  • Ползунки для 4 рычагов подвески (rocker_left/right, bogie_left/right) — ±25°
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rover.rviz')

    use_gui_arg = DeclareLaunchArgument('use_gui', default_value='true')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        Command('xacro ' + xacro_path),
        value_type=str
)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_gui)
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_gui_arg,
        use_rviz_arg,
        rsp_node,
        jsp_node,
        jsp_gui_node,
        rviz_node,
    ])
