#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('rover_description')
    xacro_path = os.path.join(pkg, 'urdf', 'rover.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'rover_sim.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg, 'rviz', 'rover.rviz')

    args = [
        DeclareLaunchArgument('speed',    default_value='0.5'),
        DeclareLaunchArgument('scale',    default_value='2.5'),
        DeclareLaunchArgument('loops',    default_value='2'),
        DeclareLaunchArgument('delay',    default_value='10.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('max_path', default_value='5000'),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    return LaunchDescription(args + [
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': False}],
             output='screen'),
        Node(package='rover_description',
             executable='ackermann_sim_node.py', name='ackermann_sim',
             parameters=[{'a_distance': 0.4035, 'b_distance': 0.4035,
                          'track_width': 0.779, 'wheel_radius': 0.097,
                          'publish_rate': 50.0}],
             output='screen'),
        Node(package='rover_description',
             executable='path_publisher.py', name='path_publisher',
             parameters=[{'max_poses': LaunchConfiguration('max_path'),
                          'min_distance': 0.01}],
             output='screen'),
        Node(package='rover_description',
             executable='figure_lemniscate_rviz.py', name='figure_lemniscate',
             parameters=[{'linear_speed':  LaunchConfiguration('speed'),
                          'scale':         LaunchConfiguration('scale'),
                          'loop_count':    LaunchConfiguration('loops'),
                          'delay_sec':     LaunchConfiguration('delay'),
                          'a_distance':    0.4035,
                          'max_steer_deg': 30.0,
                          'publish_rate':  20.0}],
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             output='screen'),
    ])
