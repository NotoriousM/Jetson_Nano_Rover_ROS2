# rover_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rover', executable='ackermann_calculator_node',
             name='ackermann_calculator', output='screen'),
        Node(package='rover', executable='serial_bridge_node',
             name='serial_bridge', output='screen'),
        Node(package='rover', executable='wheel_monitor_node',
             name='wheel_monitor', output='screen'),
        Node(package='rover', executable='odometry_node',
             name='wheel_odometry', output='screen'),
    ])