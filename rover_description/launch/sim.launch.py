#!/usr/bin/env python3
"""
sim.launch.py — Запуск симуляции виртуального движения ровера.

Запускает:
  1. robot_state_publisher   — читает URDF, публикует /robot_description
                               и преобразует /joint_states → /tf
  2. ackermann_sim_node      — принимает /motion_commands, считает Аккермана,
                               публикует /joint_states и TF odom→base_footprint
  3. robot_keyboard_controller — ввод с клавиатуры → /motion_commands
  4. rviz2                   — визуализация (fixed frame = odom)

Использование:
  ros2 launch rover_description sim.launch.py
  ros2 launch rover_description sim.launch.py use_rviz:=false   # без RViz2
  ros2 launch rover_description sim.launch.py use_keyboard:=false

ВАЖНО: В RViz2 установите Fixed Frame = 'odom', чтобы видеть движение робота.
       Если нужна только анимация суставов без перемещения — используйте
       Fixed Frame = 'base_link' или 'base_footprint'.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share  = get_package_share_directory('rover_description')

    # Путь к URDF/xacro.
    # Если файл называется rover.urdf.xacro — используйте xacro-команду.
    # Если это чистый .urdf — просто откройте файл напрямую.
    xacro_path  = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rover_sim.rviz')

    # Если rover_sim.rviz ещё не создан, используем rover.rviz как запасной вариант
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_share, 'rviz', 'rover.rviz')

    # ── Аргументы запуска ─────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_rviz',     default_value='true',
                              description='Запускать RViz2?'),
        DeclareLaunchArgument('use_keyboard', default_value='true',
                              description='Запускать клавиатурный контроллер?'),

        # Параметры геометрии ровера (передаются в sim-ноду)
        DeclareLaunchArgument('a_distance',   default_value='0.4035'),
        DeclareLaunchArgument('b_distance',   default_value='0.4035'),
        DeclareLaunchArgument('track_width',  default_value='0.779'),
        DeclareLaunchArgument('wheel_radius', default_value='0.097'),
    ]

    # ── LaunchConfiguration ───────────────────────────────────────────────
    use_rviz     = LaunchConfiguration('use_rviz')
    use_keyboard = LaunchConfiguration('use_keyboard')

    robot_description = ParameterValue(
        Command('xacro ' + xacro_path),
        value_type=str
    )

    # ── Узлы ─────────────────────────────────────────────────────────────

    # 1. robot_state_publisher
    #    Читает URDF, слушает /joint_states, публикует /tf.
    #    use_sim_time=False — используем системное время (не Gazebo).
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    # 2. Узел симуляции Аккермана
    #    Принимает /motion_commands, публикует /joint_states + TF odom→base_footprint
    sim_node = Node(
        package='rover_description',           # ← укажите правильный пакет!
        executable='ackermann_sim_node.py',
        name='ackermann_sim',
        parameters=[{
            'a_distance':   LaunchConfiguration('a_distance'),
            'b_distance':   LaunchConfiguration('b_distance'),
            'track_width':  LaunchConfiguration('track_width'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'publish_rate': 50.0,
        }],
        output='screen',
    )

    # 3. Клавиатурный контроллер
    #    Запускается в отдельном терминале через prefix='xterm -e'.
    #    Если xterm не установлен: sudo apt install xterm
    keyboard_node = Node(
        package='rover_description',           # ← укажите правильный пакет!
        executable='robot_keyboard_controller.py',
        name='keyboard_controller',
        prefix='xterm -e',
        parameters=[{
            'max_speed':        1.0,   # м/с
            'max_steering_angle': 30.0,  # градусы
            'base_step_angle':    2.0,
            'base_step_speed':    0.2,
        }],
        condition=IfCondition(use_keyboard),
        output='screen',
    )

    # 4. RViz2
    #    Используем тот же rviz-конфиг, что и в display.launch.py,
    #    но Fixed Frame нужно вручную сменить на 'odom' для наблюдения движения.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [
        rsp_node,
        sim_node,
        keyboard_node,
        rviz_node,
    ])