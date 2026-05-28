#!/usr/bin/env python3
"""
display_gazebo.launch.py — Просмотр модели ровера в Gazebo без физики.
=====================================================================
Аналог display.launch.py, но в среде Gazebo вместо RViz2.
Физический движок запускается на паузе — модель не падает, не улетает.
Слайдеры joint_state_publisher_gui позволяют двигать суставы.

Зачем это нужно:
  • Проверить что URDF правильно отображается в Gazebo (меши, цвета)
  • Убедиться что суставы двигаются в правильную сторону
  • Отладить визуальную геометрию перед включением физики
  • Работает без ros2_control и без контроллеров

Запуск:
  ros2 launch rover_description display_gazebo.launch.py
  ros2 launch rover_description display_gazebo.launch.py use_gui:=false
=====================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('rover_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_path = os.path.join(pkg, 'urdf', 'rover.urdf.xacro')
    world_path  = os.path.join(pkg, 'worlds', 'empty.world')

    # Аргументы
    args = [
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Показывать слайдеры суставов (joint_state_publisher_gui)?'
        ),
        DeclareLaunchArgument(
            'gz_gui', default_value='true',
            description='Показывать окно Gazebo GUI?'
        ),
    ]

    # URDF через xacro — тот же файл что и для физической симуляции
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    # ── 1. Gazebo в режиме ПАУЗЫ ────────────────────────────────────────
    # paused:=true — ключевой флаг. Физический движок загружается,
    # но НЕ тикает. Модель появляется там, где заспавнена, и остаётся там.
    # Гравитация не работает, коллизии не вычисляются.
    # Нажать Play в Gazebo UI если нужно включить физику позже.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   world_path,
            'paused':  'true',    # ← вся магия здесь
            'verbose': 'false',
            'gui':     LaunchConfiguration('gz_gui'),
        }.items()
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────
    # Читает URDF и слушает /joint_states.
    # Публикует TF: без этого Gazebo не знает как отображать звенья.
    # use_sim_time=False потому что физика стоит на паузе —
    # симуляционное время не течёт, поэтому используем системные часы.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ── 3. Загрузка модели в Gazebo ───────────────────────────────────────
    # Та же команда что и в gazebo_sim.launch.py.
    # -z 0.3 — модель появится на 30 см над землёй, но не упадёт
    # потому что физика стоит на паузе.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rover',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'rover',
            '-x', '0.0', '-y', '0.0', '-z', '0.3', '-Y', '0.0',
        ],
        output='screen',
    )

    # ── 4a. joint_state_publisher_gui — СЛАЙДЕРЫ (если use_gui:=true) ────
    # Создаёт окно со слайдерами для каждого сустава из URDF.
    # При движении слайдера публикует в /joint_states → robot_state_publisher
    # обновляет TF → Gazebo показывает новое положение суставов.
    # Так вы видите в реальном времени как двигаются колёса и рулевые суставы.
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    # ── 4b. joint_state_publisher — без GUI (если use_gui:=false) ────────
    # Публикует нулевые позиции всех суставов. Нужен чтобы robot_state_publisher
    # не ругался на отсутствующие /joint_states.
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_gui')),
    )

    return LaunchDescription(args + [
        gazebo,
        rsp_node,
        spawn_entity,
        jsp_gui_node,
        jsp_node,
    ])