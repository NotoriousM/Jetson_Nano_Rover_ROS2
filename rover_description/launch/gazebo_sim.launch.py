#!/usr/bin/env python3
"""
gazebo_sim.launch.py — Полный запуск симуляции ровера в Gazebo.
=========================================================================
Последовательность запуска (порядок критически важен):

  1. Gazebo (gzserver + gzclient) — физический движок + окно симуляции
  2. robot_state_publisher — читает URDF, публикует /robot_description
  3. spawn_entity — загружает модель ровера в сцену Gazebo
  4. Ждём 4 секунды (TimerAction) — Gazebo должен успеть инициализировать
     controller_manager после загрузки модели
  5. joint_state_broadcaster — читает из Gazebo, публикует /joint_states
  6. steering_controller — принимает команды поворота (position)
  7. drive_controller — принимает команды скорости (velocity)
  8. ackermann_gazebo_node — вычисляет Аккермана, шлёт контроллерам
  9. robot_keyboard_controller — ввод с клавиатуры (опционально)
  10. rviz2 — 3D-визуализация (опционально)

Почему TimerAction, а не OnProcessExit:
  spawn_entity.py завершается сразу после того как Gazebo подтвердил
  приём модели — но физика и controller_manager ещё продолжают
  инициализироваться. OnProcessExit запускал бы spawner до того как
  controller_manager готов принять запросы, и spawner падал с ошибкой
  "controller manager not found". TimerAction(4.0) надёжнее на практике.

Использование:
  ros2 launch rover_description gazebo_sim.launch.py
  ros2 launch rover_description gazebo_sim.launch.py use_rviz:=true
  ros2 launch rover_description gazebo_sim.launch.py use_keyboard:=false
  ros2 launch rover_description gazebo_sim.launch.py gz_gui:=false  # без окна Gazebo
=========================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('rover_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_path  = os.path.join(pkg, 'urdf', 'rover.urdf.xacro')
    world_path  = os.path.join(pkg, 'worlds', 'empty.world')

    # Пробуем rover_sim.rviz, если нет — берём rover.rviz
    rviz_config = os.path.join(pkg, 'rviz', 'rover_sim.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg, 'rviz', 'rover.rviz')

    # ── Аргументы командной строки ─────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Запускать RViz2 для визуализации?'
        ),
        DeclareLaunchArgument(
            'use_keyboard', default_value='true',
            description='Запускать клавиатурный контроллер?'
        ),
        DeclareLaunchArgument(
            'gz_gui', default_value='true',
            description='Показывать окно Gazebo GUI?'
        ),
        DeclareLaunchArgument(
            'world', default_value=world_path,
            description='Путь к файлу мира Gazebo (.world)'
        ),
    ]

    # ── URDF через xacro ──────────────────────────────────────────────
    # Command('xacro ...') выполняет xacro и возвращает URDF-строку.
    # robot_state_publisher публикует её в /robot_description.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    # ─────────────────────────────────────────────────────────────────
    # ШАГ 1: Gazebo
    # Запускаем через стандартный launch-файл пакета gazebo_ros.
    # verbose=false: меньше шума в терминале при нормальной работе.
    # ─────────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   LaunchConfiguration('world'),
            'verbose': 'false',
            'gui':     LaunchConfiguration('gz_gui'),
        }.items()
    )

    # ─────────────────────────────────────────────────────────────────
    # ШАГ 2: robot_state_publisher
    # use_sim_time=True важно: нода должна использовать симуляционное
    # время Gazebo, а не системные часы. Иначе TF будет «прыгать».
    # ─────────────────────────────────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────
    # ШАГ 3: Загрузка модели в Gazebo
    # -topic /robot_description: spawn_entity читает URDF из этого топика
    # -z 0.3: стартовая высота над землёй — чтобы ровер не «провалился»
    #          в момент первого контакта с поверхностью.
    # ─────────────────────────────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rover',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'rover',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.60',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────
    # ШАГИ 4–6: Контроллеры ros2_control
    # spawner.py (с расширением .py) — это Foxy-специфично. В Galactic
    # и выше просто spawner (без .py).
    # ─────────────────────────────────────────────────────────────────
    load_jsb = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    load_steering = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[
            'steering_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    load_drive = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[
            'drive_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────
    # ШАГ 7: ackermann_gazebo_node
    # Вычисляет геометрию Аккермана и публикует команды контроллерам.
    # ─────────────────────────────────────────────────────────────────
    ackermann_node = Node(
        package='rover_description',
        executable='ackermann_gazebo_node.py',
        name='ackermann_gazebo',
        parameters=[{
            'a_distance':   0.4035,
            'b_distance':   0.4035,
            'track_width':  0.779,
            'wheel_radius': 0.097,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────
    # TimerAction: ждём 4 секунды после запуска перед загрузкой контроллеров.
    # Gazebo + controller_manager требуют времени на инициализацию после
    # загрузки модели. 4 секунды — достаточный запас на большинстве машин.
    # Если видите ошибку "controller manager not found" — увеличьте до 6.0.
    # ─────────────────────────────────────────────────────────────────
    delayed_controllers = TimerAction(
        period=4.0,
        actions=[
            load_jsb,
            load_steering,
            load_drive,
            ackermann_node,
        ]
    )

    # ─────────────────────────────────────────────────────────────────
    # ШАГ 8: Клавиатурный контроллер (опционально)
    # prefix='xterm -e' открывает отдельное окно терминала, потому что
    # клавиатурный ввод требует прямого доступа к stdin (tty).
    # Если xterm не установлен: sudo apt install xterm
    # ─────────────────────────────────────────────────────────────────
    keyboard_node = Node(
        package='rover_description',
        executable='robot_keyboard_controller.py',
        name='keyboard_controller',
        prefix='xterm -e',
        parameters=[{
            'max_speed':          2.0,
            'max_steering_angle': 30.0,
            'base_step_speed':    0.2,
            'base_step_angle':    2.0,
            'auto_brake':         True,
        }],
        condition=IfCondition(LaunchConfiguration('use_keyboard')),
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────
    # ШАГ 9: RViz2 (опционально)
    # По умолчанию выключен (use_rviz:=false) — обычно достаточно
    # окна Gazebo. Включайте когда нужно отладить TF-дерево или суставы.
    # ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription(args + [
        gazebo,
        rsp_node,
        spawn_entity,
        delayed_controllers,
        keyboard_node,
        rviz_node,
    ])