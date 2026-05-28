#!/usr/bin/env python3
"""
robot_bringup.launch.py — ОСНОВНАЯ АРХИТЕКТУРА РОБОТА (v3)

Запускает базовые узлы:
  flag_safety_node         — аппаратная защита (читает флаг STM32)
  ackermann_calculator     — кинематика
  serial_controller        — мост Jetson↔STM32 (USB CDC)
  odometry_node            — позиция и /tf
  rover_status_node        — агрегированный статус

Контроллеры (клавиатура / геймпад / траектория) запускаются ОТДЕЛЬНО.

Использование:
  ros2 launch rover_nodes robot_bringup.launch.py
  ros2 launch rover_nodes robot_bringup.launch.py log_level:=debug

После запуска:
  # 1. Проверить что всё работает:
  ros2 topic echo /rover/status

  # 2. Если STM32 не имеет аппаратных флагов — система сразу готова к работе.
  #    Если защита заблокирована вручную — снять:
  ros2 topic pub /safety/clear std_msgs/msg/Bool "{data: false}" --once

  # 3. Запустить контроллер (в другом терминале):
  ros2 launch rover_nodes keyboard_control.launch.py
  # или:
  ros2 launch rover_nodes dualshock4_control.launch.py
  # или:
  ros2 launch rover_nodes trajectory_control.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_nodes')
    params    = os.path.join(pkg_share, 'config', 'robot_params.yaml')

    args = [
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='debug | info | warn | error'),
        DeclareLaunchArgument(
            'params_file', default_value=params,
            description='Path to params YAML'),
    ]

    log_level = LaunchConfiguration('log_level')
    prms      = LaunchConfiguration('params_file')

    def rover_node(executable: str):
        return Node(
            package='rover_nodes',
            executable=executable,
            name=executable,
            parameters=[prms],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen',
            emulate_tty=True,
        )

    # Порядок запуска:
    # 1. flag_safety  — слушает /motion_commands → /motion_commands_safe
    # 2. ackermann    — готов вычислять кинематику
    # 3. serial       — подключается к STM32 (задержка 1с — USB enum).
    #                   Только когда serial запустится, появится /wheels/state
    #                   с реальными флагами от STM32.
    # 4. odometry     — ждёт первые данные от serial (задержка 2с).
    # 5. status       — агрегирует всё (запускается последним).

    safety    = rover_node('flag_safety_node')
    ackermann = rover_node('ackermann_calculator_node')
    serial    = TimerAction(period=1.0, actions=[rover_node('serial_controller_node')])
    odometry  = TimerAction(period=2.0, actions=[rover_node('odometry_node')])
    status    = TimerAction(period=3.0, actions=[rover_node('rover_status_node')])

    log_start = LogInfo(msg=(
        '\n'
        '═══════════════════════════════════════════════════════\n'
        '  robot_control_v3 — STARTING\n'
        '═══════════════════════════════════════════════════════\n'
        '  Safety source: STM32 hardware flag (usb_stop_flag).\n'
        '  System is ready when /wheels/state shows flag=0 for all wheels.\n'
        '\n'
        '  Then launch a controller (in another terminal):\n'
        '    ros2 launch rover_nodes keyboard_control.launch.py\n'
        '    ros2 launch rover_nodes dualshock4_control.launch.py\n'
        '    ros2 launch rover_nodes trajectory_control.launch.py\n'
        '═══════════════════════════════════════════════════════'
    ))

    log_ready = TimerAction(
        period=4.0,
        actions=[LogInfo(msg='✅ All nodes launched. System READY.')],
    )

    return LaunchDescription([
        *args,
        log_start,
        safety,
        ackermann,
        serial,
        odometry,
        status,
        log_ready,
    ])
