#!/usr/bin/env python3
"""
KEYBOARD CONTROLLER NODE
=========================
Управление с клавиатуры через terminal stdin.

ВЫХОД: /motion_commands [MotionCommand] → flag_safety_node
       /safety/clear     [Bool]          → flag_safety_node

КЛАВИШИ:
  W / S  — скорость вперёд / назад
  A / D  — поворот влево / вправо
  C      — центрирование руля (угол=0)
  SPACE  — полная остановка (скорость=0, угол=0)
  Q      — выход
  X      — ручной блок защиты (стоп всему)
  R      — снять ручной блок (если STM32 не держит флаг)

АРХИТЕКТУРА:
  Поток 0 (ROS): таймер 20 Гц → публикует текущее состояние
  Поток 1 (keyboard): блокирующий read stdin → обновляет состояние
"""

import sys
import select
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Bool

from rover_interfaces.msg import MotionCommand


class KeyboardControllerNode(Node):

    def __init__(self):
        super().__init__('keyboard_controller_node')

        D = ParameterType.PARAMETER_DOUBLE

        def pd(d):
            return ParameterDescriptor(description=d, type=D)

        self.declare_parameter('max_speed',          2.0,  pd('Макс. скорость (м/с)'))
        self.declare_parameter('max_steering_angle', 30.0, pd('Макс. угол (°)'))
        self.declare_parameter('step_speed',         0.2,  pd('Шаг скорости'))
        self.declare_parameter('step_angle',         3.0,  pd('Шаг угла'))
        self.declare_parameter('publish_rate_hz',    20.0, pd('Частота публикации'))

        self.max_speed  = self.get_parameter('max_speed').value
        self.max_angle  = self.get_parameter('max_steering_angle').value
        self.step_speed = self.get_parameter('step_speed').value
        self.step_angle = self.get_parameter('step_angle').value
        rate            = self.get_parameter('publish_rate_hz').value

        # Текущее состояние (атомарные float через GIL)
        self._speed = 0.0
        self._steer = 0.0
        self._running = True

        # Публикаторы
        self._cmd_pub    = self.create_publisher(
            MotionCommand, '/motion_commands', 10)
        self._safety_pub = self.create_publisher(
            Bool, '/safety/clear', 10)

        # Сохраняем настройки терминала
        try:
            self._tty_settings = termios.tcgetattr(sys.stdin)
        except termios.error:
            self._tty_settings = None
            self.get_logger().error(
                'Cannot access terminal. Run from interactive terminal!')

        # Отдельный поток для блокирующего чтения клавиатуры
        self._kb_thread = threading.Thread(
            target=self._keyboard_loop,
            name='keyboard',
            daemon=True,
        )
        self._kb_thread.start()

        # Таймер публикации
        self.create_timer(1.0 / rate, self._publish_callback)

        self._print_help()

    def _print_help(self):
        msg = """
╔════════════════════════════════════════════════╗
║      KEYBOARD CONTROLLER — управление          ║
╠════════════════════════════════════════════════╣
║  W / S — скорость вперёд / назад              ║
║  A / D — поворот влево / вправо               ║
║  C     — центрирование руля                   ║
║  SPACE — полная остановка                     ║
║  X     — ручной блок защиты                   ║
║  R     — снять ручной блок                    ║
║  Q     — выход                                ║
╚════════════════════════════════════════════════╝
"""
        self.get_logger().info(msg)
        print(msg)

    def _get_key(self, timeout=0.1):
        """Неблокирующее чтение одной клавиши с таймаутом."""
        if self._tty_settings is None:
            return ''
        try:
            tty.setraw(sys.stdin.fileno())
            r, _, _ = select.select([sys.stdin], [], [], timeout)
            if r:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._tty_settings)
        return key

    def _keyboard_loop(self):
        """Поток клавиатуры — блокирует stdin."""
        while self._running and rclpy.ok():
            key = self._get_key(timeout=0.05)
            if not key:
                continue

            key = key.lower()

            if key == 'q':
                self.get_logger().info('Quit by user (Q)')
                self._speed = 0.0
                self._steer = 0.0
                self._send_command()
                self._running = False
                break

            elif key == ' ':
                self._speed = 0.0
                self._steer = 0.0
                self.get_logger().info('STOP (space)')

            elif key == 'c':
                self._steer = 0.0
                self.get_logger().info('Steering centered (C)')

            elif key == 'w':
                self._speed = min(self._speed + self.step_speed, self.max_speed)

            elif key == 's':
                self._speed = max(self._speed - self.step_speed, -self.max_speed)

            elif key == 'a':
                self._steer = max(self._steer - self.step_angle, -self.max_angle)

            elif key == 'd':
                self._steer = min(self._steer + self.step_angle, self.max_angle)

            elif key == 'x':
                msg = Bool()
                msg.data = True
                self._safety_pub.publish(msg)
                self.get_logger().warn('Manual safety block ENGAGED (X)')

            elif key == 'r':
                msg = Bool()
                msg.data = False
                self._safety_pub.publish(msg)
                self.get_logger().info('Manual safety block RELEASED (R)')

            self._print_status()

    def _print_status(self):
        direction = (
            '→ ВПЕРЁД' if self._speed > 0
            else '← НАЗАД' if self._speed < 0
            else '· СТОП'
        )
        turn = (
            '↶ ЛЕВО' if self._steer < 0
            else '↷ ПРАВО' if self._steer > 0
            else '↑ ПРЯМО'
        )
        print(
            f'\rSpeed: {self._speed:5.2f} m/s {direction} | '
            f'Steer: {self._steer:5.1f}° {turn}    ',
            end='', flush=True,
        )

    def _publish_callback(self):
        """20 Гц публикация текущего состояния."""
        self._send_command()

    def _send_command(self):
        msg = MotionCommand()
        msg.linear_velocity = float(self._speed)
        msg.steering_angle  = float(self._steer)
        msg.source          = 'keyboard'
        self._cmd_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        if self._tty_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._tty_settings)
            except Exception:
                pass
        # Финальная команда стоп
        try:
            stop = MotionCommand()
            stop.source = 'keyboard'
            self._cmd_pub.publish(stop)
            time.sleep(0.1)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControllerNode()
    try:
        while rclpy.ok() and node._running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
