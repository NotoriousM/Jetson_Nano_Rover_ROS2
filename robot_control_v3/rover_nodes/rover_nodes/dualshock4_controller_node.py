#!/usr/bin/env python3
"""
DUALSHOCK4 CONTROLLER NODE
===========================
Управление с геймпада PlayStation 4 через evdev.

ВЫХОД: /motion_commands [MotionCommand] → flag_safety_node
       /safety/clear     [Bool]          → flag_safety_node

УПРАВЛЕНИЕ:
  Левый стик Y     — скорость (вверх=вперёд, вниз=назад)
  Правый стик X    — угол поворота (влево/вправо)
  X (крест)        — ручной блок защиты ON
  O (круг)         — снять ручной блок
  △ (треугольник)  — увеличить max_speed на 0.1 м/с
  □ (квадрат)      — уменьшить max_speed на 0.1 м/с
  OPTIONS          — центрирование руля

АРХИТЕКТУРА:
  Поток 0 (ROS): таймеры публикации
  Поток 1 (evdev): блокирующее чтение событий контроллера
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Bool

from rover_interfaces.msg import MotionCommand

try:
    import evdev
    from evdev import InputDevice, ecodes
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False


# ── Кнопки DualShock 4 ───────────────────────────────────────────────────────
BTN_X       = 304   # ✕ — manual safety ON
BTN_CIRCLE  = 305   # ◯ — manual safety OFF
BTN_TRI     = 307   # △ — speed +
BTN_SQUARE  = 308   # □ — speed −
BTN_OPTIONS = 315   # OPTIONS — центрирование

# Оси
AXIS_LY = 1   # левый стик Y (скорость)
AXIS_RX = 3   # правый стик X (поворот)


class DualShock4ControllerNode(Node):

    def __init__(self):
        super().__init__('dualshock4_controller_node')

        D = ParameterType.PARAMETER_DOUBLE

        def pd(d):
            return ParameterDescriptor(description=d, type=D)

        self.declare_parameter('max_speed',          2.0,  pd('Макс. скорость (м/с)'))
        self.declare_parameter('max_steering_angle', 30.0, pd('Макс. угол (°)'))
        self.declare_parameter('deadzone',           0.1,  pd('Мёртвая зона стика (0..1)'))
        self.declare_parameter('steering_exponent',  1.5,  pd('Нелинейность поворота'))
        self.declare_parameter('publish_rate_hz',    20.0, pd('Частота публикации'))

        self.max_speed = self.get_parameter('max_speed').value
        self.max_angle = self.get_parameter('max_steering_angle').value
        self.deadzone  = self.get_parameter('deadzone').value
        self.steer_exp = self.get_parameter('steering_exponent').value
        rate           = self.get_parameter('publish_rate_hz').value

        # Состояние (атомарные float через GIL)
        self._speed = 0.0
        self._steer = 0.0
        self._running = True

        # Публикаторы
        self._cmd_pub    = self.create_publisher(
            MotionCommand, '/motion_commands', 10)
        self._safety_pub = self.create_publisher(
            Bool, '/safety/clear', 10)

        # Поиск геймпада
        self._device = self._find_controller()
        if self._device is None:
            self.get_logger().error(
                'DualShock4 not found! Check connection (USB or Bluetooth).'
            )
            self._running = False
            return

        self.get_logger().info(f'Connected: {self._device.name} ({self._device.path})')

        # Поток чтения событий
        self._gamepad_thread = threading.Thread(
            target=self._gamepad_loop,
            name='dualshock4',
            daemon=True,
        )
        self._gamepad_thread.start()

        # Таймер публикации
        self.create_timer(1.0 / rate, self._publish_callback)

    def _find_controller(self):
        """Ищем DualShock 4 среди устройств ввода."""
        if not EVDEV_AVAILABLE:
            self.get_logger().error('evdev module not installed: pip3 install evdev')
            return None

        for path in evdev.list_devices():
            try:
                dev = InputDevice(path)
                name = dev.name.lower()
                if 'wireless controller' in name or 'dualshock' in name or 'sony' in name:
                    return dev
            except Exception:
                pass
        return None

    def _gamepad_loop(self):
        """Поток чтения событий — блокирующий read_loop()."""
        try:
            for event in self._device.read_loop():
                if not self._running or not rclpy.ok():
                    break
                if event.type == ecodes.EV_KEY:
                    self._process_button(event)
                elif event.type == ecodes.EV_ABS:
                    self._process_axis(event)
        except Exception as e:
            self.get_logger().error(f'Gamepad error: {e}')

    def _process_button(self, event):
        """Обработка нажатия кнопок."""
        if event.value != 1:   # обрабатываем только нажатие, не отпускание
            return

        if event.code == BTN_X:
            msg = Bool(); msg.data = True
            self._safety_pub.publish(msg)
            self.get_logger().warn('Manual safety block ENGAGED (X)')

        elif event.code == BTN_CIRCLE:
            msg = Bool(); msg.data = False
            self._safety_pub.publish(msg)
            self.get_logger().info('Manual safety block RELEASED (O)')

        elif event.code == BTN_TRI:
            self.max_speed = min(self.max_speed + 0.1, 5.0)
            self.get_logger().info(f'Max speed → {self.max_speed:.1f} m/s')

        elif event.code == BTN_SQUARE:
            self.max_speed = max(self.max_speed - 0.1, 0.1)
            self.get_logger().info(f'Max speed → {self.max_speed:.1f} m/s')

        elif event.code == BTN_OPTIONS:
            self._steer = 0.0
            self.get_logger().info('Steering centered (OPTIONS)')

    def _process_axis(self, event):
        """Обработка осей стиков. Значение event.value: 0..255, центр 128."""
        # Нормализация в диапазон −1..+1
        value = (event.value - 128) / 128.0

        if abs(value) < self.deadzone:
            value = 0.0

        if event.code == AXIS_LY:
            # Левый стик Y: вверх = отрицательное значение в evdev
            v = -value
            v = math.copysign(abs(v) ** 1.5, v) if v != 0 else 0
            self._speed = v * self.max_speed

        elif event.code == AXIS_RX:
            v = math.copysign(abs(value) ** self.steer_exp, value) if value != 0 else 0
            self._steer = v * self.max_angle

    def _publish_callback(self):
        """20 Гц публикация текущего состояния."""
        msg = MotionCommand()
        msg.linear_velocity = float(self._speed)
        msg.steering_angle  = float(self._steer)
        msg.source          = 'ds4'
        self._cmd_pub.publish(msg)

        self.get_logger().debug(
            f'Speed: {self._speed:.2f}m/s  Steer: {self._steer:.1f}°',
            throttle_duration_sec=0.5,
        )

    def destroy_node(self):
        self._running = False
        try:
            stop = MotionCommand()
            stop.source = 'ds4'
            self._cmd_pub.publish(stop)
            time.sleep(0.1)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualShock4ControllerNode()
    if not node._running:
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
