#!/usr/bin/env python3
"""
ACKERMANN CALCULATOR NODE
==========================
Роль: вычисляет скорости и углы 6 колёс по модели Аккермана.

ВХОД:  /motion_commands_safe [MotionCommand]   ← flag_safety_node
ВЫХОД: /wheel/{name}/cmd     [WheelCommand]    → serial_controller_node (×6)

Это чистая математика — нет I/O, нет потоков, только вычисления.
"""

import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rover_interfaces.msg import MotionCommand, WheelCommand


WHEEL_NAMES = [
    'front_left',  'middle_left',  'rear_left',
    'front_right', 'middle_right', 'rear_right',
]
NON_STEERING = {'middle_left', 'middle_right'}   # средние колёса без сервопривода


class AckermannCalculatorNode(Node):

    def __init__(self):
        super().__init__('ackermann_calculator_node')

        # ── Параметры ────────────────────────────────────────────────────────
        D = ParameterType.PARAMETER_DOUBLE

        def pd(d):
            return ParameterDescriptor(description=d, type=D)

        self.declare_parameter('wheelbase',             0.807,  pd('Колёсная база (м)'))
        self.declare_parameter('track_width',           0.779,  pd('Ширина колеи (м)'))
        self.declare_parameter('a_distance',            0.4035, pd('Передняя ось → центр (м)'))
        self.declare_parameter('b_distance',            0.4035, pd('Задняя ось → центр (м)'))
        self.declare_parameter('initial_pos_servo_deg', 90.0,   pd('Нейтраль сервопривода (°)'))
        self.declare_parameter('max_speed',             2.0,    pd('Макс. скорость (м/с)'))
        self.declare_parameter('max_steering_angle',    35.0,   pd('Макс. угол поворота (°)'))

        self.a    = self.get_parameter('a_distance').value
        self.b    = self.get_parameter('b_distance').value
        self.W    = self.get_parameter('track_width').value
        self.srv0 = self.get_parameter('initial_pos_servo_deg').value
        self.max_v  = self.get_parameter('max_speed').value
        self.max_th = self.get_parameter('max_steering_angle').value

        # ── Публикаторы (по одному на колесо) ────────────────────────────────
        self._cmd_pubs = {
            name: self.create_publisher(WheelCommand, f'/wheel/{name}/cmd', 10)
            for name in WHEEL_NAMES
        }

        # ── Подписка на финальные команды (после E-Stop фильтра) ─────────────
        self.create_subscription(
            MotionCommand, '/motion_commands_safe',
            self._motion_callback, 10
        )

        self._last_v = 0.0
        self._last_th = 0.0
        self.create_timer(3.0, self._diag_callback)

        self.get_logger().info(
            f'AckermannCalculatorNode ready\n'
            f'  Subscribes: /motion_commands_safe\n'
            f'  Publishes:  /wheel/{{name}}/cmd × 6\n'
            f'  Geometry:   a={self.a}m b={self.b}m W={self.W}m\n'
            f'  Limits:     v_max={self.max_v}m/s θ_max={self.max_th}°'
        )

    def _motion_callback(self, msg: MotionCommand):
        """Получаем MotionCommand → вычисляем 6 WheelCommand."""
        # Ограничения
        v  = max(-self.max_v,  min(self.max_v,  float(msg.linear_velocity)))
        th = max(-self.max_th, min(self.max_th, float(msg.steering_angle)))
        self._last_v, self._last_th = v, th

        speeds, angles = self._compute_ackermann(v, th)

        # Публикуем WheelCommand для каждого колеса
        for name in WHEEL_NAMES:
            cmd = WheelCommand()
            cmd.wheel_name = name
            cmd.speed_cmd  = float(speeds[name])
            cmd.angle_cmd  = float(angles[name])
            self._cmd_pubs[name].publish(cmd)

        self.get_logger().debug(
            f'[{msg.source}] v={v:.2f}m/s θ={th:.1f}°',
            throttle_duration_sec=0.5,
        )

    def _compute_ackermann(self, V: float, theta: float):
        """
        Геометрия Аккермана для 6-колёсного шасси.

        ICC = Instantaneous Center of Curvature (мгновенный центр поворота)
        R = расстояние от ICC до продольной оси робота
        omega = угловая скорость = V / R
        """
        # Прямолинейное движение — все колёса одинаково
        if abs(theta) < 0.01:
            return (
                {n: V         for n in WHEEL_NAMES},
                {n: self.srv0 for n in WHEEL_NAMES},
            )

        a, b, W = self.a, self.b, self.W
        R     = a / math.tan(math.radians(theta))
        omega = V / R

        # Скорости пропорциональны радиусу от ICC
        speeds = {
            'front_left':   omega * math.sqrt(a**2 + (R + W/2)**2),
            'middle_left':  omega * abs(R + W/2),
            'rear_left':    omega * math.sqrt(b**2 + (R + W/2)**2),
            'front_right':  omega * math.sqrt(a**2 + (R - W/2)**2),
            'middle_right': omega * abs(R - W/2),
            'rear_right':   omega * math.sqrt(b**2 + (R - W/2)**2),
        }

        # Углы поворота сервоприводов
        raw = {
            'front_left':  math.degrees(math.atan2(a, R + W/2)),
            'front_right': math.degrees(math.atan2(a, R - W/2)),
            'rear_left':  -math.degrees(math.atan2(b, R + W/2)),
            'rear_right': -math.degrees(math.atan2(b, R - W/2)),
        }

        angles = {}
        for n in WHEEL_NAMES:
            if n in NON_STEERING:
                angles[n] = self.srv0   # средние колёса всегда прямо
            else:
                # Смещаем от нейтрали и ограничиваем 0..180°
                angles[n] = max(0.0, min(180.0, self.srv0 + raw[n]))

        return speeds, angles

    def _diag_callback(self):
        self.get_logger().info(
            f'[Ackermann] v={self._last_v:.2f}m/s θ={self._last_th:.1f}°',
            throttle_duration_sec=3.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannCalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
