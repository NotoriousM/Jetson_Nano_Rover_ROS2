#!/usr/bin/env python3
"""
Нода 1: Расчёт модели Аккермана.
Вход:  /motion_commands   Float32MultiArray [speed, steering_angle]
Выход: /wheel_commands    Float32MultiArray [FL_spd, ML_spd, RL_spd, FR_spd, MR_spd, RR_spd,
                                             alpha_L1, alpha_R1, beta_L3, beta_R3]
"""
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class AckermannCalculatorNode(Node):
    def __init__(self):
        super().__init__('ackermann_calculator')

        # Параметры геометрии робота
        self.declare_parameter('wheelbase',  0.807)
        self.declare_parameter('track_width', 0.779)
        self.declare_parameter('a_distance',  0.4035)
        self.declare_parameter('b_distance',  0.4035)

        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.a = self.get_parameter('a_distance').value
        self.b = self.get_parameter('b_distance').value

        # Подписка на входящие команды
        self.create_subscription(
            Float32MultiArray,
            '/motion_commands',
            self._command_callback,
            10
        )

        # Публикация рассчитанных команд для колёс
        self.wheel_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_commands',
            10
        )

        self.get_logger().info(
            f"AckermannCalculator ready | a={self.a}m, b={self.b}m, W={self.W}m"
        )

    # ──────────────────────────────────────────────────────────────
    def _command_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().error(
                "Expected [speed, angle], got len=%d" % len(msg.data),
                throttle_duration_sec=2.0
            )
            return

        V     = float(msg.data[0])
        theta = float(msg.data[1])

        speeds, angles = self._ackermann(V, theta)

        # Формат: [6 скоростей] + [4 угла]
        out = Float32MultiArray()
        out.data = [float(v) for v in speeds] + [float(a) for a in angles]
        self.wheel_cmd_pub.publish(out)

        self.get_logger().info(
            f"V={V:.2f} θ={theta:.1f}° → speeds={[f'{v:.2f}' for v in speeds]}",
            throttle_duration_sec=1.0
        )

    # ──────────────────────────────────────────────────────────────
    def _ackermann(self, V: float, theta: float) -> Tuple[List[float], List[float]]:
        """Модель Аккермана для 6-колёсного ровера."""
        if abs(theta) < 0.001:
            return [V] * 6, [0.0] * 4

        R = self.a / math.tan(math.radians(theta))

        half_W = self.W / 2

        # Радиусы для каждого колеса
        R_L1 = math.sqrt(self.a**2 + (R + half_W)**2)
        R_R1 = math.sqrt(self.a**2 + (R - half_W)**2)
        R_L2 = R + half_W
        R_R2 = R - half_W
        R_L3 = math.sqrt(self.b**2 + (R + half_W)**2)
        R_R3 = math.sqrt(self.b**2 + (R - half_W)**2)

        # Углы поворота сервоприводов (градусы)
        alpha_L1 = math.degrees(math.atan(self.a / (R + half_W)))
        alpha_R1 = math.degrees(math.atan(self.a / (R - half_W)))
        beta_L3  = math.degrees(math.atan(self.b / (R + half_W)))
        beta_R3  = math.degrees(math.atan(self.b / (R - half_W)))

        # Скорости (пропорционально радиусу)
        def spd(Ri): return V * Ri / R if abs(R) > 0.001 else V

        speeds = [spd(R_L1), spd(R_L2), spd(R_L3),   # левые: FL, ML, RL
                  spd(R_R1), spd(R_R2), spd(R_R3)]    # правые: FR, MR, RR
        angles = [alpha_L1, alpha_R1, beta_L3, beta_R3]

        return speeds, angles


# ──────────────────────────────────────────────────────────────────
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