#!/usr/bin/env python3
"""
ackermann_gazebo_node.py — Управление ровером в Gazebo через ros2_control.
=========================================================================
Принимает:  /motion_commands               Float32MultiArray [speed_m_s, steering_deg]
Публикует:  /steering_controller/commands  Float64MultiArray [4 угла рад]
            /drive_controller/commands     Float64MultiArray [6 угл. скоростей рад/с]

Принципиальное отличие от ackermann_sim_node.py:
  В RViz2-симуляции нода САМА рисует положение робота (публикует /joint_states
  и TF вручную). В Gazebo нода только КОМАНДУЕТ: отдаёт целевые углы рулевым
  суставам и целевые угловые скорости ведущим. Gazebo сам считает физику
  (трение, инерцию, контакт с землёй) и через joint_state_broadcaster
  публикует РЕАЛЬНЫЕ показания суставов.

Математика Аккермана (все исправления из ackermann_sim_node.py сохранены):
  • math.atan(a / (R ± W/2)) вместо atan2 — правильный знак для левых поворотов.
    atan2 давал углы второго квадранта (~127°) при отрицательном знаменателе.
  • abs(R ± W/2) для радиусов средних колёс — при левом повороте R < 0,
    и без abs() радиус становится отрицательным, что переворачивает знак скорости.
  • Знаки публикуемых углов: [+alpha_L, +alpha_R, -beta_L, -beta_R].
    Задние колёса зеркальны передним — геометрическое требование Аккермана.
  • DRIVE_SIGNS [+1,+1,+1,-1,-1,-1]: правый рокер повёрнут на 180° в URDF,
    поэтому правые колёса получают обратный знак угловой скорости.
=========================================================================
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray


class AckermannGazeboNode(Node):

    # Порядок СТРОГО совпадает с joints в controllers.yaml.
    STEER_ORDER = [
        'rotate_left_front_joint',   # [0] → +alpha_L
        'rotate_right_front_joint',  # [1] → +alpha_R
        'rotate_left_rear_joint',    # [2] → -beta_L  (зеркально переднему)
        'rotate_right_rear_joint',   # [3] → -beta_R  (зеркально переднему)
    ]
    DRIVE_ORDER = [
        'wheel_left_front_joint',    # [0] FL
        'wheel_left_middle_joint',   # [1] ML
        'wheel_left_rear_joint',     # [2] RL
        'wheel_right_front_joint',   # [3] FR  ← обратный знак
        'wheel_right_middle_joint',  # [4] MR  ← обратный знак
        'wheel_right_rear_joint',    # [5] RR  ← обратный знак
    ]

    # Знаки угловых скоростей.
    # Правый рокер в URDF: rpy="1.5708 0 3.14159" — повёрнут на 180° по Z.
    # Ось X правых колёс в мировых координатах направлена в противоположную сторону.
    # Чтобы ровер ехал вперёд при положительном V — правые колёса получают знак -1.
    # Если при нажатии W ровер едет назад — смените правую тройку на +1.
    DRIVE_SIGNS = [+1, +1, +1,    # левые: FL, ML, RL
                   -1, -1, -1]    # правые: FR, MR, RR

    def __init__(self):
        super().__init__('ackermann_gazebo')

        self.declare_parameter('a_distance',   0.4035)
        self.declare_parameter('b_distance',   0.4035)
        self.declare_parameter('track_width',  0.779)
        self.declare_parameter('wheel_radius', 0.097)

        self.a = self.get_parameter('a_distance').value
        self.b = self.get_parameter('b_distance').value
        self.W = self.get_parameter('track_width').value
        self.r = self.get_parameter('wheel_radius').value

        self._speed = 0.0
        self._theta = 0.0

        self.create_subscription(
            Float32MultiArray,
            '/motion_commands',
            self._cmd_cb,
            10
        )

        # ВАЖНО: ros2_control принимает Float64MultiArray (double), не Float32.
        # Если отправить Float32 — контроллер тихо игнорирует команду.
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )
        self.drive_pub = self.create_publisher(
            Float64MultiArray,
            '/drive_controller/commands',
            10
        )

        # 50 Гц — контроллеры должны постоянно получать команду,
        # иначе velocity controller обнуляет скорость по таймауту.
        self.create_timer(1.0 / 50.0, self._publish_cb)

        self.get_logger().info(
            f"AckermannGazebo ready | "
            f"a={self.a} b={self.b} W={self.W} r={self.r}"
        )

    def _cmd_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self._speed = float(msg.data[0])
            self._theta = float(msg.data[1])

    def _publish_cb(self):
        steer_rad, wheel_omega = self._ackermann(self._speed, self._theta)

        steer_msg = Float64MultiArray()
        steer_msg.data = [float(a) for a in steer_rad]
        self.steer_pub.publish(steer_msg)

        drive_msg = Float64MultiArray()
        drive_msg.data = [float(w) for w in wheel_omega]
        self.drive_pub.publish(drive_msg)

    def _ackermann(
        self,
        V: float,
        theta_deg: float
    ) -> Tuple[List[float], List[float]]:
        """
        Возвращает (steer_angles_rad [4], wheel_angular_vel_rad_s [6]).
        Математика идентична ackermann_sim_node.py — все исправления сохранены.
        """
        if abs(V) < 1e-4:
            return [0.0, 0.0, 0.0, 0.0], [0.0] * 6

        if abs(theta_deg) < 0.05:
            omega = V / self.r
            return [0.0, 0.0, 0.0, 0.0], [
                omega * self.DRIVE_SIGNS[i] for i in range(6)
            ]

        theta_rad = math.radians(theta_deg)
        R = self.a / math.tan(theta_rad)   # R>0: вправо, R<0: влево
        half_W = self.W / 2.0

        # math.atan (один аргумент) — правильные знаки для обоих направлений.
        # math.atan2 давал бы ~127° при R < 0 (второй квадрант).
        alpha_L = math.atan(self.a / (R + half_W))
        alpha_R = math.atan(self.a / (R - half_W))
        beta_L  = math.atan(self.b / (R + half_W))
        beta_R  = math.atan(self.b / (R - half_W))

        # Передние положительные, задние с обратным знаком — геометрия ICR.
        steer_rad = [+alpha_L, +alpha_R, -beta_L, -beta_R]

        abs_R = abs(R)

        # hypot всегда положителен (через корень квадратный).
        # abs() для средних — без него при R<0 радиус отрицательный,
        # и скорость меняет знак (колёса крутятся назад при движении вперёд).
        R_FL = math.hypot(self.a, R + half_W)
        R_FR = math.hypot(self.a, R - half_W)
        R_ML = abs(R + half_W)
        R_MR = abs(R - half_W)
        R_RL = math.hypot(self.b, R + half_W)
        R_RR = math.hypot(self.b, R - half_W)

        sign_V = 1.0 if V >= 0 else -1.0

        def lin_spd(Ri: float) -> float:
            return sign_V * abs(V) * Ri / abs_R

        lin_speeds = [
            lin_spd(R_FL), lin_spd(R_ML), lin_spd(R_RL),
            lin_spd(R_FR), lin_spd(R_MR), lin_spd(R_RR),
        ]

        # Линейная скорость → угловая (ω = v/r), применяем DRIVE_SIGNS.
        wheel_omega = [
            (lin_speeds[i] / self.r) * self.DRIVE_SIGNS[i]
            for i in range(6)
        ]

        return steer_rad, wheel_omega


def main(args=None):
    rclpy.init(args=args)
    node = AckermannGazeboNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_steer = Float64MultiArray()
        stop_steer.data = [0.0, 0.0, 0.0, 0.0]
        node.steer_pub.publish(stop_steer)

        stop_drive = Float64MultiArray()
        stop_drive.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node.drive_pub.publish(stop_drive)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()