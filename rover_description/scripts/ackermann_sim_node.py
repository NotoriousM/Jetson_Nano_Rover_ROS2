#!/usr/bin/env python3
"""
ackermann_sim_node.py — Узел виртуальной симуляции движения ровера.

Принимает:  /motion_commands  Float32MultiArray [speed_m_s, steering_angle_deg]
Публикует:  /joint_states     — 4 рулевых сустава + 6 ведущих (для RViz2)
            /odom             — одометрия (nav_msgs/Odometry)
            TF:  odom → base_footprint  — позволяет роверу «ехать» в RViz2

Геометрия Аккермана для 6-колёсного ровера:
  • Передняя и задняя оси — поворотные (одинаковое направление поворота).
  • Средняя ось — неповоротная.
  • Радиус поворота центра: R = a / tan(θ).
  • Каждое колесо имеет свой радиус R_i, отсюда — своя скорость.

Оси суставов в URDF:
  • Рулевые: axis 0 1 0 (ось Y в системе рокера = вертикаль мира).
  • Ведущие:  axis 1 0 0 (ось X колеса = ось вала).
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros


# ─── Вспомогательные функции ────────────────────────────────────────────────

def euler_to_quat(yaw: float) -> Quaternion:
    """Конвертирует угол рыскания в кватернион (крен и тангаж = 0)."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


# ─── Основной класс ──────────────────────────────────────────────────────────

class AckermannSimNode(Node):

    # Имена суставов должны точно совпадать с именами в URDF!
    STEER_JOINTS = [
        'rotate_left_front_joint',   # FL — угол alpha_L
        'rotate_right_front_joint',  # FR — угол alpha_R
        'rotate_left_rear_joint',    # RL — угол beta_L
        'rotate_right_rear_joint',   # RR — угол beta_R
    ]
    DRIVE_JOINTS = [
        'wheel_left_front_joint',    # FL
        'wheel_left_middle_joint',   # ML (неповоротное)
        'wheel_left_rear_joint',     # RL
        'wheel_right_front_joint',   # FR
        'wheel_right_middle_joint',  # MR (неповоротное)
        'wheel_right_rear_joint',    # RR
    ]

    # Знаки направления вращения.
    # Из-за разной ориентации CAD-мешей левые и правые колёса могут
    # крутиться в противоположных направлениях при одинаковом знаке скорости.
    # Если в RViz2 колесо крутится назад при движении вперёд — измените знак.
    DRIVE_SIGNS = [+1, +1, +1,   # левые: FL, ML, RL
                   -1, -1, -1]   # правые: FR, MR, RR (ось X правого рокера = -X базового)

    def __init__(self):
        super().__init__('ackermann_sim')

        # ── Параметры геометрии ───────────────────────────────────────────
        self.declare_parameter('a_distance',   0.4035)  # м: передняя ось → центр
        self.declare_parameter('b_distance',   0.4035)  # м: задняя ось → центр
        self.declare_parameter('track_width',  0.779)   # м: расстояние между колёсами
        self.declare_parameter('wheel_radius', 0.097)   # м: радиус колеса
        self.declare_parameter('publish_rate', 50.0)    # Гц: частота публикации

        self.a  = self.get_parameter('a_distance').value
        self.b  = self.get_parameter('b_distance').value
        self.W  = self.get_parameter('track_width').value
        self.r  = self.get_parameter('wheel_radius').value
        rate    = self.get_parameter('publish_rate').value

        # ── Состояние симуляции ───────────────────────────────────────────
        self.cmd_speed   = 0.0   # м/с, входящая команда
        self.cmd_theta   = 0.0   # градусы, угол поворота передней оси

        # Накопленные углы вращения колёс (рад) — интегрируются по времени
        self.wheel_pos: List[float] = [0.0] * 6

        # Одометрия (поза в плоскости)
        self.odom_x   = 0.0
        self.odom_y   = 0.0
        self.odom_yaw = 0.0

        self.last_time = self.get_clock().now()

        # ── ROS-интерфейсы ────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray,
            '/motion_commands',
            self._cmd_cb,
            10
        )

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub  = self.create_publisher(Odometry,   '/odom',         10)

        # TF-broadcaster для трансформа odom → base_footprint
        self.tf_br = tf2_ros.TransformBroadcaster(self)

        # Таймер публикации
        self.create_timer(1.0 / rate, self._publish_cb)

        self.get_logger().info(
            f"AckermannSim ready | a={self.a} b={self.b} W={self.W} r={self.r} @ {rate}Hz"
        )

    # ─── Callbacks ──────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Float32MultiArray):
        """Сохраняем команду управления."""
        if len(msg.data) >= 2:
            self.cmd_speed = float(msg.data[0])
            self.cmd_theta = float(msg.data[1])

    def _publish_cb(self):
        """
        Главный таймер: вычисляет геометрию Аккермана, интегрирует положения,
        публикует joint_states, одометрию и TF.
        """
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Защита от аномальных шагов (например, при запуске или паузе)
        if dt <= 0.0 or dt > 0.5:
            return

        steer_rad, wheel_spd = self._ackermann(self.cmd_speed, self.cmd_theta)

        # Интегрируем углы вращения колёс (v = ω·r → ω = v/r)
        for i in range(6):
            self.wheel_pos[i] += (wheel_spd[i] / self.r) * self.DRIVE_SIGNS[i] * dt

        # Интегрируем одометрию (кинематика точки ICR)
        V     = self.cmd_speed
        theta = self.cmd_theta
        if abs(theta) > 0.001:
            R     = self.a / math.tan(math.radians(theta))
            omega = V / R  # угловая скорость рыскания
            self.odom_yaw -= omega * dt
            if abs(V) > 1e-4:
                self.odom_x += V * math.cos(self.odom_yaw) * dt
                self.odom_y += V * math.sin(self.odom_yaw) * dt
        else:
            self.odom_x += V * math.cos(self.odom_yaw) * dt
            self.odom_y += V * math.sin(self.odom_yaw) * dt

        stamp = now.to_msg()
        self._pub_joint_states(steer_rad, stamp)
        self._pub_odom(stamp)

    # ─── Вычисление Аккермана ────────────────────────────────────────────────

    def _ackermann(
        self,
        V: float,
        theta_deg: float
    ) -> Tuple[List[float], List[float]]:
        """
        Возвращает (steer_angles_rad [4], wheel_speeds_m_s [6]).

        Углы руления: alpha_L, alpha_R (перед), beta_L, beta_R (зад).
        Все четыре угла — одного знака: ровер с симметричным a=b
        едет как «краб», при этом геометрия Аккермана выполняется
        (все 6 колёс направлены на единый ICR).

        Если ровер крутится не в ту сторону, измените знак theta
        в строке расчёта R ниже.
        """
        if abs(V) < 1e-4:
            # Стоим: рулевые углы обнуляем плавно через публикацию нулей
            return [0.0, 0.0, 0.0, 0.0], [0.0] * 6

        if abs(theta_deg) < 0.05:
            # Прямолинейное движение
            return [0.0, 0.0, 0.0, 0.0], [V] * 6

        # Радиус поворота мгновенного центра (ICR) от продольной оси
        theta_rad = math.radians(theta_deg)
        R = self.a / math.tan(theta_rad)  # > 0: поворот вправо

        half_W = self.W / 2.0

        # ── Рулевые углы (рад) ────────────────────────────────────────────
        # atan даёт правильный знак, т.к. R меняет знак вместе с theta
        alpha_L = math.atan(self.a / (R + half_W))  # переднее левое
        alpha_R = math.atan(self.a / (R - half_W))  # переднее правое
        beta_L  = math.atan(self.b / (R + half_W))  # заднее левое
        beta_R  = math.atan(self.b / (R - half_W)) # заднее правое

        steer = [alpha_L, alpha_R, beta_L, beta_R]

        # ── Радиусы для каждого колеса ────────────────────────────────────
        R_FL = math.hypot(self.a, R + half_W)
        R_FR = math.hypot(self.a, R - half_W)
        R_ML = abs(R + half_W)
        R_MR = abs(R - half_W)
        R_RL = math.hypot(self.b, R + half_W)
        R_RR = math.hypot(self.b, R - half_W)

        # ── Скорости колёс: v_i = V * R_i / |R| ─────────────────────────
        # Знак V сохраняется (задний ход работает корректно)
        abs_R = abs(R)
        sign_V = 1.0 if V >= 0 else -1.0

        def spd(Ri: float) -> float:
            return sign_V * abs(V) * Ri / abs_R

        # Порядок: FL, ML, RL, FR, MR, RR
        speeds = [spd(R_FL), spd(R_ML), spd(R_RL),
                  spd(R_FR), spd(R_MR), spd(R_RR)]

        return steer, speeds

    # ─── Публикация ──────────────────────────────────────────────────────────

    def _pub_joint_states(self, steer_rad: List[float], stamp):
        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = ''

        js.name = self.STEER_JOINTS + self.DRIVE_JOINTS

        # Рулевые: угол в радианах
        # Ведущие: накопленный угол поворота (непрерывный)
        alpha_L, alpha_R, beta_L, beta_R = steer_rad
        
        # alpha (передние): знак ИНВЕРТИРУЕМ — ось Y рокера после поворота base_footprint
    #   на -90° указывает вертикально вверх, поэтому положительный угол = влево.
    #   Нам нужно вправо (D) = отрицательный угол.
    # beta (задние): знак НЕ ИНВЕРТИРУЕМ — задние должны быть зеркальны передним,
    #   это геометрическое требование Аккермана (все 6 колёс смотрят на общий ICR).
        js.position = [+alpha_L, +alpha_R, -beta_L, -beta_R] + list(self.wheel_pos)


        # Угловые скорости (необязательно, но полезно для отладки)
        js.velocity = [0.0] * 4 + [
            (self.cmd_speed / self.r) * s for s in self.DRIVE_SIGNS
        ]

        self.joint_pub.publish(js)

    def _pub_odom(self, stamp):
        """Публикует одометрию и TF odom → base_footprint."""
        q = euler_to_quat(self.odom_yaw)

        # Одометрия (nav_msgs/Odometry)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x  = self.odom_x
        odom.pose.pose.position.y  = self.odom_y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = self.cmd_speed
        odom.twist.twist.angular.z = (
            self.cmd_speed / (self.a / math.tan(math.radians(self.cmd_theta)))
            if abs(self.cmd_theta) > 0.05 else 0.0
        )
        self.odom_pub.publish(odom)

        # TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp            = stamp
        t.header.frame_id         = 'odom'
        t.child_frame_id          = 'base_footprint'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation      = q
        self.tf_br.sendTransform(t)


# ─── Точка входа ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AckermannSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()