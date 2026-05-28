#!/usr/bin/env python3
"""
ODOMETRY NODE
==============
Дифференциальная одометрия по средним (неповоротным) колёсам.

ВХОД:  /wheels/state  [RoverWheelsState]    ← serial_controller_node
       /initialpose   [PoseWithCovStamped]  ← RViz (кнопка 2D Pose Estimate)
ВЫХОД: /odom          [nav_msgs/Odometry]   → Nav2, RViz, диагностика
       /tf            (odom → base_link)    → tf2_ros TransformBroadcaster
СЕРВИС: /reset_odometry [ResetOdometry]     ← внешний сброс

Использует именованные поля msg.middle_left.speed / msg.middle_right.speed —
исключает ошибку перепутанных индексов из старой версии с Float32MultiArray.
"""

import math
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster

from rover_interfaces.msg import RoverWheelsState
from rover_interfaces.srv import ResetOdometry


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')
        self._declare_params()

        self.W       = self.get_parameter('track_width').value
        self.k_cvt   = self.get_parameter('speed_conversion_factor').value
        self.rate    = self.get_parameter('publish_rate').value
        self.f_odom  = self.get_parameter('odom_frame_id').value
        self.f_base  = self.get_parameter('base_frame_id').value

        # ── Состояние ────────────────────────────────────────────────────────
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vw = 0.0
        self.v_left = self.v_right = 0.0
        self.distance_traveled = 0.0
        self.last_time = self.get_clock().now()

        # Ковариации
        self.pose_cov  = list(np.diag([0.01, 0.01, 1e6, 1e6, 1e6, 0.02]).flatten())
        self.twist_cov = list(np.diag([0.01, 0.01, 1e6, 1e6, 1e6, 0.02]).flatten())

        # ── QoS ──────────────────────────────────────────────────────────────
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Публикаторы ──────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self._tf_bcast = TransformBroadcaster(self)

        # ── Подписки ─────────────────────────────────────────────────────────
        self.create_subscription(
            RoverWheelsState, '/wheels/state',
            self._wheels_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self._initial_pose_callback, 10
        )

        # ── Сервис сброса ────────────────────────────────────────────────────
        self.create_service(
            ResetOdometry, '/reset_odometry', self._reset_callback
        )

        # ── Таймер ───────────────────────────────────────────────────────────
        self.create_timer(1.0 / self.rate, self._publish_callback)
        self.create_timer(2.0, self._diag_callback)

        self.get_logger().info(
            f'OdometryNode ready\n'
            f'  Subscribes: /wheels/state, /initialpose\n'
            f'  Publishes:  /odom, /tf (odom→base_link)\n'
            f'  Service:    /reset_odometry\n'
            f'  Params: W={self.W}m  rate={self.rate}Hz'
        )

    def _declare_params(self):
        D = ParameterType.PARAMETER_DOUBLE
        I = ParameterType.PARAMETER_INTEGER
        S = ParameterType.PARAMETER_STRING

        def pd(d): return ParameterDescriptor(description=d, type=D)
        def pi(d): return ParameterDescriptor(description=d, type=I)
        def ps(d): return ParameterDescriptor(description=d, type=S)

        self.declare_parameter('track_width',             0.779, pd('Ширина колеи (м)'))
        self.declare_parameter('wheel_radius',            0.1,   pd('Радиус колеса (м)'))
        self.declare_parameter('speed_conversion_factor', 1.0,   pd('Коэф. перевода → м/с'))
        self.declare_parameter('publish_rate',            50,    pi('Частота /odom (Гц)'))
        self.declare_parameter('odom_frame_id', 'odom',          ps('Родительский фрейм'))
        self.declare_parameter('base_frame_id', 'base_link',     ps('Дочерний фрейм'))

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _wheels_callback(self, msg: RoverWheelsState):
        """
        Получаем состояние всех колёс. Используем СРЕДНИЕ колёса —
        они неповоротные и дают чистую дифференциальную модель.

        КРИТИЧНО: используем именованные поля.
          БЫЛО:  v = msg.data[1]                    ← легко ошибиться
          СТАЛО: v = msg.middle_left.speed          ← невозможно перепутать
        """
        if msg.middle_left.is_connected:
            self.v_left  = msg.middle_left.speed * self.k_cvt
        if msg.middle_right.is_connected:
            self.v_right = msg.middle_right.speed * self.k_cvt

        if msg.connected_count < 6:
            self.get_logger().warn(
                f'Only {msg.connected_count}/6 wheels connected',
                throttle_duration_sec=5.0,
            )

        self._integrate()

    def _integrate(self):
        """Дифференциальная модель + интеграция методом средней точки."""
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        if dt < 1e-6 or dt > 1.0:
            self.last_time = now
            return

        vL, vR = self.v_left, self.v_right
        self.vx = (vR + vL) / 2.0
        self.vw = (vR - vL) / self.W

        # Метод средней точки для повышенной точности
        d_yaw   = self.vw * dt
        mid_yaw = self.yaw + d_yaw / 2.0
        self.yaw = self._norm_angle(self.yaw + d_yaw)

        dx = self.vx * math.cos(mid_yaw) * dt
        dy = self.vx * math.sin(mid_yaw) * dt
        self.x += dx
        self.y += dy

        # Накопленная дистанция (не зависит от направления)
        self.distance_traveled += math.sqrt(dx*dx + dy*dy)
        self.last_time = now

    def _publish_callback(self):
        """50 Гц публикация /odom + /tf."""
        now = self.get_clock().now()
        q = self._yaw_to_quat(self.yaw)

        # /odom
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.f_odom
        odom.child_frame_id  = self.f_base
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance       = self.pose_cov
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.angular.z = self.vw
        odom.twist.covariance      = self.twist_cov
        self._odom_pub.publish(odom)

        # TF: odom → base_link
        tf = TransformStamped()
        tf.header.stamp    = now.to_msg()
        tf.header.frame_id = self.f_odom
        tf.child_frame_id  = self.f_base
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self._tf_bcast.sendTransform(tf)

    def _diag_callback(self):
        self.get_logger().info(
            f'Odom: x={self.x:.3f}m y={self.y:.3f}m yaw={math.degrees(self.yaw):.1f}° '
            f'v={self.vx:.3f}m/s ω={math.degrees(self.vw):.1f}°/s '
            f'dist={self.distance_traveled:.2f}m',
            throttle_duration_sec=2.0,
        )

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Кнопка '2D Pose Estimate' в RViz."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        _, _, self.yaw = self._quat_to_euler(o.x, o.y, o.z, o.w)
        self.vx = self.vw = 0.0
        self.get_logger().info(
            f'Pose from RViz: x={self.x:.2f} y={self.y:.2f} '
            f'yaw={math.degrees(self.yaw):.1f}°'
        )

    def _reset_callback(
        self,
        req: ResetOdometry.Request,
        res: ResetOdometry.Response,
    ) -> ResetOdometry.Response:
        """Сервис /reset_odometry — гарантированный сброс с подтверждением."""
        try:
            self.x = float(req.x)
            self.y = float(req.y)
            self.yaw = math.radians(float(req.yaw_deg))
            self.vx = self.vw = 0.0
            self.distance_traveled = 0.0
            res.success = True
            res.message = (
                f'Reset OK: x={self.x:.3f}m y={self.y:.3f}m '
                f'yaw={req.yaw_deg:.1f}°'
            )
            self.get_logger().info(res.message)
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _norm_angle(a: float) -> float:
        while a > math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    @staticmethod
    def _yaw_to_quat(y: float) -> Tuple[float, float, float, float]:
        h = y * 0.5
        return 0.0, 0.0, math.sin(h), math.cos(h)

    @staticmethod
    def _quat_to_euler(x, y, z, w) -> Tuple[float, float, float]:
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        sinp  = 2*(w*y - z*x)
        pitch = (math.copysign(math.pi/2, sinp)
                 if abs(sinp) >= 1 else math.asin(sinp))
        yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
