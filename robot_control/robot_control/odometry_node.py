#!/usr/bin/env python3
"""
Нода 4: Одометрия по средним (неповоротным) колёсам.
Вход:  /wheel/middle_left/speed   Float32
       /wheel/middle_right/speed  Float32
Выход: /odom                      Odometry
       TF: odom → base_link
"""
import math
from typing import Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        # Параметры
        def dp(desc, ptype):
            return ParameterDescriptor(description=desc, type=ptype)

        DPD = ParameterType.PARAMETER_DOUBLE
        DPI = ParameterType.PARAMETER_INTEGER
        DPS = ParameterType.PARAMETER_STRING

        self.declare_parameter('track_width',             0.779, dp("Ширина колеи (м)", DPD))
        self.declare_parameter('wheel_radius',            0.1,   dp("Радиус колеса (м)", DPD))
        self.declare_parameter('speed_conversion_factor', 1.0,   dp("Коэф. перевода к м/с", DPD))
        self.declare_parameter('publish_rate',            50,    dp("Частота публикации Гц", DPI))
        self.declare_parameter('odom_frame_id',   'odom',        dp("Фрейм odom", DPS))
        self.declare_parameter('base_frame_id',   'base_link',   dp("Фрейм base_link", DPS))

        self.W      = self.get_parameter('track_width').value
        self.r      = self.get_parameter('wheel_radius').value
        self.k_cvt  = self.get_parameter('speed_conversion_factor').value
        self.rate   = self.get_parameter('publish_rate').value
        self.odom_f = self.get_parameter('odom_frame_id').value
        self.base_f = self.get_parameter('base_frame_id').value

        # Состояние одометрии
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.vtheta = 0.0
        self.v_left = self.v_right = 0.0
        self.last_time = self.get_clock().now()

        # Ковариации (постоянные)
        self.pose_cov  = list(np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.02]).flatten())
        self.twist_cov = list(np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.02]).flatten())

        # QoS
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Публикаторы
        self.odom_pub  = self.create_publisher(Odometry, '/odom', qos)
        self.tf_bcast  = TransformBroadcaster(self)

        # Подписки
        self.create_subscription(Float32, '/wheel/middle_left/speed',
                                 lambda m: self._update('left',  m.data * self.k_cvt), 10)
        self.create_subscription(Float32, '/wheel/middle_right/speed',
                                 lambda m: self._update('right', m.data * self.k_cvt), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self._reset_pose, 10)

        # Таймеры
        self.create_timer(1.0 / self.rate, self._publish)
        self.create_timer(2.0, self._diagnostics)

        self.get_logger().info(
            f"OdometryNode ready | W={self.W}m, r={self.r}m, {self.rate}Hz"
        )

    # ── Callbacks ──────────────────────────────────────────────────
    def _update(self, side: str, speed: float):
        if side == 'left':
            self.v_left  = speed
        else:
            self.v_right = speed
        self._integrate()

    def _integrate(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        if dt < 1e-6:
            return

        vL, vR = self.v_left, self.v_right
        self.vx     = (vR + vL) / 2.0
        self.vtheta = (vR - vL) / self.W

        self.yaw += self.vtheta * dt
        self.yaw  = self._norm_angle(self.yaw)

        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        self.last_time = now

    def _reset_pose(self, msg: PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        _, _, self.yaw = self._quat_to_euler(o.x, o.y, o.z, o.w)
        self.vx = self.vy = self.vtheta = 0.0
        self.get_logger().info(
            f"Pose reset: x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}°"
        )

    # ── Timer ──────────────────────────────────────────────────────
    def _publish(self):
        now = self.get_clock().now()
        q   = self._euler_to_quat(0.0, 0.0, self.yaw)

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.odom_f
        odom.child_frame_id  = self.base_f

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance         = self.pose_cov

        odom.twist.twist.linear.x    = self.vx
        odom.twist.twist.angular.z   = self.vtheta
        odom.twist.covariance        = self.twist_cov

        self.odom_pub.publish(odom)

        # TF transform
        tf = TransformStamped()
        tf.header.stamp    = now.to_msg()
        tf.header.frame_id = self.odom_f
        tf.child_frame_id  = self.base_f
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_bcast.sendTransform(tf)

    def _diagnostics(self):
        dist = math.sqrt(self.x**2 + self.y**2)
        self.get_logger().info(
            f"x={self.x:.3f}m y={self.y:.3f}m θ={math.degrees(self.yaw):.1f}° "
            f"v={self.vx:.3f}m/s ω={math.degrees(self.vtheta):.1f}°/s dist={dist:.2f}m",
            throttle_duration_sec=2.0
        )

    # ── Static helpers ─────────────────────────────────────────────
    @staticmethod
    def _norm_angle(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    @staticmethod
    def _euler_to_quat(r, p, y) -> Tuple[float, float, float, float]:
        cy, sy = math.cos(y/2), math.sin(y/2)
        cp, sp = math.cos(p/2), math.sin(p/2)
        cr, sr = math.cos(r/2), math.sin(r/2)
        return (sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy,
                cr*cp*cy + sr*sp*sy)

    @staticmethod
    def _quat_to_euler(x, y, z, w) -> Tuple[float, float, float]:
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        sinp  = 2*(w*y - z*x)
        pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw


# ──────────────────────────────────────────────────────────────────
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