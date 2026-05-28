#!/usr/bin/env python3
"""
ROVER STATUS NODE — УДОБНЫЙ МОНИТОРИНГ
========================================
Агрегирует данные со всех узлов в один топик /rover/status.
Это удобно для отладки и мониторинга — одна команда показывает всё:

  ros2 topic echo /rover/status

ВХОДЫ:
  /odom                    [Odometry]              ← odometry_node
  /wheels/state            [RoverWheelsState]      ← serial_controller_node
  /safety/active           [Bool]                  ← flag_safety_node
  /safety/active_flags     [String]                ← flag_safety_node
  /motion_commands_safe    [MotionCommand]         ← flag_safety_node (для source)
  /trajectory/status       [TrajectoryStatus]      ← straight_trajectory_node

ВЫХОД:
  /rover/status            [RoverStatus]           агрегированный статус 1 Гц
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry

from rover_interfaces.msg import (
    RoverWheelsState,
    MotionCommand,
    RoverStatus,
    TrajectoryStatus,
)


class RoverStatusNode(Node):

    def __init__(self):
        super().__init__('rover_status_node')

        # Кэшируем последние данные
        self._x = self._y = self._yaw = 0.0
        self._speed = 0.0
        self._steering = 0.0
        self._distance_traveled = 0.0
        self._wheels_connected = 0
        self._safety_active   = False
        self._safety_flags    = 'OK'
        self._active_source   = 'none'
        self._trajectory_state = 'idle'
        self._start_x = 0.0
        self._start_y = 0.0
        self._first_odom = True

        # QoS BEST_EFFORT (та же что у одометрии)
        qos_be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Подписки
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, qos_be)
        self.create_subscription(
            RoverWheelsState, '/wheels/state', self._wheels_cb, 10)
        self.create_subscription(
            Bool, '/safety/active', self._safety_cb, 10)
        self.create_subscription(
            String, '/safety/active_flags', self._safety_flags_cb, 10)
        self.create_subscription(
            MotionCommand, '/motion_commands_safe', self._cmd_cb, 10)
        self.create_subscription(
            TrajectoryStatus, '/trajectory/status', self._traj_cb, 10)

        # Публикатор
        self._status_pub = self.create_publisher(RoverStatus, '/rover/status', 10)

        # 1 Гц публикация + красивый вывод в консоль
        self.create_timer(1.0, self._publish_callback)

        self.get_logger().info(
            'RoverStatusNode ready\n'
            '  Aggregates: /odom, /wheels/state, /safety/*,\n'
            '              /motion_commands_safe, /trajectory/status\n'
            '  Publishes:  /rover/status [RoverStatus] (1 Hz)\n'
            '  Watch with: ros2 topic echo /rover/status'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        if self._first_odom:
            self._start_x = msg.pose.pose.position.x
            self._start_y = msg.pose.pose.position.y
            self._first_odom = False

        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        o = msg.pose.pose.orientation
        siny = 2 * (o.w * o.z + o.x * o.y)
        cosy = 1 - 2 * (o.y * o.y + o.z * o.z)
        self._yaw = math.atan2(siny, cosy)

        # Дистанция от начала (по прямой)
        dx = self._x - self._start_x
        dy = self._y - self._start_y
        self._distance_traveled = math.sqrt(dx*dx + dy*dy)

    def _wheels_cb(self, msg: RoverWheelsState):
        self._wheels_connected = msg.connected_count

    def _safety_cb(self, msg: Bool):
        self._safety_active = bool(msg.data)

    def _safety_flags_cb(self, msg: String):
        self._safety_flags = msg.data

    def _cmd_cb(self, msg: MotionCommand):
        # Команды от safety не должны показывать активный источник
        if msg.source not in ('safety', ''):
            self._active_source = msg.source
        self._speed = msg.linear_velocity
        self._steering = msg.steering_angle

    def _traj_cb(self, msg: TrajectoryStatus):
        self._trajectory_state = msg.state

    def _publish_callback(self):
        """1 Гц агрегированный статус."""
        # Определяем общее состояние
        if self._safety_active:
            state = 'safety_block'
        elif self._trajectory_state == 'running':
            state = 'trajectory'
        elif abs(self._speed) > 0.01:
            state = 'driving'
        else:
            state = 'idle'

        # Публикация
        msg = RoverStatus()
        msg.state              = state
        msg.safety_active      = self._safety_active
        msg.safety_flags       = self._safety_flags
        msg.active_source      = self._active_source
        msg.wheels_connected   = self._wheels_connected
        msg.current_speed      = float(self._speed)
        msg.current_steering   = float(self._steering)
        msg.distance_traveled  = float(self._distance_traveled)
        msg.stamp              = self.get_clock().now().to_msg()
        self._status_pub.publish(msg)

        # Красивый вывод в логи
        sym = 'BLOCK' if self._safety_active else 'OK   '
        self.get_logger().info(
            f'\n'
            f'  state={state:<14s}  source={self._active_source}\n'
            f'  pos=({self._x:.2f}, {self._y:.2f}) '
            f'yaw={math.degrees(self._yaw):.1f}° dist={self._distance_traveled:.2f}m\n'
            f'  speed={self._speed:.2f}m/s  steer={self._steering:.1f}°\n'
            f'  Safety [{sym}]: {self._safety_flags}\n'
            f'  Wheels connected: {self._wheels_connected}/6'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RoverStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
