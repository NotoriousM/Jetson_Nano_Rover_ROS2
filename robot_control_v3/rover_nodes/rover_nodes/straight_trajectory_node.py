#!/usr/bin/env python3
"""
STRAIGHT TRAJECTORY NODE
==========================
Автономное движение по прямой на заданную дистанцию.

ВХОД:  /odom                    [Odometry]           ← odometry_node
ВЫХОД: /motion_commands         [MotionCommand]      → flag_safety_node
       /trajectory/status       [TrajectoryStatus]   → диагностика

СЕРВИС: /start_straight_trajectory [StartStraightTrajectory]
  Вызов: ros2 service call /start_straight_trajectory \\
            rover_interfaces/srv/StartStraightTrajectory \\
            "{distance: 2.0, speed: 0.5}"

  distance > 0 — движение вперёд на N метров
  distance < 0 — движение назад на N метров
  speed > 0    — скорость движения (всегда положительная)

АЛГОРИТМ:
  1. Запоминаем стартовую позицию (x0, y0) из /odom
  2. Каждый раз публикуем команду движения с текущим знаком скорости
  3. Считаем пройденную дистанцию = sqrt((x-x0)² + (y-y0)²)
  4. По достижении target_distance — публикуем стоп

ЗАМЕДЛЕНИЕ:
  За brake_distance метров до цели — линейно снижаем скорость до brake_speed.
  Это предотвращает проскок цели из-за инерции.
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from nav_msgs.msg import Odometry

from rover_interfaces.msg import MotionCommand, TrajectoryStatus
from rover_interfaces.srv import StartStraightTrajectory


class StraightTrajectoryNode(Node):

    def __init__(self):
        super().__init__('straight_trajectory_node')

        D = ParameterType.PARAMETER_DOUBLE

        def pd(d):
            return ParameterDescriptor(description=d, type=D)

        self.declare_parameter('publish_rate_hz',     20.0,  pd('Частота публикации'))
        self.declare_parameter('brake_distance',      0.3,   pd('Дистанция замедления (м)'))
        self.declare_parameter('brake_speed',         0.1,   pd('Скорость на замедлении'))
        self.declare_parameter('finish_tolerance',    0.05,  pd('Допуск до цели (м)'))
        self.declare_parameter('max_speed',           1.5,   pd('Макс. скорость трактории'))

        self.brake_distance   = self.get_parameter('brake_distance').value
        self.brake_speed      = self.get_parameter('brake_speed').value
        self.finish_tolerance = self.get_parameter('finish_tolerance').value
        self.max_speed        = self.get_parameter('max_speed').value
        rate                  = self.get_parameter('publish_rate_hz').value

        # Состояние траектории
        self._state            = 'idle'      # idle | running | finished | aborted
        self._target_distance  = 0.0          # |distance|
        self._direction        = 1.0          # +1 или -1
        self._target_speed     = 0.0          # cruising speed
        self._start_x          = 0.0
        self._start_y          = 0.0
        self._traveled         = 0.0
        self._start_time       = self.get_clock().now()

        # Текущая позиция из /odom
        self._cur_x = 0.0
        self._cur_y = 0.0

        # Публикаторы
        self._cmd_pub    = self.create_publisher(
            MotionCommand, '/motion_commands', 10)
        self._status_pub = self.create_publisher(
            TrajectoryStatus, '/trajectory/status', 10)

        # Подписка на одометрию
        self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )

        # Сервис запуска
        self.create_service(
            StartStraightTrajectory,
            '/start_straight_trajectory',
            self._start_service_callback,
        )

        # Таймер публикации
        self.create_timer(1.0 / rate, self._control_callback)

        self.get_logger().info(
            'StraightTrajectoryNode ready\n'
            '  Service: /start_straight_trajectory\n'
            '  Status:  /trajectory/status\n'
            '  Example: ros2 service call /start_straight_trajectory '
            'rover_interfaces/srv/StartStraightTrajectory '
            '"{distance: 2.0, speed: 0.5}"'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        """Обновляем текущую позицию."""
        self._cur_x = msg.pose.pose.position.x
        self._cur_y = msg.pose.pose.position.y

        # Пересчитываем пройденную дистанцию если идёт траектория
        if self._state == 'running':
            dx = self._cur_x - self._start_x
            dy = self._cur_y - self._start_y
            self._traveled = math.sqrt(dx * dx + dy * dy)

    def _start_service_callback(
        self,
        req: StartStraightTrajectory.Request,
        res: StartStraightTrajectory.Response,
    ) -> StartStraightTrajectory.Response:
        """Запуск траектории."""

        # Валидация
        if self._state == 'running':
            res.success = False
            res.message = 'Trajectory already running. Send stop first.'
            return res

        if abs(req.distance) < 0.01:
            res.success = False
            res.message = f'Distance too small: {req.distance}'
            return res

        if req.speed <= 0 or req.speed > self.max_speed:
            res.success = False
            res.message = f'Speed must be 0 < speed ≤ {self.max_speed}, got {req.speed}'
            return res

        # Запоминаем старт
        self._target_distance = abs(float(req.distance))
        self._direction       = 1.0 if req.distance > 0 else -1.0
        self._target_speed    = float(req.speed)
        self._start_x         = self._cur_x
        self._start_y         = self._cur_y
        self._traveled        = 0.0
        self._start_time      = self.get_clock().now()
        self._state           = 'running'

        msg_text = (
            f'Started: distance={req.distance:.2f}m '
            f'(|d|={self._target_distance:.2f}, dir={"+":>2}{int(self._direction):d}) '
            f'speed={req.speed:.2f}m/s '
            f'from ({self._start_x:.2f}, {self._start_y:.2f})'
        )
        self.get_logger().info(msg_text)

        res.success = True
        res.message = msg_text
        return res

    def _control_callback(self):
        """20 Гц управление движением."""

        if self._state != 'running':
            # Публикуем статус (idle/finished/aborted)
            self._publish_status()
            return

        remaining = self._target_distance - self._traveled

        # ── Достигли цели ────────────────────────────────────────────────────
        if remaining <= self.finish_tolerance:
            self._state = 'finished'
            self.get_logger().info(
                f'Trajectory FINISHED: traveled {self._traveled:.3f}m '
                f'(target {self._target_distance:.3f}m)'
            )
            stop = MotionCommand()
            stop.source = 'trajectory'
            self._cmd_pub.publish(stop)
            self._publish_status()
            return

        # ── Профиль скорости с замедлением ───────────────────────────────────
        if remaining <= self.brake_distance:
            # Линейная интерполяция от target_speed до brake_speed
            t = remaining / self.brake_distance     # 0..1
            speed = self.brake_speed + (self._target_speed - self.brake_speed) * t
        else:
            speed = self._target_speed

        # Применяем направление
        signed_speed = speed * self._direction

        # Публикуем команду движения
        cmd = MotionCommand()
        cmd.linear_velocity = float(signed_speed)
        cmd.steering_angle  = 0.0     # прямо
        cmd.source          = 'trajectory'
        self._cmd_pub.publish(cmd)

        self._publish_status()

    def _publish_status(self):
        """Публикуем статус выполнения."""
        msg = TrajectoryStatus()
        msg.state              = self._state
        msg.target_distance    = float(self._target_distance * self._direction)
        msg.traveled_distance  = float(self._traveled * self._direction)
        msg.remaining_distance = float(
            max(0.0, self._target_distance - self._traveled) * self._direction
        )
        msg.progress_percent   = (
            min(100.0, 100.0 * self._traveled / self._target_distance)
            if self._target_distance > 0 else 0.0
        )
        msg.elapsed_time = (
            (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        )
        msg.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StraightTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
