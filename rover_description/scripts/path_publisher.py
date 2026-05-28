#!/usr/bin/env python3
"""
path_publisher.py — Аккумулятор траектории для RViz2.
Подписывается на /odom и публикует nav_msgs/Path в /rover_path.
Добавьте в RViz2 дисплей Path → Topic: /rover_path чтобы видеть
непрерывную линию траектории движения ровера.

Запуск: ros2 run rover_description path_publisher.py
Сброс траектории: ros2 topic pub --once /reset_path std_msgs/Bool '{data: true}'
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

        # Параметры
        self.declare_parameter('max_poses', 2000)    # Максимум точек в памяти
        self.declare_parameter('min_distance', 0.02) # Минимальное расстояние (м) между точками
                                                     # Слишком маленькое → тормоза, слишком большое → рваная линия

        self.max_poses   = self.get_parameter('max_poses').value
        self.min_dist    = self.get_parameter('min_distance').value

        # Хранилище траектории
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_x = None
        self.last_y = None

        # Подписка на одометрию
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Подписка на сброс траектории
        self.create_subscription(Bool, '/reset_path', self._reset_cb, 10)

        # Публикация пути
        self.path_pub = self.create_publisher(Path, '/rover_path', 10)

        # Таймер: публикуем путь с частотой 10 Гц (хватает для визуализации)
        self.create_timer(0.1, self._publish_path)

        self.get_logger().info(
            f"PathPublisher ready | max_poses={self.max_poses} | "
            f"min_distance={self.min_dist} м"
        )
        self.get_logger().info(
            "Добавьте в RViz2: Add → Path → Topic: /rover_path"
        )
        self.get_logger().info(
            "Сброс: ros2 topic pub --once /reset_path std_msgs/Bool '{data: true}'"
        )

    def _odom_cb(self, msg: Odometry):
        """Добавляем новую точку в путь если робот достаточно сдвинулся."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Пропускаем если сдвиг меньше min_distance (экономим память)
        if self.last_x is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.min_dist:
                return

        # Создаём PoseStamped и добавляем в путь
        pose = PoseStamped()
        pose.header.stamp    = msg.header.stamp
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose

        self.path.poses.append(pose)

        # Обрезаем если слишком длинный (скользящее окно)
        if len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses:]

        self.last_x = x
        self.last_y = y

    def _reset_cb(self, msg: Bool):
        """Сброс траектории."""
        if msg.data:
            self.path.poses.clear()
            self.last_x = None
            self.last_y = None
            self.get_logger().info("Траектория сброшена")

    def _publish_path(self):
        """Публикуем текущий путь."""
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()