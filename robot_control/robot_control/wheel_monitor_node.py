#!/usr/bin/env python3
"""
Нода 3: Мониторинг и диагностика колёс.
Вход:  /wheel/{name}/speed  Float32  (×6)
       /wheel/{name}/flag   UInt8    (×6)
       /wheel/{name}/angle  Float32  (×6)
Выход: только логи + /wheels/diagnostics String (JSON)
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, UInt8

WHEEL_NAMES = [
    'front_left', 'middle_left', 'rear_left',
    'front_right', 'middle_right', 'rear_right'
]


class WheelMonitorNode(Node):
    def __init__(self):
        super().__init__('wheel_monitor')

        self.declare_parameter('speed_warn_threshold', 5.0)   # м/с
        self.declare_parameter('stall_flag_value',     1)     # флаг застревания
        self.declare_parameter('report_rate',          2.0)   # Гц

        self.speed_warn = self.get_parameter('speed_warn_threshold').value
        self.stall_flag = self.get_parameter('stall_flag_value').value

        # Хранилище состояния
        self.state = {
            name: {'speed': 0.0, 'flag': 0, 'angle': 90.0, 'last_update': 0.0}
            for name in WHEEL_NAMES
        }

        # Подписки на каждое колесо
        for name in WHEEL_NAMES:
            self.create_subscription(
                Float32, f'/wheel/{name}/speed',
                lambda msg, n=name: self._on_speed(n, msg), 10
            )
            self.create_subscription(
                UInt8, f'/wheel/{name}/flag',
                lambda msg, n=name: self._on_flag(n, msg), 10
            )
            self.create_subscription(
                Float32, f'/wheel/{name}/angle',
                lambda msg, n=name: self._on_angle(n, msg), 10
            )

        # Публикатор диагностики
        self.diag_pub = self.create_publisher(String, '/wheels/diagnostics', 10)

        rate = self.get_parameter('report_rate').value
        self.create_timer(1.0 / rate, self._report)

        self.get_logger().info("WheelMonitor started")

    # ── Callbacks ──────────────────────────────────────────────────
    def _on_speed(self, name: str, msg: Float32):
        self.state[name]['speed'] = msg.data
        self.state[name]['last_update'] = time.time()

        if abs(msg.data) > self.speed_warn:
            self.get_logger().warn(
                f"{name}: speed={msg.data:.2f} m/s exceeds threshold {self.speed_warn}",
                throttle_duration_sec=2.0
            )

    def _on_flag(self, name: str, msg: UInt8):
        prev = self.state[name]['flag']
        self.state[name]['flag'] = msg.data
        if msg.data == self.stall_flag and prev != self.stall_flag:
            self.get_logger().warn(f"⚠️  {name}: STALL detected (flag={msg.data})")

    def _on_angle(self, name: str, msg: Float32):
        self.state[name]['angle'] = msg.data

    # ── Timer ──────────────────────────────────────────────────────
    def _report(self):
        now = time.time()
        report = {}
        for name, s in self.state.items():
            age = now - s['last_update'] if s['last_update'] > 0 else -1
            report[name] = {
                'speed': round(s['speed'], 3),
                'flag':  s['flag'],
                'angle': round(s['angle'], 1),
                'data_age_sec': round(age, 2)
            }

        msg = String()
        msg.data = json.dumps(report, ensure_ascii=False)
        self.diag_pub.publish(msg)


# ──────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = WheelMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()