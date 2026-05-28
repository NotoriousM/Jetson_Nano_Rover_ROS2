#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class FigureEightRViz(Node):

    def __init__(self):
        super().__init__('figure_eight')

        self.declare_parameter('linear_speed',  0.5)
        self.declare_parameter('scale',         2.5)
        self.declare_parameter('loop_count',    2)
        self.declare_parameter('a_distance',    0.4035)
        self.declare_parameter('max_steer_deg', 30.0)
        self.declare_parameter('publish_rate',  20.0)
        self.declare_parameter('delay_sec',     10.0)

        self.v         = self.get_parameter('linear_speed').value
        self.R         = self.get_parameter('scale').value
        self.loops     = self.get_parameter('loop_count').value
        self.a_dist    = self.get_parameter('a_distance').value
        self.max_steer = self.get_parameter('max_steer_deg').value
        rate           = self.get_parameter('publish_rate').value
        self.delay     = self.get_parameter('delay_sec').value

        self.steer_deg = math.degrees(math.atan(self.a_dist / self.R))
        self.steer_deg = min(self.steer_deg, self.max_steer)
        actual_R = self.a_dist / math.tan(math.radians(self.steer_deg))
        self.T_circle = 2.0 * math.pi * actual_R / self.v

        self._started     = False
        self._start_time  = None
        self._stopped     = False
        self._launch_time = self.get_clock().now().nanoseconds * 1e-9

        self.pub = self.create_publisher(Float32MultiArray, '/motion_commands', 10)
        self._timer = self.create_timer(1.0 / rate, self._cb)

        self.get_logger().info(
            f"\n"
            f"  ╔═══════════════════════════════════════════╗\n"
            f"  ║   Восьмёрка — два круга (RViz2)           ║\n"
            f"  ╠═══════════════════════════════════════════╣\n"
            f"  ║  Скорость:    {self.v:.2f} м/с                  ║\n"
            f"  ║  Радиус:      {actual_R:.2f} м                    ║\n"
            f"  ║  Угол руля:   {self.steer_deg:.1f}°                    ║\n"
            f"  ║  T_круга:     {self.T_circle:.1f} с                   ║\n"
            f"  ║  Петли:       {self.loops}                           ║\n"
            f"  ║  СТАРТ через: {self.delay:.0f} с                   ║\n"
            f"  ╚═══════════════════════════════════════════╝\n"
            f"  Настройте RViz2: Fixed Frame=odom, Add→Path→/rover_path"
        )

    def _cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if not self._started:
            remaining = self.delay - (now - self._launch_time)
            if remaining > 0:
                self._publish(0.0, 0.0)
                self.get_logger().info(
                    f"Старт через {remaining:.0f} с...",
                    throttle_duration_sec=1.0)
                return
            else:
                self._start_time = now
                self._started = True
                self.get_logger().info(
                    f"▶ СТАРТ! Правый → Левый × {self.loops} | "
                    f"итого {self.loops*2*self.T_circle:.0f} с")

        t = now - self._start_time
        T_eight = 2.0 * self.T_circle
        total   = self.loops * T_eight

        if t >= total:
            if not self._stopped:
                self._publish(0.0, 0.0)
                self._stopped = True
                if self._timer:
                    self._timer.cancel()
                self.get_logger().info("■ Восьмёрка завершена!")
            return

        phase = t % T_eight
        steer = +self.steer_deg if phase < self.T_circle else -self.steer_deg
        side  = "→ Правый" if phase < self.T_circle else "← Левый"

        self.get_logger().info(
            f"{side} | {t/total*100:.0f}% | руль {steer:+.1f}°",
            throttle_duration_sec=5.0)

        self._publish(self.v, steer)

    def _publish(self, speed, steer):
        msg = Float32MultiArray()
        msg.data = [float(speed), float(steer)]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FigureEightRViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()