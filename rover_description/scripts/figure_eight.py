#!/usr/bin/env python3
"""
figure_eight.py — Скрипт движения по траектории «восьмёрка».

Запускается ОТДЕЛЬНО после того, как Gazebo уже запущена и робот заспавнен:
  ros2 run rover_description figure_eight
  ros2 run rover_description figure_eight --ros-args -p linear_speed:=0.3 -p scale:=3.0

Параметры:
  linear_speed  — скорость движения (м/с),      по умолчанию 0.4
  scale         — масштаб петли (м),             по умолчанию 2.0
  loop_count    — количество повторений,         по умолчанию 3
  a_distance    — передняя база Аккермана (м),   по умолчанию 0.4035
  max_steer_deg — макс. угол руля (°),           по умолчанию 35.0

Публикует:
  /cmd_vel              [Twist]         → Gazebo planar_move плагин
  /motion_commands_safe [MotionCommand] → ackermann_calculator_node (если доступен)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from rover_interfaces.msg import MotionCommand
    HAS_MOTION_CMD = True
except ImportError:
    HAS_MOTION_CMD = False


class FigureEightNode(Node):

    def __init__(self):
        super().__init__('figure_eight_node')

        # ── Параметры ─────────────────────────────────────────────
        self.declare_parameter('linear_speed',  0.4)
        self.declare_parameter('scale',         2.0)
        self.declare_parameter('loop_count',    3)
        self.declare_parameter('a_distance',    0.4035)
        self.declare_parameter('max_steer_deg', 35.0)

        self.v         = self.get_parameter('linear_speed').value
        self.a_scale   = self.get_parameter('scale').value
        self.loops     = self.get_parameter('loop_count').value
        self.a_dist    = self.get_parameter('a_distance').value
        self.max_steer = self.get_parameter('max_steer_deg').value

        # Период одного цикла: длина лемнискаты ≈ 4·a·1.2111
        self.T_cycle = 4.0 * self.a_scale * 1.2111 / self.v

        # ── Издатели ──────────────────────────────────────────────
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel', 10)

        if HAS_MOTION_CMD:
            self.pub_motion = self.create_publisher(
                MotionCommand, '/motion_commands_safe', 10)
        else:
            self.pub_motion = None
            self.get_logger().warn(
                'rover_interfaces не найден — публикуется только /cmd_vel')

        # ── Таймер 20 Гц ──────────────────────────────────────────
        self._t0      = self.get_clock().now().nanoseconds * 1e-9
        self._t_end   = self._t0 + self.loops * self.T_cycle
        self._stopped = False
        self._timer   = self.create_timer(0.05, self._cb)

        self.get_logger().info(
            f'\n'
            f'  ╔══ figure_eight (Ackermann) ══╗\n'
            f'  ║  v         = {self.v} м/с\n'
            f'  ║  scale     = {self.a_scale} м\n'
            f'  ║  T_cycle   = {self.T_cycle:.1f} с\n'
            f'  ║  loops     = {self.loops}\n'
            f'  ║  total     = {self.loops * self.T_cycle:.0f} с\n'
            f'  ╚═══════════════════════════════╝'
        )

    # ─────────────────────────────────────────────────────────────
    def _cb(self):
        t_now = self.get_clock().now().nanoseconds * 1e-9

        if t_now >= self._t_end:
            if not self._stopped:
                self._stop()
            return

        s = ((t_now - self._t0) / self.T_cycle) * 2.0 * math.pi
        a = self.a_scale

        # Лемниската Бернулли — первые и вторые производные
        dx  =  a * math.cos(s)
        dy  =  a * math.cos(2.0 * s)
        ddx = -a * math.sin(s)
        ddy = -2.0 * a * math.sin(2.0 * s)

        spd = math.hypot(dx, dy)
        if spd < 1e-9:
            spd = 1e-9

        # Кривизна и угловая скорость
        kappa = (dx * ddy - dy * ddx) / (spd ** 3)
        omega = self.v * kappa

        # Угол рулевого колеса (Ackermann)
        if abs(kappa) > 1e-6:
            R     = 1.0 / kappa
            steer = math.degrees(math.atan2(self.a_dist, R))
            steer = max(-self.max_steer, min(self.max_steer, steer))
        else:
            steer = 0.0

        # → /cmd_vel (Gazebo)
        tw = Twist()
        tw.linear.x  = float(self.v)
        tw.angular.z = float(omega)
        self.pub_twist.publish(tw)

        # → /motion_commands_safe (реальный робот)
        if self.pub_motion:
            mc = MotionCommand()
            mc.linear_velocity = float(self.v)
            mc.steering_angle  = float(steer)
            mc.source          = 'figure_eight'
            self.pub_motion.publish(mc)

    def _stop(self):
        self.pub_twist.publish(Twist())
        if self.pub_motion:
            mc = MotionCommand()
            mc.linear_velocity = 0.0
            mc.steering_angle  = 0.0
            mc.source          = 'figure_eight'
            self.pub_motion.publish(mc)
        self._stopped = True
        self._timer.cancel()
        self.get_logger().info('✓ Траектория завершена. Робот остановлен.')


def main(args=None):
    rclpy.init(args=args)
    node = FigureEightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node._stopped:
            node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
