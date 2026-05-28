#!/usr/bin/env python3
"""
figure_eight_rviz.py — Автономное движение ровера по траектории «восьмёрка».
=========================================================================
Публикует в /motion_commands (Float32MultiArray [speed_m_s, steering_angle_deg])
которые потребляет ackermann_sim_node.py. Газебо не нужен.

Алгоритм — параметрическая кривая Лиссажу (фигура «восьмёрка»):
  x(s) = a * sin(s)           — продольная координата
  y(s) = a * sin(2s) / 2      — поперечная координата

  где s ∈ [0, 2π] — один полный цикл «восьмёрки».

Кривизна вычисляется аналитически:
  κ = (dx·d²y − dy·d²x) / |v|³

  κ > 0: кривая поворачивает ВЛЕВО  → steering_angle < 0 (наша конвенция)
  κ < 0: кривая поворачивает ВПРАВО → steering_angle > 0

Знаковая конвенция /motion_commands (проверена в нашей симуляции):
  steering_angle > 0 → поворот ВПРАВО (D на клавиатуре)
  steering_angle < 0 → поворот ВЛЕВО  (A на клавиатуре)

Параметры запуска:
  linear_speed   — скорость (м/с),                 по умолчанию 0.5
  scale          — масштаб петли (м),               по умолчанию 3.0
  loop_count     — число полных «восьмёрок»,        по умолчанию 2
  a_distance     — продольная база Аккермана (м),   по умолчанию 0.4035
  max_steer_deg  — ограничение угла руля (°),       по умолчанию 30.0
  publish_rate   — частота команд (Гц),             по умолчанию 20.0
  warm_up_sec    — пауза перед стартом (с),         по умолчанию 2.0

Пример:
  ros2 run rover_description figure_eight_rviz.py \
    --ros-args -p linear_speed:=0.3 -p scale:=2.0 -p loop_count:=3
=========================================================================
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FigureEightRViz(Node):

    def __init__(self):
        super().__init__('figure_eight')

        # ── Параметры ─────────────────────────────────────────────────────
        self.declare_parameter('linear_speed',  0.5)
        self.declare_parameter('scale',         3.0)
        self.declare_parameter('loop_count',    2)
        self.declare_parameter('a_distance',    0.4035)
        self.declare_parameter('max_steer_deg', 30.0)
        self.declare_parameter('publish_rate',  20.0)
        self.declare_parameter('warm_up_sec',   2.0)

        self.v         = self.get_parameter('linear_speed').value
        self.scale     = self.get_parameter('scale').value
        self.loops     = self.get_parameter('loop_count').value
        self.a_dist    = self.get_parameter('a_distance').value
        self.max_steer = self.get_parameter('max_steer_deg').value
        rate           = self.get_parameter('publish_rate').value
        self.warm_up   = self.get_parameter('warm_up_sec').value

        # ── Период одного цикла «восьмёрки» ──────────────────────────────
        # Длина дуги Лиссажу ≈ 4·a·E(k) ≈ 4·a·1.211 (интеграл числово)
        self.T_cycle = 4.0 * self.scale * 1.211 / self.v

        # ── ROS2-интерфейс ────────────────────────────────────────────────
        self.pub = self.create_publisher(Float32MultiArray, '/motion_commands', 10)

        self._start_time = None   # момент первого вызова таймера
        self._stopped    = False

        self._timer = self.create_timer(1.0 / rate, self._cb)

        self.get_logger().info(
            f"\n"
            f"  ╔══════════════════════════════════════╗\n"
            f"  ║   Траектория «Восьмёрка» (RViz2)     ║\n"
            f"  ╠══════════════════════════════════════╣\n"
            f"  ║  Скорость:  {self.v:.2f} м/с               ║\n"
            f"  ║  Масштаб:   {self.scale:.1f} м                  ║\n"
            f"  ║  T_цикла:   {self.T_cycle:.1f} с                ║\n"
            f"  ║  Петли:     {self.loops}                       ║\n"
            f"  ║  Итого:     {self.loops * self.T_cycle:.0f} с                  ║\n"
            f"  ║  Прогрев:   {self.warm_up:.1f} с                 ║\n"
            f"  ╚══════════════════════════════════════╝"
        )

    # ─── Основной таймер ─────────────────────────────────────────────────

    def _cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Запоминаем время первого вызова (нода уже запущена)
        if self._start_time is None:
            self._start_time = now
            self._publish(0.0, 0.0)
            return

        elapsed = now - self._start_time

        # ── Прогрев: стоим на месте ───────────────────────────────────────
        if elapsed < self.warm_up:
            self._publish(0.0, 0.0)
            remaining = self.warm_up - elapsed
            if int(remaining * 10) % 5 == 0:  # лог каждые 0.5с
                self.get_logger().info(
                    f"Прогрев... старт через {remaining:.1f} с",
                    throttle_duration_sec=0.5
                )
            return

        # ── Движение по «восьмёрке» ───────────────────────────────────────
        t = elapsed - self.warm_up
        total_time = self.loops * self.T_cycle

        if t >= total_time:
            if not self._stopped:
                self._publish(0.0, 0.0)
                self._stopped = True
                if self._timer:
                    self._timer.cancel()
                self.get_logger().info("✓ Восьмёрка завершена! Ровер остановлен.")
            return

        # Прогресс (для лога)
        progress = (t / total_time) * 100.0
        self.get_logger().info(
            f"Прогресс: {progress:.0f}% | t={t:.1f}/{total_time:.0f}с",
            throttle_duration_sec=5.0
        )

        # ── Параметр кривой s ∈ [0, 2π] за один T_cycle ─────────────────
        s = (t / self.T_cycle) * 2.0 * math.pi
        a = self.scale

        # ── Кривая Лиссажу: x(s)=a·sin(s), y(s)=a·sin(2s)/2 ─────────────
        # Первые производные по s (вектор скорости по параметру):
        dx = a * math.cos(s)
        dy = a * math.cos(2.0 * s)

        # Вторые производные (вектор ускорения по параметру):
        ddx = -a * math.sin(s)
        ddy = -2.0 * a * math.sin(2.0 * s)

        # Модуль скорости параметризации (не физическая скорость!)
        spd = math.hypot(dx, dy)
        if spd < 1e-9:
            spd = 1e-9

        # ── Знаковая кривизна κ = (dx·d²y − dy·d²x) / |v|³ ─────────────
        # κ > 0 → левый поворот (в стандартных координатах XY, Y вверх/влево)
        # κ < 0 → правый поворот
        kappa = (dx * ddy - dy * ddx) / (spd ** 3)

        # ── Угол руля из кривизны ─────────────────────────────────────────
        # Радиус ICR: R = 1/κ  (знак сохраняется)
        # R > 0 → ICR слева  → поворот ВЛЕВО  → steering_angle < 0 (наша конвенция)
        # R < 0 → ICR справа → поворот ВПРАВО → steering_angle > 0
        # Формула: steering_angle = -atan(a_dist / R)
        # Знак минус — из анализа нашей системы координат (base_footprint
        # повёрнут на -90° относительно стандартного положения CAD-модели).
        if abs(kappa) > 1e-6:
            R = 1.0 / kappa
            steer_deg = -math.degrees(math.atan(self.a_dist / R))
        else:
            steer_deg = 0.0

        # Ограничиваем угол аппаратными пределами
        steer_deg = max(-self.max_steer, min(self.max_steer, steer_deg))

        self._publish(self.v, steer_deg)

    # ─── Публикация ───────────────────────────────────────────────────────

    def _publish(self, speed: float, steer: float):
        msg = Float32MultiArray()
        msg.data = [float(speed), float(steer)]
        self.pub.publish(msg)


# ─── Точка входа ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightRViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)  # безопасная остановка
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()