#!/usr/bin/env python3
"""
Лемниската Бернулли через реальную кривизну кривой Лиссажу.

Кривая: x(s) = a·sin(s),  y(s) = a·sin(2s)/2
Старт:  s₀ = π/4  — точка где dx=a/√2, dy=0 → ровер едет строго вперёд (+X)
Кривизна: κ = (dx·d²y - dy·d²x) / |v|³
Руль:     θ = -atan(L·κ)  (знак минус — корректировка нашей системы координат)
Шаг:      ds = v·dt / (dx,dy)  — arc-length parameterization
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FigureEightRViz(Node):

    def __init__(self):
        super().__init__('figure_lemniscate')

        # Параметры (можно менять через ros2 param set)
        self.declare_parameter('linear_speed',  0.5)
        self.declare_parameter('scale',         3.0)
        self.declare_parameter('loop_count',    2)
        self.declare_parameter('a_distance',    0.4035)
        self.declare_parameter('max_steer_deg', 30.0)
        self.declare_parameter('publish_rate',  20.0)
        self.declare_parameter('delay_sec',     10.0)

        self.v         = self.get_parameter('linear_speed').value
        self.a         = self.get_parameter('scale').value        # масштаб кривой
        self.loops     = self.get_parameter('loop_count').value
        self.L         = self.get_parameter('a_distance').value   # продольная база (м)
        self.max_steer = self.get_parameter('max_steer_deg').value
        self.rate      = self.get_parameter('publish_rate').value
        self.delay     = self.get_parameter('delay_sec').value

        self.dt = 1.0 / self.rate

        # Кривая: x = a·sin(s), y = a·sin(2s)/2
        # При s=π/4: dx>0, dy=0 → направление чисто +X
        self._s  = math.pi / 4.0
        self._s0 = math.pi / 4.0

        self._started     = False
        self._start_time  = None
        self._stopped     = False
        self._launch_time = self.get_clock().now().nanoseconds * 1e-9

        self.pub = self.create_publisher(Float32MultiArray, '/motion_commands', 10)
        self.create_timer(self.dt, self._cb)

        self.get_logger().info(
            f"\n"
            f"  ╔══════════════════════════════════════════╗\n"
            f"  ║   Лемниската Бернулли (RViz2)            ║\n"
            f"  ╠══════════════════════════════════════════╣\n"
            f"  ║  Скорость:    {self.v:.2f} м/с                ║\n"
            f"  ║  Масштаб a:   {self.a:.1f} м                   ║\n"
            f"  ║  Петли:       {self.loops}                         ║\n"
            f"  ║  СТАРТ через: {self.delay:.0f} с                   ║\n"
            f"  ╚══════════════════════════════════════════╝\n"
            f"  RViz2: Fixed Frame=odom | Add→Path→/rover_path"
        )

    def _steer_at(self, s):
        """
        Вычисляет угол руля для параметра s кривой x = a·sin(s), y = a·sin(2s)/2.
        Возвращает (steer_deg, spd), где spd — скорость параметризации (dx,dy).
        """
        a = self.a

        dx  =  a * math.cos(s)
        dy  =  a * math.cos(2.0 * s)
        ddx = -a * math.sin(s)
        ddy = -2.0 * a * math.sin(2.0 * s)

        spd = math.hypot(dx, dy)
        if spd < 1e-9:
            return 0.0, spd

        # Знаковая кривизна
        kappa = (dx * ddy - dy * ddx) / (spd ** 3)

        # Угол руля в градусах (знак минус — для нашей системы координат)
        if abs(kappa) > 1e-6:
            R = 1.0 / kappa
            steer_deg = -math.degrees(math.atan(self.L / R))
        else:
            steer_deg = 0.0

        steer_deg = max(-self.max_steer, min(self.max_steer, steer_deg))
        return steer_deg, spd

    def _cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # ----- Отсчёт задержки перед стартом -----
        if not self._started:
            remaining = self.delay - (now - self._launch_time)
            if remaining > 0:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"Старт через {remaining:.0f} с...",
                    throttle_duration_sec=1.0)
                return
            else:
                self._started = True
                self._start_time = now
                self.get_logger().info("▶ СТАРТ! Лемниската Бернулли")

        # ----- Проверка завершения всех петель -----
        s_done = self._s - self._s0
        s_total = 2.0 * math.pi * self.loops   # один полный цикл параметра s = 2π

        if s_done >= s_total:
            if not self._stopped:
                self._pub(0.0, 0.0)
                self._stopped = True
                self.get_logger().info(
                    "■ Лемниската завершена!\n"
                    "  Сброс: ros2 topic pub --once /reset_path std_msgs/msg/Bool '{data: true}'"
                )
            return

        # ----- Вычисление текущего угла руля -----
        steer_deg, spd = self._steer_at(self._s)

        # ----- Продвижение параметра s с постоянной физической скоростью -----
        if spd > 1e-9:
            self._s += self.v * self.dt / spd

        # ----- Логирование прогресса -----
        progress = s_done / s_total * 100.0
        # Для красивого вывода кривизны (опционально)
        kappa_show = 0.0
        if abs(steer_deg) > 0.01:
            try:
                R_cur = self.L / math.tan(math.radians(abs(steer_deg)))
                kappa_show = 1.0 / R_cur if R_cur != 0 else 0.0
            except:
                kappa_show = 0.0

        self.get_logger().info(
            f"s={self._s:.2f}  κ={kappa_show:.2f}  руль={steer_deg:+.1f}°  |  {progress:.0f}%",
            throttle_duration_sec=5.0)

        self._pub(self.v, steer_deg)

    def _pub(self, speed, steer_deg):
        """Публикация команды: [линейная скорость (м/с), угол руля (градусы)]"""
        msg = Float32MultiArray()
        msg.data = [float(speed), float(steer_deg)]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FigureEightRViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Прерывание по Ctrl+C")
    finally:
        node._pub(0.0, 0.0)  # финальная остановка
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()