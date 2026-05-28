#!/usr/bin/env python3
"""
robot_keyboard_controller.py — Клавиатурное управление для симуляции ровера.

Публикует: /motion_commands  Float32MultiArray [speed_m_s, steering_angle_deg]

Управление:
  W / S     — увеличить / уменьшить скорость
  A / D     — повернуть влево / вправо
  Space / K — полная остановка
  C         — центрирование руля (скорость сохраняется)
  Q         — выход

Отличия от аппаратной версии:
  • Убран параметр initial_servo_pos (не нужен в симуляции).
  • Добавлен таймер публикации: команда отправляется в /motion_commands
    с частотой 20 Гц, даже если клавиши не нажаты. Это обеспечивает
    стабильный поток команд для ackermann_sim_node.
  • Добавлено автоматическое замедление: если пользователь не нажимал
    клавишу более IDLE_TIMEOUT секунд, скорость плавно снижается к нулю.
    Это предотвращает ситуацию «запустил — забыл — ровер уехал».
"""

import sys
import select
import termios
import tty
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# ─── Константы ───────────────────────────────────────────────────────────────

PUBLISH_RATE_HZ  = 20.0   # Гц: как часто отправлять команду по таймеру
IDLE_TIMEOUT_SEC = 3.0    # Секунды бездействия до начала автоторможения
BRAKE_STEP       = 0.05   # м/с: шаг снижения скорости при автоторможении
                           # (за один тик таймера = 0.05 м/с / 20 Гц = 1 м/с²)


# ─── Класс узла ──────────────────────────────────────────────────────────────

class KeyboardController(Node):

    def __init__(self):
        super().__init__('robot_keyboard_controller')

        # ── Параметры (можно переопределить при запуске) ──────────────────
        self.declare_parameter('max_speed',          0.7)  # м/с
        self.declare_parameter('max_steering_angle', 30.0) # градусы
        self.declare_parameter('base_step_speed',    0.2)  # м/с на нажатие
        self.declare_parameter('base_step_angle',    2.0)  # градусы на нажатие
        self.declare_parameter('auto_brake',         True) # вкл/выкл автоторможение

        self._read_params()

        # ── Состояние робота ──────────────────────────────────────────────
        self.speed          = 0.0  # текущая скорость, м/с
        self.steering_angle = 0.0  # текущий угол руля, градусы
        self.last_key_time  = time.time()

        # ── Ускорение при удержании клавиши ──────────────────────────────
        # Чем дольше держишь W/A/S/D — тем быстрее меняется значение.
        self.accel_factor  = 1.0
        self.active_keys   = set()

        # ── ROS2 publisher ────────────────────────────────────────────────
        self.pub = self.create_publisher(Float32MultiArray, '/motion_commands', 10)

        # ── Таймер публикации ─────────────────────────────────────────────
        # Публикует команду с постоянной частотой, независимо от ввода.
        # Это важно: ackermann_sim_node интегрирует команды по времени,
        # и стабильный поток даёт точную симуляцию.
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._timer_cb)

        # ── Настройки терминала (сохраняем для восстановления) ───────────
        self.term_settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            f"KeyboardController ready | "
            f"max_speed={self.max_speed} м/с | "
            f"max_angle={self.max_steering_angle}° | "
            f"auto_brake={'on' if self.auto_brake else 'off'}"
        )
        self._print_help()

    # ─── Вспомогательные методы ──────────────────────────────────────────────

    def _read_params(self):
        """Читает ROS2-параметры в локальные переменные."""
        self.max_speed          = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.base_step_speed    = self.get_parameter('base_step_speed').value
        self.base_step_angle    = self.get_parameter('base_step_angle').value
        self.auto_brake         = self.get_parameter('auto_brake').value

    def _print_help(self):
        print("\n╔═══════════════════════════════════════╗")
        print("║  Управление ровером (симуляция)       ║")
        print("╠═══════════════════════════════════════╣")
        print("║  W / S  — скорость +/-                ║")
        print("║  A / D  — руль влево/вправо            ║")
        print("║  Space  — СТОП (скорость + руль = 0)  ║")
        print("║  C      — центрировать руль            ║")
        print("║  Q      — выход                       ║")
        print("╚═══════════════════════════════════════╝\n")

    def _publish(self):
        """Публикует текущую команду в /motion_commands."""
        msg = Float32MultiArray()
        msg.data = [float(self.speed), float(self.steering_angle)]
        self.pub.publish(msg)

    # ─── Таймер: периодическая публикация + автоторможение ───────────────────

    def _timer_cb(self):
        """
        Вызывается 20 раз в секунду.

        Делает две вещи:
        1. Публикует текущую команду — даже если клавиш не нажато,
           sim-нода получает стабильный поток.
        2. Если auto_brake включён и прошло IDLE_TIMEOUT секунд без ввода —
           плавно снижает скорость к нулю.
        """
        # Обновляем параметры (могут меняться через ros2 param set)
        self._read_params()

        # Автоторможение
        if self.auto_brake:
            idle = time.time() - self.last_key_time
            if idle > IDLE_TIMEOUT_SEC and abs(self.speed) > 0.001:
                # Снижаем скорость в сторону нуля на один шаг
                if self.speed > 0:
                    self.speed = max(0.0, self.speed - BRAKE_STEP)
                else:
                    self.speed = min(0.0, self.speed + BRAKE_STEP)

        self._publish()

    # ─── Основной цикл ввода с клавиатуры ────────────────────────────────────

    def _get_key(self) -> str:
        """
        Неблокирующее чтение одного символа с клавиатуры.
        Возвращает пустую строку, если в течение 0.1 с ничего не нажато.
        """
        tty.setraw(sys.stdin.fileno())
        # select ждёт 0.1 с — это даёт ROS2 время обработать callbacks
        ready, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if ready else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_settings)
        return key

    def run(self):
        """
        Основной цикл: читает клавиши и обновляет состояние.
        ROS2-таймер работает параллельно в том же потоке через spin_once.
        """
        print("Нажимайте клавиши (Q — выход)...")

        try:
            while rclpy.ok():
                key = self._get_key()

                # rclpy.spin_once обрабатывает таймеры и callbacks пока
                # мы ждём нажатия — таймер публикации работает регулярно
                rclpy.spin_once(self, timeout_sec=0.0)

                if not key:
                    # Клавиша не нажата → очищаем активные клавиши,
                    # сбрасываем ускорение
                    self.active_keys.clear()
                    self.accel_factor = 1.0
                    continue

                # Обновляем время последнего нажатия (сбрасывает автоторможение)
                self.last_key_time = time.time()

                # Ускорение: чем дольше держим клавишу — тем крупнее шаг
                if key in ('w', 's', 'a', 'd'):
                    self.active_keys.add(key)
                    self.accel_factor = min(self.accel_factor + 0.15, 3.0)
                else:
                    self.active_keys.clear()
                    self.accel_factor = 1.0

                step_spd = self.base_step_speed * self.accel_factor
                step_ang = self.base_step_angle * self.accel_factor

                # ── Обработка нажатий ─────────────────────────────────────
                if key == 'q':
                    print("\nВыход...")
                    break

                elif key in (' ', 'k'):
                    self.speed          = 0.0
                    self.steering_angle = 0.0
                    print("\n⚡ ПОЛНАЯ ОСТАНОВКА")

                elif key == 'c':
                    self.steering_angle = 0.0
                    print("\n🎯 Руль центрирован")

                elif key == 'w':
                    self.speed = min(self.speed + step_spd, self.max_speed)

                elif key == 's':
                    self.speed = max(self.speed - step_spd, -self.max_speed)

                elif key == 'a':
                    self.steering_angle = max(
                        self.steering_angle - step_ang,
                        -self.max_steering_angle
                    )

                elif key == 'd':
                    self.steering_angle = min(
                        self.steering_angle + step_ang,
                        self.max_steering_angle
                    )

                # ── Вывод состояния в строку терминала ────────────────────
                dir_sym  = "▲ ВПЕРЕД" if self.speed > 0 else ("▼ НАЗАД" if self.speed < 0 else "■ СТОП")
                turn_sym = "◄ ЛЕВО"   if self.steering_angle < 0 else ("► ПРАВО" if self.steering_angle > 0 else "↑ ПРЯМО")
                print(
                    f"\rV={self.speed:5.2f} м/с {dir_sym} | "
                    f"θ={self.steering_angle:5.1f}° {turn_sym} | "
                    f"x{self.accel_factor:.1f}   ",
                    end='', flush=True
                )

        except Exception as e:
            self.get_logger().error(f"Ошибка: {e}")

        finally:
            # Всегда восстанавливаем терминал и отправляем СТОП
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_settings)
            print("\n\nОстановка ровера и завершение работы...")


# ─── Точка входа ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Отправляем команду остановки перед выходом
        node.speed          = 0.0
        node.steering_angle = 0.0
        node._publish()
        time.sleep(0.1)

        node.destroy_node()
        rclpy.shutdown()
        print("ROS2 узел остановлен.")


if __name__ == '__main__':
    main()