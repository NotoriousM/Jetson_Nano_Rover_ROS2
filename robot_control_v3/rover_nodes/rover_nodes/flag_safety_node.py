#!/usr/bin/env python3
"""
FLAG SAFETY NODE — АППАРАТНАЯ АВАРИЙНАЯ ОСТАНОВКА
====================================================
Заменяет бессмысленные программные estop/watchdog (которые не знают,
что реально происходит на железе) на узел, который реагирует на
АППАРАТНЫЙ флаг `usb_stop_flag` от STM32 (поле SendData.flag).

Источник правды — нижний уровень. STM32 выставляет флаг при:
  • Превышении тока  (защита моторов)
  • Сбое энкодера    (потеря обратной связи)
  • Любой другой аппаратной проблеме (см. прошивку servocontrol)

Когда хотя бы одно колесо подняло флаг != 0, мы:
  1. Блокируем поток команд /motion_commands → /motion_commands_safe
  2. Публикуем нули
  3. Публикуем диагностику /safety/active_flags

Также прослушиваем `/safety/clear` — кнопка ручного снятия после
устранения причины (например, оператор разгрузил мотор).

ВХОД:
  /motion_commands       [MotionCommand]      ← keyboard / dualshock4 / trajectory
  /wheels/state          [RoverWheelsState]   ← serial_controller_node (поле flag!)
  /safety/clear          [Bool]               ← ручной сброс защиты

ВЫХОД:
  /motion_commands_safe  [MotionCommand]      → ackermann_calculator_node
  /safety/active         [Bool]               → диагностика
  /safety/active_flags   [String]             → какие колёса подняли флаг

ПРОТОКОЛ STM32 (см. main.c прошивки):
  typedef struct {
    float    speed;     // 4 байта — encoder_getVelocity()
    uint8_t  flag;      // 1 байт  — usb_stop_flag (0 = OK, !=0 = stop)
  } SendData;
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Bool, String

from rover_interfaces.msg import MotionCommand, RoverWheelsState


WHEEL_NAMES = [
    'front_left',  'middle_left',  'rear_left',
    'front_right', 'middle_right', 'rear_right',
]


class FlagSafetyNode(Node):

    def __init__(self):
        super().__init__('flag_safety_node')

        D = ParameterType.PARAMETER_DOUBLE
        B = ParameterType.PARAMETER_BOOL

        def pd(d): return ParameterDescriptor(description=d, type=D)
        def pb(d): return ParameterDescriptor(description=d, type=B)

        self.declare_parameter(
            'auto_clear_when_flag_resets', True,
            pb('Автоматически снимать защиту когда STM32 сбросил все флаги'))
        self.declare_parameter(
            'publish_rate_hz', 20.0,
            pd('Частота публикации статуса и стоп-команд'))

        self._auto_clear = self.get_parameter('auto_clear_when_flag_resets').value
        rate             = self.get_parameter('publish_rate_hz').value

        # Состояние защиты
        self._safety_active = False                      # сейчас блокируем?
        self._active_flags  = {}                         # имя колеса -> uint8 flag
        self._manual_block  = False                      # оператор явно остановил?

        # Публикаторы
        self._safe_cmd_pub = self.create_publisher(
            MotionCommand, '/motion_commands_safe', 10)
        self._status_pub   = self.create_publisher(
            Bool, '/safety/active', 10)
        self._flags_pub    = self.create_publisher(
            String, '/safety/active_flags', 10)

        # Подписки
        self.create_subscription(
            MotionCommand, '/motion_commands',
            self._cmd_callback, 10)
        self.create_subscription(
            RoverWheelsState, '/wheels/state',
            self._wheels_callback, 10)
        self.create_subscription(
            Bool, '/safety/clear',
            self._clear_callback, 10)

        self.create_timer(1.0 / rate, self._publish_callback)

        self.get_logger().info(
            'FlagSafetyNode ready\n'
            '  Source of truth: STM32 SendData.flag (usb_stop_flag)\n'
            '  Inputs:  /motion_commands /wheels/state /safety/clear\n'
            '  Outputs: /motion_commands_safe /safety/active /safety/active_flags'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _wheels_callback(self, msg: RoverWheelsState):
        """
        Анализируем флаги от каждого колеса.
        Если хотя бы одно колесо имеет flag != 0 — активируем защиту.
        """
        wheels = {
            'front_left':   msg.front_left,
            'middle_left':  msg.middle_left,
            'rear_left':    msg.rear_left,
            'front_right':  msg.front_right,
            'middle_right': msg.middle_right,
            'rear_right':   msg.rear_right,
        }

        new_flags = {}
        for name, ws in wheels.items():
            # Учитываем флаг только подключённых колёс
            if ws.is_connected and ws.flag != 0:
                new_flags[name] = int(ws.flag)

        # Изменилось — логируем
        if new_flags != self._active_flags:
            if new_flags:
                self.get_logger().warn(
                    f'⚠ Hardware flags raised: '
                    f'{", ".join(f"{n}=0x{f:02X}" for n, f in new_flags.items())}'
                )
            else:
                self.get_logger().info('All hardware flags cleared')

        self._active_flags = new_flags

        # Управление состоянием защиты
        if new_flags:
            # Любой флаг — активируем защиту
            self._safety_active = True
        elif self._auto_clear and not self._manual_block:
            # Авто-снятие если все флаги сброшены и нет ручного блока
            self._safety_active = False

    def _cmd_callback(self, msg: MotionCommand):
        """Фильтруем поток команд."""
        if self._safety_active:
            self._send_stop()
            self.get_logger().warn(
                f'BLOCKED command from [{msg.source}] (safety active)',
                throttle_duration_sec=2.0,
            )
        else:
            self._safe_cmd_pub.publish(msg)

    def _clear_callback(self, msg: Bool):
        """
        /safety/clear:
          true  — оператор требует ОСТАНОВКИ (manual block)
          false — оператор СНИМАЕТ блок и просит проверить флаги STM32
        """
        if msg.data:
            self._manual_block  = True
            self._safety_active = True
            self._send_stop()
            self.get_logger().warn('Manual safety block ENGAGED by operator')
        else:
            self._manual_block = False
            # Если на STM32 нет флагов — снимаем защиту
            if not self._active_flags:
                self._safety_active = False
                self.get_logger().info('Manual safety block RELEASED — system OK')
            else:
                self.get_logger().warn(
                    f'Cannot release: hardware flags still raised: '
                    f'{list(self._active_flags.keys())}'
                )

    def _publish_callback(self):
        """20 Гц: публикация стоп-команд + статуса."""
        if self._safety_active:
            self._send_stop()

        # Статус (Bool)
        status = Bool()
        status.data = self._safety_active
        self._status_pub.publish(status)

        # Расшифровка флагов (String)
        flags_msg = String()
        if self._manual_block and not self._active_flags:
            flags_msg.data = 'manual_block'
        elif self._active_flags:
            flags_msg.data = ' '.join(
                f'{name}=0x{f:02X}' for name, f in self._active_flags.items()
            )
        else:
            flags_msg.data = 'OK'
        self._flags_pub.publish(flags_msg)

    def _send_stop(self):
        """Безопасная команда: всё в ноль."""
        stop = MotionCommand()
        stop.linear_velocity = 0.0
        stop.steering_angle  = 0.0
        stop.source          = 'safety'
        self._safe_cmd_pub.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = FlagSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
