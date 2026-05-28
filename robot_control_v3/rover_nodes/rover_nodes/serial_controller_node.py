#!/usr/bin/env python3
"""
SERIAL CONTROLLER NODE — МОСТ МЕЖДУ ROS2 И 6 STM32
====================================================

ВХОД:  /wheel/{name}/cmd   [WheelCommand]      ← ackermann_calculator_node × 6
ВЫХОД: /wheel/{name}/state [WheelState]        → odometry_node, мониторинг   × 6
       /wheels/state       [RoverWheelsState]  → odometry_node (агрегат)

ПРОТОКОЛ STM32 (USB CDC, Little-Endian, без padding):

  Jetson → STM32 (TX, 6 байт):
    typedef struct {
      float    speed;    // 4 байта — целевая скорость колеса (м/с)
      uint16_t angle;    // 2 байта — угол сервопривода (0..180°)
    } WheelData;
    Питон-эквивалент: struct.pack('<fH', speed, int(angle))

  STM32 → Jetson (RX, 5 байт):
    typedef struct {
      float   speed;     // 4 байта — encoder_getVelocity()
      uint8_t flag;      // 1 байт  — usb_stop_flag
    } SendData;
    Питон-эквивалент: struct.unpack('<fB', data)

АРХИТЕКТУРА ПОТОКОВ:
  Поток 0 (ROS Main):          1×Executor + create_timer(50ms) → _publish()
  Потоки 1-6 (daemon):          по одному на каждый COM-порт (SerialPortManager)

  Общие данные self._data:
    Пишут (под Lock):  потоки 1-6 в _on_data_callback()
    Читает (snapshot): поток 0 в _publish()

  Queue(maxsize=1) в каждом порту: всегда только последняя команда.
"""

import struct
import threading
import time
import queue
from typing import Dict

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rover_interfaces.msg import WheelCommand, WheelState, RoverWheelsState

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ── Бинарный протокол STM32 ──────────────────────────────────────────────────
TX_FORMAT = '<fH'    # float speed + uint16 angle
TX_SIZE   = struct.calcsize(TX_FORMAT)   # = 6 байт

RX_FORMAT = '<fB'    # float speed + uint8 flag
RX_SIZE   = struct.calcsize(RX_FORMAT)   # = 5 байт

WHEEL_NAMES = [
    'front_left',  'middle_left',  'rear_left',
    'front_right', 'middle_right', 'rear_right',
]

# Стабильные имена устройств через udev:
# /etc/udev/rules.d/99-rover-wheels.rules
PORT_DEFAULTS = {
    'front_right':  '/dev/ttyROVER_WHEEL_1',
    'middle_right': '/dev/ttyROVER_WHEEL_2',
    'rear_right':   '/dev/ttyROVER_WHEEL_3',
    'front_left':   '/dev/ttyROVER_WHEEL_4',
    'middle_left':  '/dev/ttyROVER_WHEEL_5',
    'rear_left':    '/dev/ttyROVER_WHEEL_6',
}


# ═════════════════════════════════════════════════════════════════════════════
class SerialPortManager:
    """
    Управляет одним COM-портом в отдельном daemon-потоке.

    Каждый поток выполняет:
      1. Подключение / переподключение к COM-порту
      2. Отправка команд: Queue.get → struct.pack → ser.write
      3. Приём ответов: ser.read → struct.unpack → callback
    """

    def __init__(self, port_path, baudrate, timeout, init_servo,
                 wheel_name, on_data_callback):
        self.port_path     = port_path
        self.baudrate      = baudrate
        self.timeout       = timeout
        self.init_servo    = init_servo
        self.wheel_name    = wheel_name
        self._on_data_cb   = on_data_callback

        self._ser          = None
        self._port_lock    = threading.Lock()    # защита self._ser
        self._connected    = False
        self._running      = False
        self._thread       = None
        self._rxbuf        = bytearray()

        # Queue(1): всегда только ПОСЛЕДНЯЯ команда
        self._cmd_queue    = queue.Queue(maxsize=1)

        self.stats = {'tx': 0, 'rx': 0, 'errors': 0, 'reconnects': 0}

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self):
        self._running = True
        self._thread  = threading.Thread(
            target=self._loop,
            name=f'serial_{self.wheel_name}',
            daemon=True,    # автоматически завершается с главным процессом
        )
        self._thread.start()

    def stop(self):
        """Корректное завершение: сигнал → join → отправка стопа → close."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._close_port()

    def send_command(self, speed: float, angle: int) -> bool:
        """
        Кладёт команду в Queue(1).
        Если в очереди уже есть команда — выбрасывает её.
        Это гарантирует что в очереди всегда самая свежая команда.
        """
        try:
            self._cmd_queue.get_nowait()    # выбросить старую
        except queue.Empty:
            pass
        try:
            self._cmd_queue.put_nowait((float(speed), int(angle)))
            return True
        except queue.Full:
            return False    # крайне редкая race-condition

    def _connect(self) -> bool:
        if not SERIAL_AVAILABLE:
            return False
        try:
            ser = serial.Serial(
                self.port_path, self.baudrate,
                timeout=self.timeout, write_timeout=0.1,
            )
            # Init-пакет: speed=0, angle=нейтраль
            init = struct.pack(TX_FORMAT, 0.0, int(self.init_servo))
            ser.write(init)
            ser.flush()

            with self._port_lock:
                self._ser = ser
                self._connected = True

            self._rxbuf = bytearray()
            self.stats['reconnects'] += 1
            return True
        except Exception:
            with self._port_lock:
                self._ser = None
                self._connected = False
            return False

    def _close_port(self):
        """Перед закрытием — STOP пакет на STM32."""
        with self._port_lock:
            ser, self._ser = self._ser, None
            self._connected = False
        if ser and ser.is_open:
            try:
                stop = struct.pack(TX_FORMAT, 0.0, int(self.init_servo))
                ser.write(stop)
                ser.flush()
                ser.close()
            except Exception:
                pass

    def _loop(self):
        """Главный цикл COM-порта."""
        RECONNECT_DELAY = 2.0
        last_attempt = 0.0

        while self._running:
            # ── Подключение ────────────────────────────────────────
            if not self._connected:
                now = time.monotonic()
                if now - last_attempt >= RECONNECT_DELAY:
                    last_attempt = now
                    ok = self._connect()
                    print(f'[{self.wheel_name}] '
                          f'{"Connected" if ok else "Failed"}: {self.port_path}')
                time.sleep(0.05)
                continue

            try:
                # ── TX: отправка команды ───────────────────────────
                try:
                    speed, angle = self._cmd_queue.get_nowait()
                    pkt = struct.pack(TX_FORMAT, speed, angle)   # 6 байт
                    with self._port_lock:
                        if self._ser and self._ser.is_open:
                            self._ser.write(pkt)
                            self._ser.flush()
                            self.stats['tx'] += 1
                except queue.Empty:
                    pass

                # ── RX: чтение ответа ──────────────────────────────
                with self._port_lock:
                    ser_ref = self._ser

                if ser_ref and ser_ref.is_open and ser_ref.in_waiting > 0:
                    raw = ser_ref.read(ser_ref.in_waiting)
                    self._rxbuf.extend(raw)
                    self._parse_packets()

            except Exception as e:
                print(f'[{self.wheel_name}] {type(e).__name__}: {e}')
                self.stats['errors'] += 1
                self._close_port()

            time.sleep(0.001)    # 1 мс — баланс между latency и CPU load

        self._close_port()

    def _parse_packets(self):
        """Парсим поток байт на пакеты по 5 байт."""
        while len(self._rxbuf) >= RX_SIZE:
            pkt = bytes(self._rxbuf[:RX_SIZE])
            self._rxbuf = self._rxbuf[RX_SIZE:]
            try:
                speed, flag = struct.unpack(RX_FORMAT, pkt)
                self.stats['rx'] += 1
                if self._on_data_cb:
                    # callback вызывается в этом потоке —
                    # SerialControllerNode пишет под Lock
                    self._on_data_cb(self.wheel_name, float(speed), int(flag))
            except struct.error:
                self.stats['errors'] += 1


# ═════════════════════════════════════════════════════════════════════════════
class SerialControllerNode(Node):

    def __init__(self):
        super().__init__('serial_controller_node')
        self._declare_params()

        baud     = self.get_parameter('baudrate').value
        timeout  = self.get_parameter('timeout').value
        init_pos = self.get_parameter('initial_pos_servo_deg').value

        # ── Общие данные + Lock ──────────────────────────────────────────────
        # Пишут: COM-потоки в _on_data_callback()
        # Читает: ROS таймер в _publish()
        self._data_lock = threading.Lock()
        self._data: Dict[str, dict] = {
            n: {
                'speed':     0.0,                  # с энкодера
                'flag':      0,                    # флаг STM32
                'angle_cmd': float(init_pos),      # угол который послали
                'connected': False,
            }
            for n in WHEEL_NAMES
        }

        # ── Запуск менеджеров COM-портов (6 daemon-потоков) ──────────────────
        self._mgrs: Dict[str, SerialPortManager] = {}
        for name in WHEEL_NAMES:
            port = self.get_parameter(f'port_{name}').value
            mgr  = SerialPortManager(
                port_path        = port,
                baudrate         = baud,
                timeout          = timeout,
                init_servo       = init_pos,
                wheel_name       = name,
                on_data_callback = self._on_data_callback,
            )
            self._mgrs[name] = mgr
            mgr.start()
            self.get_logger().info(f'Port: {name} → {port}')

        # ── Публикаторы ──────────────────────────────────────────────────────
        # /wheel/{name}/state — состояние каждого колеса отдельно
        self._state_pubs = {
            n: self.create_publisher(WheelState, f'/wheel/{n}/state', 10)
            for n in WHEEL_NAMES
        }
        # /wheels/state — все колёса в одном сообщении (именованные поля!)
        self._all_pub = self.create_publisher(
            RoverWheelsState, '/wheels/state', 10)

        # ── Подписки на WheelCommand × 6 ─────────────────────────────────────
        for name in WHEEL_NAMES:
            self.create_subscription(
                WheelCommand,
                f'/wheel/{name}/cmd',
                lambda msg, n=name: self._on_cmd(msg, n),
                10,
            )

        # ── Таймеры ──────────────────────────────────────────────────────────
        self.create_timer(0.05, self._publish)        # 20 Гц публикация
        self.create_timer(5.0,  self._monitor)        # 0.2 Гц диагностика

        self.get_logger().info(
            'SerialControllerNode ready\n'
            '  6× daemon threads for COM ports\n'
            '  Subscribes: /wheel/{name}/cmd × 6\n'
            '  Publishes:  /wheel/{name}/state × 6\n'
            '              /wheels/state'
        )

    def _declare_params(self):
        I = ParameterType.PARAMETER_INTEGER
        D = ParameterType.PARAMETER_DOUBLE
        S = ParameterType.PARAMETER_STRING

        def pi(d): return ParameterDescriptor(description=d, type=I)
        def pd(d): return ParameterDescriptor(description=d, type=D)
        def ps(d): return ParameterDescriptor(description=d, type=S)

        self.declare_parameter('baudrate',              115200, pi('Скорость порта'))
        self.declare_parameter('timeout',               0.1,    pd('Таймаут (с)'))
        self.declare_parameter('initial_pos_servo_deg', 90.0,   pd('Нейтраль сервы (°)'))
        for name, default in PORT_DEFAULTS.items():
            self.declare_parameter(f'port_{name}', default, ps(f'COM-порт {name}'))

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_data_callback(self, wheel_name: str, speed: float, flag: int):
        """
        ВЫЗЫВАЕТСЯ ИЗ COM-ПОТОКА (не из ROS!).
        Правило: только запись данных под Lock.
        НЕЛЬЗЯ вызывать ROS API из этого метода.
        """
        with self._data_lock:
            if wheel_name in self._data:
                self._data[wheel_name]['speed'] = speed
                self._data[wheel_name]['flag']  = flag

    def _on_cmd(self, msg: WheelCommand, expected_name: str):
        """Получаем WheelCommand → передаём в Queue(1) COM-потока."""
        if msg.wheel_name and msg.wheel_name != expected_name:
            self.get_logger().warn(
                f'Wheel name mismatch: expected={expected_name} '
                f'got={msg.wheel_name}',
                throttle_duration_sec=2.0,
            )

        with self._data_lock:
            self._data[expected_name]['angle_cmd'] = float(msg.angle_cmd)

        mgr = self._mgrs.get(expected_name)
        if mgr:
            mgr.send_command(msg.speed_cmd, int(msg.angle_cmd))

    def _publish(self):
        """
        20 Гц публикация состояния. Snapshot pattern:
          1. Под Lock делаем копию (~0.01 мс)
          2. Освобождаем Lock
          3. Публикуем из копии (медленно, без Lock)

        Это гарантирует что COM-потоки не блокируются на время публикации.
        """
        # Шаг 1: snapshot под Lock
        with self._data_lock:
            for n, mgr in self._mgrs.items():
                self._data[n]['connected'] = mgr.connected
            snap = {k: dict(v) for k, v in self._data.items()}
        # Lock свободен

        now = self.get_clock().now().to_msg()
        ws_map = {}

        # Шаг 2: публикуем WheelState × 6 (без Lock)
        for name, d in snap.items():
            ws = WheelState()
            ws.wheel_name   = name
            ws.speed        = float(d['speed'])
            ws.angle_cmd    = float(d['angle_cmd'])
            ws.flag         = int(d['flag'])
            ws.is_connected = bool(d['connected'])
            ws.stamp        = now
            ws_map[name]    = ws
            self._state_pubs[name].publish(ws)

        # Шаг 3: публикуем агрегат RoverWheelsState с ИМЕНОВАННЫМИ полями.
        # msg.middle_left.speed — нельзя перепутать с middle_right!
        all_ws = RoverWheelsState()
        all_ws.front_left   = ws_map['front_left']
        all_ws.middle_left  = ws_map['middle_left']
        all_ws.rear_left    = ws_map['rear_left']
        all_ws.front_right  = ws_map['front_right']
        all_ws.middle_right = ws_map['middle_right']
        all_ws.rear_right   = ws_map['rear_right']
        all_ws.stamp        = now
        all_ws.connected_count = sum(1 for d in snap.values() if d['connected'])
        self._all_pub.publish(all_ws)

    def _monitor(self):
        """Диагностика каждые 5 секунд."""
        bad = [n for n, m in self._mgrs.items() if not m.connected]
        if bad:
            self.get_logger().warn(
                f'Disconnected: {bad}',
                throttle_duration_sec=10.0,
            )
        else:
            tx = sum(m.stats['tx'] for m in self._mgrs.values())
            rx = sum(m.stats['rx'] for m in self._mgrs.values())
            self.get_logger().info(
                f'All 6 wheels OK | TX={tx} RX={rx}',
                throttle_duration_sec=10.0,
            )

    def destroy_node(self):
        self.get_logger().info('Stopping serial managers...')
        for mgr in self._mgrs.values():
            mgr.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
