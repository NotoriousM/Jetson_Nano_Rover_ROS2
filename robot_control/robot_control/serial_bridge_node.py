#!/usr/bin/env python3
"""
Нода 2: Serial Bridge — мост между ROS2 и STM32 через UART.
Вход:  /wheel_commands       Float32MultiArray [6 скоростей + 4 угла]
Выход: /wheel/{name}/speed   Float32
       /wheel/{name}/flag    UInt8
       /wheel/{name}/angle   Float32
       /wheel/{name}/data    Float32MultiArray
       /wheels/all_data      Float32MultiArray
"""
import queue
import struct
import threading
import time
from typing import Dict

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, UInt8


# ══════════════════════════════════════════════════════════════════
class SerialPortManager:
    """Управляет одним COM-портом в отдельном потоке."""

    STRUCT_SIZE = 5  # STM32 → float speed (4) + uint8 flag (1)

    def __init__(self, port_path: str, baudrate: int, timeout: float,
                 initial_pos: float, wheel_name: str, data_cb):
        self.port_path    = port_path
        self.baudrate     = baudrate
        self.timeout      = timeout
        self.initial_pos  = initial_pos
        self.wheel_name   = wheel_name
        self.data_cb      = data_cb

        self.ser             = None
        self.connected       = False
        self.running         = False
        self.receive_buffer  = bytearray()
        self.command_queue   = queue.Queue(maxsize=100)
        self._thread         = None

    # ── Lifecycle ──────────────────────────────────────────────────
    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._close()

    # ── Public API ─────────────────────────────────────────────────
    def send_command(self, speed: float, angle: int) -> bool:
        try:
            self.command_queue.put_nowait((speed, angle))
            return True
        except queue.Full:
            return False

    # ── Private ────────────────────────────────────────────────────
    def _connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port_path, baudrate=self.baudrate, timeout=self.timeout
            )
            self.connected = True
            self.receive_buffer = bytearray()
            # Начальная позиция сервопривода
            self.ser.write(struct.pack('<fH', 0.0, int(self.initial_pos)))
            self.ser.flush()
            return True
        except Exception:
            self.connected = False
            if self.ser:
                try: self.ser.close()
                except: pass
                self.ser = None
            return False

    def _close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(struct.pack('<fH', 0.0, int(self.initial_pos)))
                self.ser.flush()
                self.ser.close()
            except Exception:
                pass
        self.connected = False
        self.ser = None

    def _parse(self):
        while len(self.receive_buffer) >= self.STRUCT_SIZE:
            pkt = self.receive_buffer[:self.STRUCT_SIZE]
            self.receive_buffer = self.receive_buffer[self.STRUCT_SIZE:]
            try:
                speed = struct.unpack('<f', pkt[0:4])[0]
                flag  = pkt[4]
                if self.data_cb:
                    self.data_cb(self.wheel_name, speed, flag)
            except Exception as e:
                print(f"[{self.wheel_name}] Parse error: {e}")

    def _loop(self):
        last_attempt = 0.0
        RETRY_DELAY  = 1.0

        while self.running:
            now = time.time()

            if not self.connected:
                if now - last_attempt >= RETRY_DELAY:
                    ok = self._connect()
                    print(f"[{self.port_path}] {'Connected' if ok else 'Failed, retrying...'}")
                    last_attempt = now
                time.sleep(0.1)
                continue

            try:
                # 1. Отправка команды из очереди
                try:
                    speed, angle = self.command_queue.get_nowait()
                    if self.ser and self.ser.is_open:
                        self.ser.write(struct.pack('<fH', speed, angle))
                        self.ser.flush()
                except queue.Empty:
                    pass

                # 2. Чтение ответа от STM32
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    self.receive_buffer.extend(self.ser.read(self.ser.in_waiting))
                    self._parse()

            except serial.SerialException as e:
                print(f"[{self.port_path}] SerialException: {e}")
                self._close()
            except Exception as e:
                print(f"[{self.port_path}] Error: {e}")

            time.sleep(0.001)

        self._close()


# ══════════════════════════════════════════════════════════════════
class SerialBridgeNode(Node):
    # Порядок колёс в /wheel_commands: FL, ML, RL, FR, MR, RR
    WHEEL_ORDER = ['front_left', 'middle_left', 'rear_left',
                   'front_right', 'middle_right', 'rear_right']

    def __init__(self):
        super().__init__('serial_bridge')

        # Параметры
        self.declare_parameter('baudrate',            115200)
        self.declare_parameter('timeout',             0.1)
        self.declare_parameter('initial_pos_servo',   90.0)
        self.declare_parameter('port_front_left',     '/dev/ttyROVER_WHEEL_4')
        self.declare_parameter('port_middle_left',    '/dev/ttyROVER_WHEEL_5')
        self.declare_parameter('port_rear_left',      '/dev/ttyROVER_WHEEL_6')
        self.declare_parameter('port_front_right',    '/dev/ttyROVER_WHEEL_1')
        self.declare_parameter('port_middle_right',   '/dev/ttyROVER_WHEEL_2')
        self.declare_parameter('port_rear_right',     '/dev/ttyROVER_WHEEL_3')

        baud     = self.get_parameter('baudrate').value
        timeout  = self.get_parameter('timeout').value
        init_pos = self.get_parameter('initial_pos_servo').value
        self.initial_pos = init_pos

        port_map = {
            'front_left':   self.get_parameter('port_front_left').value,
            'middle_left':  self.get_parameter('port_middle_left').value,
            'rear_left':    self.get_parameter('port_rear_left').value,
            'front_right':  self.get_parameter('port_front_right').value,
            'middle_right': self.get_parameter('port_middle_right').value,
            'rear_right':   self.get_parameter('port_rear_right').value,
        }

        # Флаги поворотных колёс
        self._is_steering = {
            'front_left': True,  'middle_left': False,  'rear_left': True,
            'front_right': True, 'middle_right': False, 'rear_right': True,
        }

        # Текущие данные колёс
        self.wheel_data: Dict[str, dict] = {
            name: {'speed': 0.0, 'flag': 0, 'angle': float(init_pos)}
            for name in self.WHEEL_ORDER
        }

        # Инициализация SerialPortManager
        self.managers: Dict[str, SerialPortManager] = {}
        for name in self.WHEEL_ORDER:
            mgr = SerialPortManager(
                port_map[name], baud, timeout, init_pos,
                name, self._stm32_data_callback
            )
            self.managers[name] = mgr
            mgr.start()
            self.get_logger().info(f"Started {name} → {port_map[name]}")

        # Подписка на команды от ноды 1
        self.create_subscription(
            Float32MultiArray, '/wheel_commands',
            self._wheel_commands_callback, 10
        )

        # Публикаторы обратной связи
        self._pubs: Dict[str, dict] = {}
        for name in self.WHEEL_ORDER:
            self._pubs[name] = {
                'speed': self.create_publisher(Float32, f'/wheel/{name}/speed', 10),
                'flag':  self.create_publisher(UInt8,   f'/wheel/{name}/flag',  10),
                'angle': self.create_publisher(Float32, f'/wheel/{name}/angle', 10),
                'data':  self.create_publisher(Float32MultiArray, f'/wheel/{name}/data', 10),
            }
        self._all_pub = self.create_publisher(Float32MultiArray, '/wheels/all_data', 10)

        # Таймеры
        self.create_timer(0.1, self._publish_feedback)   # 10 Гц
        self.create_timer(5.0, self._monitor_ports)

        self.get_logger().info("SerialBridge ready")

    # ── Callbacks ──────────────────────────────────────────────────
    def _wheel_commands_callback(self, msg: Float32MultiArray):
        """Принимает [6 скоростей + 4 угла], раскладывает по портам."""
        if len(msg.data) != 10:
            self.get_logger().error(
                f"Expected 10 values [6 speeds + 4 angles], got {len(msg.data)}",
                throttle_duration_sec=2.0
            )
            return

        speeds = msg.data[0:6]   # FL, ML, RL, FR, MR, RR
        angles = msg.data[6:10]  # alpha_L1, alpha_R1, beta_L3, beta_R3

        # Соответствие углов: FL→angles[0], FR→angles[1], RL→angles[2], RR→angles[3]
        angle_map = {
            'front_left':  angles[0],
            'front_right': angles[1],
            'rear_left':   angles[2],
            'rear_right':  angles[3],
        }

        for i, name in enumerate(self.WHEEL_ORDER):
            speed = float(speeds[i])

            if self._is_steering[name]:
                raw_angle = float(angle_map[name])
                final_angle = int(max(0, min(180, self.initial_pos + raw_angle)))
            else:
                final_angle = int(self.initial_pos)

            self.wheel_data[name]['angle'] = float(final_angle)

            if not self.managers[name].send_command(speed, final_angle):
                self.get_logger().warn(
                    f"Queue full for {name}", throttle_duration_sec=1.0
                )

    def _stm32_data_callback(self, wheel_name: str, speed: float, flag: int):
        """Обратная связь от STM32 — вызывается из потока SerialPortManager."""
        if wheel_name in self.wheel_data:
            prev_flag = self.wheel_data[wheel_name].get('last_flag', 0)
            self.wheel_data[wheel_name]['speed'] = speed
            self.wheel_data[wheel_name]['flag']  = flag
            if flag != prev_flag:
                self.get_logger().debug(f"{wheel_name}: flag → {flag}")
                self.wheel_data[wheel_name]['last_flag'] = flag

    # ── Timers ─────────────────────────────────────────────────────
    def _publish_feedback(self):
        all_speeds, all_flags = [], []

        for name, data in self.wheel_data.items():
            self._pubs[name]['speed'].publish(Float32(data=float(data['speed'])))
            self._pubs[name]['flag'].publish(UInt8(data=int(data['flag'])))
            self._pubs[name]['angle'].publish(Float32(data=float(data['angle'])))

            arr = Float32MultiArray()
            arr.data = [float(data['speed']), float(data['flag']), float(data['angle'])]
            self._pubs[name]['data'].publish(arr)

            all_speeds.append(float(data['speed']))
            all_flags.append(float(data['flag']))

        msg = Float32MultiArray()
        msg.data = all_speeds + all_flags
        self._all_pub.publish(msg)

    def _monitor_ports(self):
        ok   = [n for n, m in self.managers.items() if m.connected]
        fail = [n for n, m in self.managers.items() if not m.connected]
        if fail:
            self.get_logger().warn(
                f"Disconnected: {fail} | Connected: {ok}",
                throttle_duration_sec=10.0
            )
        else:
            self.get_logger().info(
                f"All ports OK: {ok}", throttle_duration_sec=10.0
            )

    # ── Shutdown ───────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("Shutting down serial bridge...")
        for name, mgr in self.managers.items():
            try:
                mgr.stop()
            except Exception as e:
                self.get_logger().error(f"Error stopping {name}: {e}")
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()