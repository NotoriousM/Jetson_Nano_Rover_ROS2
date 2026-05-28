#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import evdev
from evdev import InputDevice, ecodes
import threading
import array
import math
import time

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller_node')
        
        # Параметры управления
        self.declare_parameter('max_speed', 14.0)
        self.declare_parameter('max_angle', 180)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('steering_scale', 0.7)  # Коэффициент масштабирования поворота
        self.declare_parameter('steering_exponent', 1.5)  # Экспонента для нелинейного отклика
        
        # Публикатор команд управления
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/motion_commands',
            10
        )
        
        # Инициализация джойстика
        self.controller = self.init_controller()
        if not self.controller:
            self.get_logger().error("PS4 controller not found!")
            return
            
        self.get_logger().info(f"Controller found: {self.controller.name}")
        
        # Текущее состояние
        self.linear_speed = 0.0
        self.steering_angle = 90.0  # Центральное положение
        self.target_angle = 90.0    # Целевой угол для плавного изменения
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angle = self.get_parameter('max_angle').value
        self.deadzone = self.get_parameter('deadzone').value
        self.steering_scale = self.get_parameter('steering_scale').value
        self.steering_exponent = self.get_parameter('steering_exponent').value
        self.emergency_stop = False
        
        # Для плавного изменения угла
        self.angle_step = 2.0  # Базовый шаг изменения угла
        self.max_angle_step = 10.0  # Максимальный шаг изменения угла
        self.angle_acceleration = 1.0  # Текущий коэффициент ускорения
        self.last_angle_update = time.time()
        self.angle_direction = 0  # Направление изменения угла (-1, 0, 1)
        
        # Поток для чтения событий джойстика
        self.thread = threading.Thread(target=self.read_loop)
        self.thread.daemon = True
        self.thread.start()
        
        # Таймер для публикации команд и плавного изменения угла
        self.timer = self.create_timer(0.01, self.update_angle)  # 50 Гц для плавности
        self.pub_timer = self.create_timer(0.01, self.publish_command)  # 20 Гц для публикации

    def init_controller(self):
        devices = [InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if "Wireless Controller" in device.name:
                return device
        return None

    def read_loop(self):
        try:
            for event in self.controller.read_loop():
                self.process_event(event)
        except Exception as e:
            self.get_logger().error(f"Controller read error: {str(e)}")

    def process_event(self, event):
        if event.type == ecodes.EV_KEY:
            self.process_buttons(event)
        elif event.type == ecodes.EV_ABS:
            self.process_axes(event)

    def process_buttons(self, event):
        if event.code == 305 and event.value == 1:  # Кнопка X
            self.emergency_stop = True
            self.get_logger().warn("EMERGENCY STOP activated!")
        elif event.code == 306 and event.value == 1:  # Кнопка O
            self.emergency_stop = False
            self.get_logger().info("Emergency stop released")
        elif event.code == 307 and event.value == 1:  # Кнопка △
            self.max_speed = min(self.max_speed + 0.1, 20.0)
            self.get_logger().info(f"Max speed increased to {self.max_speed:.1f} m/s")
        elif event.code == 304 and event.value == 1:  # Кнопка □
            self.max_speed = max(self.max_speed - 0.1, 0.1)
            self.get_logger().info(f"Max speed decreased to {self.max_speed:.1f} m/s")
        elif event.code == 313 and event.value == 1:  # Кнопка Options
            self.target_angle = 90.0
            self.get_logger().info("Steering centered")

    def process_axes(self, event):
        # Левый стик Y (ось 1) - линейная скорость
        if event.code == 1:
            value = (event.value - 128) / 128.0
            if abs(value) < self.deadzone:
                self.linear_speed = 0.0
            else:
                # Применяем нелинейную кривую для более точного управления
                signed_value = -value
                abs_value = abs(signed_value)
                scaled_value = math.copysign(abs_value ** 1.5, signed_value)
                self.linear_speed = scaled_value * self.max_speed
        
        # Правый стик X (ось 2) - угол поворота
        elif event.code == 2:
            value = (event.value - 128) / 128.0
            if abs(value) < self.deadzone:
                self.angle_direction = 0
                return
            
            # Применяем нелинейную кривую для более точного управления в центре
            abs_value = abs(value)
            scaled_value = math.copysign(abs_value ** self.steering_exponent, value)
            
            # Рассчитываем целевой угол с учетом масштабирования
            angle_offset = scaled_value * 90 * self.steering_scale
            self.target_angle = 90 + angle_offset
            self.target_angle = max(0, min(self.target_angle, self.max_angle))
            
            # Определяем направление для плавного изменения
            self.angle_direction = 1 if angle_offset > 0 else -1

    def update_angle(self):
        """Плавное изменение угла до целевого значения"""
        if self.steering_angle == self.target_angle:
            self.angle_acceleration = 1.0
            return
        
        # Рассчитываем шаг изменения с ускорением
        current_time = time.time()
        time_delta = current_time - self.last_angle_update
        self.last_angle_update = current_time
        
        # Увеличиваем ускорение при удержании стика
        if self.angle_direction != 0:
            self.angle_acceleration = min(self.angle_acceleration + 2.0 * time_delta, 1.5)
        else:
            self.angle_acceleration = 1.0
        
        # Рассчитываем шаг изменения
        step = self.angle_step * self.angle_acceleration
        step = min(step, self.max_angle_step)
        
        # Плавно изменяем угол
        if self.steering_angle < self.target_angle:
            self.steering_angle = min(self.steering_angle + step, self.target_angle)
        else:
            self.steering_angle = max(self.steering_angle - step, self.target_angle)

    def publish_command(self):
        msg = Float32MultiArray()
        
        if self.emergency_stop:
            msg.data = array.array('f', [0.0, 90.0])
            self.get_logger().warn("Emergency stop active! Motors disabled", throttle_duration_sec=1.0)
        else:
            msg.data = array.array('f', [self.linear_speed, self.steering_angle])
        
        self.publisher.publish(msg)
        
        # Логирование состояния с информацией о плавности
        self.get_logger().info(
            f"Speed: {self.linear_speed:.2f} m/s | Angle: {self.steering_angle:.1f}° (Target: {self.target_angle:.1f}°) | Accel: {self.angle_acceleration:.1f}x",
            throttle_duration_sec=0.2
        )

def main(args=None):
    rclpy.init(args=args)
    node = PS4ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()