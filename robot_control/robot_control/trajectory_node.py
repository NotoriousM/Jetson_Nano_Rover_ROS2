#!/usr/bin/env python3
"""
Узел для движения по траектории вперед-назад на 40 см
Совместим с ServoWheelController и моделью Аккермана
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math
import time

class SimpleLinearTrajectory(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        
        # ПАРАМЕТРЫ УПРАВЛЕНИЯ
        self.declare_parameter('forward_speed', 2.5)       # Скорость движения вперед (м/с)
        self.declare_parameter('backward_speed', 2.5)      # Скорость движения назад (м/с)
        self.declare_parameter('goal_tolerance', 0.02)     # Точность достижения цели (2 см)
        self.declare_parameter('pause_duration', 2.0)      # Пауза между движениями (сек)
        
        self.forward_speed = self.get_parameter('forward_speed').value
        self.backward_speed = self.get_parameter('backward_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.pause_duration = self.get_parameter('pause_duration').value
        
        # СОСТОЯНИЕ
        self.state = "IDLE"  # IDLE, MOVING_FORWARD, PAUSING, MOVING_BACKWARD, COMPLETED
        self.target_distance = 0.40  # 40 см вперед
        self.start_x = 0.0  # Начальная позиция по X
        self.start_y = 0.0  # Начальная позиция по Y
        self.got_start_position = False
        self.pause_start_time = 0.0
        
        # ТЕКУЩЕЕ ПОЛОЖЕНИЕ РОБОТА (из одометрии)
        self.current_x = 0.0
        self.current_y = 0.0
        
        # ПОДПИСКИ И ПУБЛИКАЦИИ
        # Подписываемся на одометрию (чтобы знать, где мы)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Публикуем команды управления для ServoWheelController
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/motion_commands',
            10
        )
        
        # Таймер для управления (10 Гц)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Таймер для логирования (1 Гц)
        self.log_timer = self.create_timer(1.0, self.log_status)
        
        self.get_logger().info('===== ПРОСТАЯ ЛИНЕЙНАЯ ТРАЕКТОРИЯ =====')
        self.get_logger().info(f'Вперед: {self.target_distance*100:.0f} см')
        self.get_logger().info(f'Назад: {self.target_distance*100:.0f} см')
        self.get_logger().info('======================================')
        
    def odom_callback(self, msg):
        """
        Получаем текущее положение робота из одометрии
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Запоминаем начальную позицию при первом сообщении
        if not self.got_start_position:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.got_start_position = True
            self.get_logger().info(f'Начальная позиция сохранена: X={self.start_x:.3f}, Y={self.start_y:.3f}')
            # Начинаем движение вперед
            self.state = "MOVING_FORWARD"
            self.get_logger().info('Начинаем движение ВПЕРЕД')
        
    def calculate_distance_traveled(self):
        """
        Вычисляем пройденное расстояние от начальной точки
        """
        if not self.got_start_position:
            return 0.0
            
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx*dx + dy*dy)
    
    def calculate_remaining_distance(self, target_distance):
        """
        Вычисляем оставшееся расстояние до цели
        """
        current_distance = self.calculate_distance_traveled()
        if target_distance >= current_distance:
            return target_distance - current_distance
        else:
            return current_distance - target_distance
    
    def control_loop(self):
        """
        Основной цикл управления - конечный автомат состояний
        """
        if not self.got_start_position:
            return
            
        if self.state == "MOVING_FORWARD":
            self.handle_moving_forward()
        elif self.state == "PAUSING":
            self.handle_pausing()
        elif self.state == "MOVING_BACKWARD":
            self.handle_moving_backward()
        elif self.state == "COMPLETED":
            self.handle_completed()
    
    def handle_moving_forward(self):
        """
        Движение вперед на заданное расстояние
        """
        current_distance = self.calculate_distance_traveled()
        remaining_distance = self.target_distance - current_distance
        
        # Если достигли цели
        if remaining_distance <= self.goal_tolerance:
            self.get_logger().info(f'✅ Достигнута точка вперед: {current_distance*100:.1f} см')
            self.send_command(0.0, 0.0)  # Остановка
            
            # Переходим к паузе
            self.state = "PAUSING"
            self.pause_start_time = time.time()
            self.get_logger().info(f'Пауза {self.pause_duration} сек...')
            return
        
        # Регулируем скорость: замедляемся при приближении к цели
        if remaining_distance < 0.2:  # Последние 20 см
            speed = self.forward_speed * (remaining_distance / 0.2)
        else:
            speed = self.forward_speed
            
        # Движение вперед - скорость положительная, угол 0
        self.send_command(speed, 0.0)
    
    def handle_pausing(self):
        """
        Пауза между движениями
        """
        elapsed = time.time() - self.pause_start_time
        
        if elapsed >= self.pause_duration:
            # Запоминаем текущую позицию как новую стартовую для движения назад
            self.start_x = self.current_x
            self.start_y = self.current_y
            
            # Переходим к движению назад
            self.state = "MOVING_BACKWARD"
            self.get_logger().info('Начинаем движение НАЗАД')
        else:
            # Остаемся на месте
            self.send_command(0.0, 0.0)
    
    def handle_moving_backward(self):
        """
        Движение назад к начальной точке
        """
        # Для движения назад считаем расстояние от текущей точки до начальной
        dx = self.start_x - self.current_x
        dy = self.start_y - self.current_y
        current_distance = math.sqrt(dx*dx + dy*dy)
        remaining_distance = self.target_distance - current_distance
        
        # Если вернулись в начало
        if current_distance <= self.goal_tolerance:
            self.get_logger().info(f'✅ Вернулись в начальную точку: {current_distance*100:.1f} см')
            self.send_command(0.0, 0.0)  # Остановка
            
            # Траектория завершена
            self.state = "COMPLETED"
            return
        
        # Регулируем скорость: замедляемся при приближении к цели
        if remaining_distance < 0.2:  # Последние 20 см
            speed = -self.backward_speed * (remaining_distance / 0.2)  # Отрицательная для движения назад
        else:
            speed = -self.backward_speed  # Отрицательная скорость для движения назад
            
        # Движение назад - скорость отрицательная, угол 0
        self.send_command(speed, 0.0)
    
    def handle_completed(self):
        """
        Траектория завершена
        """
        # Держим робота остановленным
        self.send_command(0.0, 0.0)
    
    def send_command(self, speed, angle):
        """
        Отправляем команду в топик /motion_commands
        Формат: [speed, angle] где angle в градусах
        """
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [float(speed), float(angle)]
        self.cmd_pub.publish(cmd_msg)
    
    def log_status(self):
        """
        Логирование текущего статуса
        """
        if not self.got_start_position:
            self.get_logger().info('Ожидание данных одометрии...')
            return
            
        if self.state == "COMPLETED":
            self.get_logger().info('🎉 Траектория завершена!')
            return
            
        # Преобразуем в см для наглядности
        current_x_cm = self.current_x * 100
        start_x_cm = self.start_x * 100
        
        if self.state == "MOVING_FORWARD":
            current_distance = self.calculate_distance_traveled()
            remaining = self.target_distance - current_distance
            progress_percent = (current_distance / self.target_distance) * 100
            
            self.get_logger().info(
                f'[ВПЕРЕД] '
                f'Пройдено: {current_distance*100:.1f} см / {self.target_distance*100:.0f} см '
                f'({progress_percent:.0f}%) | '
                f'Осталось: {remaining*100:.1f} см | '
                f'X: {current_x_cm:.1f} см'
            )
            
        elif self.state == "MOVING_BACKWARD":
            dx = self.start_x - self.current_x
            dy = self.start_y - self.current_y
            current_distance = math.sqrt(dx*dx + dy*dy)
            remaining = self.target_distance - current_distance
            progress_percent = (current_distance / self.target_distance) * 100
            
            self.get_logger().info(
                f'[НАЗАД] '
                f'Вернулись: {current_distance*100:.1f} см / {self.target_distance*100:.0f} см '
                f'({progress_percent:.0f}%) | '
                f'Осталось: {remaining*100:.1f} см | '
                f'X: {current_x_cm:.1f} см'
            )
            
        elif self.state == "PAUSING":
            elapsed = time.time() - self.pause_start_time
            remaining = max(0, self.pause_duration - elapsed)
            
            self.get_logger().info(
                f'[ПАУЗА] '
                f'Осталось: {remaining:.1f} сек | '
                f'X: {current_x_cm:.1f} см'
            )

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLinearTrajectory()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Прервано пользователем')
    except Exception as e:
        node.get_logger().error(f'Ошибка: {str(e)}')
    finally:
        # Останавливаем робота при завершении
        node.send_command(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()