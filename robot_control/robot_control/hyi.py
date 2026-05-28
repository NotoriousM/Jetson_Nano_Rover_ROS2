#!/usr/bin/env python3
#robot_controller_node.py
"""
Узел управления роботом с моделью Аккермана
Управляет 6-колесным роботом через serial-порты
Преобразует высокоуровневые команды [скорость, угол] в индивидуальные команды для каждого колеса
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import struct
import time 
import math

class RobotControllerNode(Node):
    def __init__(self):
        """Инициализация узла управления роботом"""
        super().__init__('robot_controller_node')
        # =========================================================================
        # ДЕКЛАРАЦИЯ ПАРАМЕТРОВ РОБОТА
        # =========================================================================
        # Геометрические параметры робота для модели Аккермана
        self.declare_parameter('wheelbase', 0.780)      # Lx - расстояние между осями
        self.declare_parameter('track_width', 0.808)    # Ly - ширина колеи
        self.declare_parameter('a_distance', 0.390)     # расстояние от передней оси до центра масс
        self.declare_parameter('b_distance', 0.390)     # расстояние от задней оси до центра масс
        self.declare_parameter('max_steering_angle_deg', 60.0)  # максимальный угол поворота колес
        self.declare_parameter('zero_steering_angle', 90.0)     # нейтральное положение сервоприводов
        
        # Параметры подключения к аппаратуре
        self.declare_parameter('baudrate', 115200)      # скорость serial-порта
        self.declare_parameter('timeout', 0.1)          # таймаут соединения
        
        # =========================================================================
        # ПОДПИСКИ НА ТОПИКИ ROS2
        # =========================================================================
        
        # Подписчик на высокоуровневые команды управления [скорость, угол]
        self.motion_sub = self.create_subscription(
            Float32MultiArray,      # тип сообщения
            '/motion_commands',     # название топика
            self.motion_callback,   # функция-обработчик
            10)                     # размер очереди
        
        # =========================================================================
        # ПОЛУЧЕНИЕ ЗНАЧЕНИЙ ПАРАМЕТРОВ
        # =========================================================================
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.a = self.get_parameter('a_distance').value      # передний свес
        self.b = self.get_parameter('b_distance').value      # задний свес
        self.max_steering_rad = math.radians(self.get_parameter('max_steering_angle_deg').value)
        self.zero_steering_angle = self.get_parameter('zero_steering_angle').value  # нейтральное положение
        
        # =========================================================================
        # КОНФИГУРАЦИЯ SERIAL-ПОРТОВ ДЛЯ 6 КОЛЕС
        # =========================================================================
        
        # Поворотные колеса (имеют управление углом поворота)
        self.steering_ports = {
            'front_right': '/dev/ttyROVER_WHEEL_1',  # переднее правое
            'rear_right': '/dev/ttyROVER_WHEEL_3',   # заднее правое
            'front_left': '/dev/ttyROVER_WHEEL_4',   # переднее левое
            'rear_left': '/dev/ttyROVER_WHEEL_6'     # заднее левое
        }
        
        # Центральные колеса (только скорость, без поворота)
        self.center_ports = {
            'right_center': '/dev/ttyROVER_WHEEL_2',  # центральное правое
            'left_center': '/dev/ttyROVER_WHEEL_5'    # центральное левое
        }
        
        # Словари для хранения подключений
        self.serial_connections = {}   # подключения к поворотным колесам
        self.center_connections = {}   # подключения к центральным колесам
        
        # =========================================================================
        # СТАТИСТИКА И МОНИТОРИНГ
        # =========================================================================
        
        self.command_count = 0    # счетчик полученных команд
        self.error_count = 0      # счетчик ошибок
        
        # =========================================================================
        # ИНИЦИАЛИЗАЦИЯ ПОДКЛЮЧЕНИЙ И ТАЙМЕРОВ
        # =========================================================================
        
        # Подключение ко всем serial-портам
        self.connect_serial_ports()
        
        # Таймер безопасности: проверяет потерю связи и останавливает робота
        self.last_command_time = time.time()    # время последней команды
        self.command_timeout = 1.0              # таймаут безопасности (сек)
        self.create_timer(0.05, self.safety_check)  # проверка каждые 50 мс
        
        # Таймер проверки соединений: восстанавливает broken-соединения
        self.create_timer(2.0, self.connection_check)  # проверка каждые 2 сек
        
        # =========================================================================
        # ЛОГИРОВАНИЕ ИНИЦИАЛИЗАЦИИ
        # =========================================================================
        
        self.get_logger().info("🚀 Узел управления роботом инициализирован (только модель Аккермана)")
        self.get_logger().info(f"📐 Параметры робота: Lx={self.wheelbase}м, Ly={self.track_width}м")
        self.get_logger().info(f"🎯 Макс. угол поворота: ±{math.degrees(self.max_steering_rad):.1f}°")
        self.get_logger().info(f"🎯 Нейтральное положение: {self.zero_steering_angle}°")
        self.get_logger().info(f"🔌 Подключено: {len(self.serial_connections)} поворотных, {len(self.center_connections)} центральных колес")

        # Отправка начального положения (90°) при запуске
        self.send_initial_position()

    def send_initial_position(self):
        """Отправка команд для установки сервоприводов в нейтральное положение"""
        initial_commands = [
            0.0, self.zero_steering_angle,  # front_right
            0.0, self.zero_steering_angle,  # rear_right  
            0.0, self.zero_steering_angle,  # front_left
            0.0, self.zero_steering_angle,  # rear_left
            0.0, 0.0                        # center wheels
        ]
        self.send_to_wheels(initial_commands)
        self.get_logger().info(f"🎯 Серводвигатели установлены в нейтральное положение: {self.zero_steering_angle}°")

    def connect_serial_ports(self):
        """Подключение ко всем serial-портам колес"""
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        # Подключение поворотных колес (управление скоростью + углом)
        for port_name, port_path in self.steering_ports.items():
            self.connect_single_port(port_name, port_path, self.serial_connections, baudrate, timeout)
        
        # Подключение центральных колес (только управление скоростью)
        for port_name, port_path in self.center_ports.items():
            self.connect_single_port(port_name, port_path, self.center_connections, baudrate, timeout)

    def connect_single_port(self, port_name, port_path, connection_dict, baudrate, timeout):
        """
        Подключение к одному serial-порту
        """
        if port_name not in connection_dict:
            try:
                # Создание serial-соединения
                ser = serial.Serial(port_path, baudrate=baudrate, 
                                  timeout=timeout, write_timeout=0.1)
                
                # Сохранение информации о подключении
                connection_dict[port_name] = {
                    'serial': ser,        # объект serial
                    'path': port_path,    # путь к устройству
                    'errors': 0           # счетчик ошибок
                }
                self.get_logger().info(f"✅ {port_name}: {port_path}")
                time.sleep(0.1)  # небольшая задержка для стабилизации
                
            except Exception as e:
                self.get_logger().warning(f"❌ {port_name} ({port_path}): {str(e)}")

    def ackermann_calculation(self, V, steering_angle_rad):
        """
        Расчет команд для колес по модели Аккермана
        
        Args:
            V: линейная скорость робота (м/с)
            steering_angle_rad: угол поворота рулевого управления (радианы)
            
        Returns:
            list: команды для 6 колес в формате [speed_fr, angle_fr, speed_rr, angle_rr, 
                  speed_fl, angle_fl, speed_rl, angle_rl, speed_rc, speed_lc]
        """
        
        # Ограничение угла поворота в пределах допустимого
        steering_angle_rad = max(-self.max_steering_rad, min(steering_angle_rad, self.max_steering_rad))
        
        # Прямолинейное движение или остановка
        if abs(steering_angle_rad) < 0.001 or abs(V) < 0.01:
            # Все колеса в нейтральном положении, одинаковые скорости
            return [V, self.zero_steering_angle, V, self.zero_steering_angle, 
                    V, self.zero_steering_angle, V, self.zero_steering_angle, 
                    V, V]
        
        # =========================================================================
        # РАСЧЕТ РАДИУСА ПОВОРОТА
        # =========================================================================
        
        # R = a / tan(δ), где δ - угол поворота рулевого управления
        try:
            R = self.a / math.tan(steering_angle_rad)  # радиус поворота центра масс
        except ZeroDivisionError:
            R = float('inf')  # бесконечный радиус = прямолинейное движение
        
        # Практически прямолинейное движение (очень большой радиус)
        if abs(R) > 1000:
            return [V, self.zero_steering_angle, V, self.zero_steering_angle, 
                    V, self.zero_steering_angle, V, self.zero_steering_angle, 
                    V, V]
        
        # =========================================================================
        # РАСЧЕТ УГЛОВ ПОВОРОТА КОЛЕС (ГЕОМЕТРИЯ АККЕРМАНА)
        # =========================================================================
        
        # Передние колеса:
        # tan(δ_i) = a / (R ± track_width/2)
        angle_fl_rad = math.atan(self.a / (R - self.track_width/2))  # переднее левое
        angle_fr_rad = math.atan(self.a / (R + self.track_width/2))  # переднее правое
        
        # Задние колеса:
        angle_rl_rad = -math.atan(self.b / (R - self.track_width/2))  # заднее левое
        angle_rr_rad = -math.atan(self.b / (R + self.track_width/2))  # заднее правое
        
        # Ограничение углов в пределах допустимого
        angle_fl_rad = max(-self.max_steering_rad, min(angle_fl_rad, self.max_steering_rad))
        angle_fr_rad = max(-self.max_steering_rad, min(angle_fr_rad, self.max_steering_rad))
        angle_rl_rad = max(-self.max_steering_rad, min(angle_rl_rad, self.max_steering_rad))
        angle_rr_rad = max(-self.max_steering_rad, min(angle_rr_rad, self.max_steering_rad))
        
        # =========================================================================
        # РАСЧЕТ СКОРОСТЕЙ КОЛЕС
        # =========================================================================
        
        # Угловая скорость
        omega = V / R
        
        # Скорости колес через угловую скорость
        speed_fl = omega * math.sqrt(self.a**2 + (R - self.track_width/2)**2)
        speed_fr = omega * math.sqrt(self.a**2 + (R + self.track_width/2)**2)
        speed_rl = omega * math.sqrt(self.b**2 + (R - self.track_width/2)**2)
        speed_rr = omega * math.sqrt(self.b**2 + (R + self.track_width/2)**2)
        
        # Центральные колеса (не поворачиваются):
        speed_rc = omega * (R - self.track_width/2)  # правое центральное
        speed_lc = omega * (R + self.track_width/2)  # левое центральное
        
        # =========================================================================
        # ПОДГОТОВКА КОМАНД ДЛЯ ОТПРАВКИ
        # =========================================================================
        
        # Конвертация углов из радиан в градусы и добавление нейтрального смещения
        angle_fl_deg = int(math.degrees(angle_fl_rad) + self.zero_steering_angle)
        angle_fr_deg = int(math.degrees(angle_fr_rad) + self.zero_steering_angle)
        angle_rl_deg = int(math.degrees(angle_rl_rad) + self.zero_steering_angle)
        angle_rr_deg = int(math.degrees(angle_rr_rad) + self.zero_steering_angle)
        
        # Ограничение углов в градусах (0-180 для сервоприводов)
        angle_fl_deg = max(0, min(angle_fl_deg, 180))
        angle_fr_deg = max(0, min(angle_fr_deg, 180))
        angle_rl_deg = max(0, min(angle_rl_deg, 180))
        angle_rr_deg = max(0, min(angle_rr_deg, 180))
        
        # Возврат команд в порядке: [ПП, ЗП, ПЛ, ЗЛ, ЦП, ЦЛ]
        return [speed_fr, angle_fr_deg, speed_rr, angle_rr_deg, 
                speed_fl, angle_fl_deg, speed_rl, angle_rl_deg,
                speed_rc, speed_lc]

    def motion_callback(self, msg):
        """
        Обработчик высокоуровневых команд управления
        """
        # Обновление времени последней команды для системы безопасности
        self.last_command_time = time.time()
        self.command_count += 1
        
        # Проверка формата сообщения
        if len(msg.data) != 2:
            self.get_logger().error("❌ Неверный формат команды. Ожидается [speed, angle]")
            return
            
        try:
            # Извлечение скорости и угла из сообщения
            V = float(msg.data[0])           # линейная скорость (м/с)
            steering_angle_deg = float(msg.data[1])  # угол поворота (градусы от джойстика)
            
            # Конвертация в радианы для расчетов
            steering_angle_rad = math.radians(steering_angle_deg)
            
            # Ограничение скорости для безопасности
            V = max(-2.0, min(V, 2.0))  # ограничение ±2 м/с
            
            # Вычисление индивидуальных команд для колес по модели Аккермана
            wheel_commands = self.ackermann_calculation(V, steering_angle_rad)
            
            # Отправка команд на физические колеса
            self.send_to_wheels(wheel_commands)
            
            # Логирование с троттлингом
            if self.command_count % 10 == 0:
                self.get_logger().info(
                    f"🎮 Управление: V={V:5.2f}м/с, θ={steering_angle_deg:5.1f}°",
                    throttle_duration_sec=0.5
                )
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"💥 Ошибка управления: {str(e)}")

    def send_to_wheels(self, wheel_commands):
        """
        Отправка индивидуальных команд на каждое колесо через serial-порты
        """
        # Проверка корректности формата команд
        if len(wheel_commands) != 10:
            self.get_logger().error(f"❌ Неверный формат команд колес: ожидается 10 значений, получено {len(wheel_commands)}")
            return
            
        success_count = 0  # счетчик успешных отправок
        
        # =========================================================================
        # ПОДГОТОВКА КОМАНД ДЛЯ КАЖДОГО КОЛЕСА
        # =========================================================================
        
        commands = {
            'front_right': {'speed': float(wheel_commands[0]), 'angle': int(wheel_commands[1])},
            'rear_right':  {'speed': float(wheel_commands[2]), 'angle': int(wheel_commands[3])},
            'front_left':  {'speed': float(wheel_commands[4]), 'angle': int(wheel_commands[5])},
            'rear_left':   {'speed': float(wheel_commands[6]), 'angle': int(wheel_commands[7])},
            'right_center': {'speed': float(wheel_commands[8]), 'angle': 0},  # центральные не поворачиваются
            'left_center':  {'speed': float(wheel_commands[9]), 'angle': 0}   # центральные не поворачиваются
        }
        
        # =========================================================================
        # ОТПРАВКА КОМАНД ПОВОРОТНЫМ КОЛЕСАМ (СКОРОСТЬ + УГОЛ)
        # =========================================================================
        
        for port_name, connection_info in self.serial_connections.items():
            if port_name in commands:
                try:
                    ser = connection_info['serial']
                    if ser.is_open:
                        cmd = commands[port_name]
                        # Упаковка данных в бинарный формат: < - little-endian, f - float, h - short
                        packed_data = struct.pack('<fh', cmd['speed'], cmd['angle'])
                        ser.write(packed_data)  # отправка данных
                        ser.flush()  # принудительная отправка буфера
                        success_count += 1
                        connection_info['errors'] = 0  # сброс счетчика ошибок
                        
                        # Детальное логирование для отладки
                        if self.command_count % 20 == 0:
                            self.get_logger().info(f"🔧 {port_name}: speed={cmd['speed']:.2f}, angle={cmd['angle']}°")
                            
                except Exception as e:
                    connection_info['errors'] += 1
                    # Логируем только первые несколько ошибок для каждого соединения
                    if connection_info['errors'] <= 3:
                        self.get_logger().warning(f"⚠️  Поворотное колесо {port_name}: {str(e)}")
        
        # =========================================================================
        # ОТПРАВКА КОМАНД ЦЕНТРАЛЬНЫМ КОЛЕСАМ (ТОЛЬКО СКОРОСТЬ)
        # =========================================================================
        
        for port_name, connection_info in self.center_connections.items():
            if port_name in commands:
                try:
                    ser = connection_info['serial']
                    if ser.is_open:
                        cmd = commands[port_name]
                        # Тот же формат, но угол всегда 0
                        packed_data = struct.pack('<fh', cmd['speed'], cmd['angle'])
                        ser.write(packed_data)
                        ser.flush()
                        success_count += 1
                        connection_info['errors'] = 0
                except Exception as e:
                    connection_info['errors'] += 1
                    if connection_info['errors'] <= 3:
                        self.get_logger().warning(f"⚠️  Центральное колесо {port_name}: {str(e)}")
        
        # Логирование статистики отправки (с троттлингом)
        if self.command_count % 10 == 0:
            self.get_logger().info(
                f"📊 Отправлено команд: {success_count}/6 колес | "
                f"Всего: {self.command_count} | Ошибки: {self.error_count}"
            )

    def safety_check(self):
        """
        Проверка безопасности - автоматическая остановка при потере связи
        """
        time_since_last_command = time.time() - self.last_command_time
        
        # Если с последней команды прошло больше таймаута - ОСТАНОВКА
        if time_since_last_command > self.command_timeout:
            # Отправка команд для установки в нейтральное положение
            neutral_commands = [
                0.0, self.zero_steering_angle,  # front_right
                0.0, self.zero_steering_angle,  # rear_right  
                0.0, self.zero_steering_angle,  # front_left
                0.0, self.zero_steering_angle,  # rear_left
                0.0, 0.0                        # center wheels
            ]
            self.send_to_wheels(neutral_commands)
            
            # Логирование с троттлингом, чтобы не засорять логи
            if self.command_count % 20 == 0:
                self.get_logger().warning("🛑 Аварийная остановка: команды не поступают")

    def connection_check(self):
        """
        Периодическая проверка и восстановление соединений
        """
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        # Проверка поворотных колес
        for port_name in list(self.serial_connections.keys()):
            conn = self.serial_connections[port_name]
            # Переподключение если много ошибок или порт закрыт
            if conn['errors'] > 5 or not conn['serial'].is_open:
                self.get_logger().warning(f"🔁 Переподключение {port_name}...")
                try:
                    conn['serial'].close()
                except:
                    pass
                del self.serial_connections[port_name]
                # Попытка нового подключения
                self.connect_single_port(port_name, self.steering_ports[port_name], 
                                       self.serial_connections, baudrate, timeout)
        
        # Проверка центральных колес
        for port_name in list(self.center_connections.keys()):
            conn = self.center_connections[port_name]
            if conn['errors'] > 5 or not conn['serial'].is_open:
                self.get_logger().warning(f"🔁 Переподключение {port_name}...")
                try:
                    conn['serial'].close()
                except:
                    pass
                del self.center_connections[port_name]
                self.connect_single_port(port_name, self.center_ports[port_name],
                                       self.center_connections, baudrate, timeout)

    def destroy_node(self):
        """
        Корректное завершение работы узла
        """
        self.get_logger().info("🛑 Завершение работы узла управления...")
        
        # Установка сервоприводов в нейтральное положение перед выходом
        neutral_commands = [
            0.0, self.zero_steering_angle,  # front_right
            0.0, self.zero_steering_angle,  # rear_right  
            0.0, self.zero_steering_angle,  # front_left
            0.0, self.zero_steering_angle,  # rear_left
            0.0, 0.0                        # center wheels
        ]
        self.send_to_wheels(neutral_commands)
        time.sleep(0.1)  # небольшая задержка для гарантии отправки
        
        # Закрытие всех serial-соединений
        for conn_dict in [self.serial_connections, self.center_connections]:
            for port_name, connection_info in conn_dict.items():
                try:
                    connection_info['serial'].close()
                    self.get_logger().info(f"🔒 Закрыто соединение: {port_name}")
                except Exception as e:
                    self.get_logger().error(f"💥 Ошибка при закрытии {port_name}: {str(e)}")
        
        super().destroy_node()

def main(args=None):
    """
    Главная функция запуска узла управления роботом
    """
    rclpy.init(args=args)
    
    controller = RobotControllerNode()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("👋 Остановлено пользователем")
    except Exception as e:
        controller.get_logger().error(f"💥 Критическая ошибка узла: {str(e)}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        controller.get_logger().info("✅ Узел управления роботом завершил работу")

if __name__ == '__main__':
    main()