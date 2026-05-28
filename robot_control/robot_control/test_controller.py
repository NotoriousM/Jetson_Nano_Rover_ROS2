#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class TestController(Node):
    def __init__(self):
        super().__init__('test_controller')
        self.publisher = self.create_publisher(Float32MultiArray, '/motion_commands', 10)
        
        # Тестовые команды: [скорость, угол]
        self.test_commands = [
            [1.0, 90],    # прямо, средняя скорость
            [0.5, 45],    # мягкий поворот
            [0.0, 90],    # остановка
            [-0.5, 135],  # назад с поворотом
            [1.0, 90],    # снова прямо
        ]
        
        self.timer = self.create_timer(2.0, self.send_test_command)
        self.command_index = 0
        
    def send_test_command(self):
        if self.command_index < len(self.test_commands):
            msg = Float32MultiArray()
            msg.data = self.test_commands[self.command_index]
            self.publisher.publish(msg)
            
            self.get_logger().info(
                f"Отправлена команда {self.command_index + 1}: "
                f"Скорость={msg.data[0]:.1f}, Угол={msg.data[1]}"
            )
            
            self.command_index += 1
        else:
            self.get_logger().info("Все тестовые команды отправлены")
            self.timer.cancel()

def main():
    rclpy.init()
    node = TestController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()