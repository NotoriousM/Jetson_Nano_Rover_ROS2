# ✅ Правильно
cd ~/ros2_ws
colcon build --packages-select robot_control

source install/setup.bash

ros2 run robot_control robot_controller_node

## Packet nodes(Python)
ros2 pkg create rover_nodes \
  --build-type ament_python \
  --dependencies rclpy std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros rover_interfaces
### Файл для понимания программной архитектуры для мобильного робота
## robot_control_v3
## Запуск
*Комнада для настроки рабочего окружения ROS2:*
```bash
source install/setup.bash
```
*В первом терминале:*
```bash
ros2 launch rover_nodes robot_bringup.launch.py
```
*Во втором терминале:*
```bash
ros2 launch rover_nodes keyboard_control.launch.py
```
## rover_description-Виртуальная модель мбильного четырех колесного робота
## Запуск