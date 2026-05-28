# Глава 2. Установка и сборка

[◀ Аппаратура](01_hardware.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Ноды ▶](03_ros2_nodes.md)

---

## Содержание

- [2.1 Системные требования](#21-системные-требования)
- [2.2 Установка ROS2 Foxy](#22-установка-ros2-foxy)
- [2.3 Клонирование репозитория](#23-клонирование-репозитория)
- [2.4 Python-зависимости](#24-python-зависимости)
- [2.5 Сборка пакетов](#25-сборка-пакетов)
- [2.6 Проверка установки](#26-проверка-установки)
- [2.7 Настройка .bashrc](#27-настройка-bashrc)
- [2.8 Права на COM-порты](#28-права-на-com-порты)
- [2.9 Решение типичных ошибок](#29-решение-типичных-ошибок)

---

## 2.1 Системные требования

| Компонент | Требование | Проверка |
|-----------|-----------|----------|
| ОС | Ubuntu 20.04 | `lsb_release -a` |
| ROS2 | Foxy | `ros2 --version` |
| Python | ≥ 3.8 | `python3 --version` |
| pyserial | ≥ 3.5 | `pip3 show pyserial` |
| colcon | любая | `colcon --version` |
| xacro | ≥ 2.0 | `ros2 pkg list \| grep xacro` |

---

## 2.2 Установка ROS2 Foxy

```bash
# 1. Системные зависимости
sudo apt update
sudo apt install -y software-properties-common curl gnupg2 lsb-release

# 2. Ключ GPG
sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# 3. Репозиторий
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# 4. Установка
sudo apt update
sudo apt install -y \
  ros-foxy-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-foxy-xacro \
  ros-foxy-joint-state-publisher-gui

# 5. rosdep
sudo rosdep init
rosdep update
```

---

## 2.3 Клонирование репозитория

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/NotoriousM/Jetson_Nano_Rover_ROS2.git
```

---

## 2.4 Python-зависимости

```bash
pip3 install pyserial

# Для DualShock 4 (опционально, только если используете геймпад)
pip3 install evdev

# Для симуляции (опционально)
sudo apt install -y ros-foxy-gazebo-ros-pkgs
```

---

## 2.5 Сборка пакетов

> **Важно:** `rover_interfaces` (кастомные msg/srv) должен быть собран **раньше** `rover_nodes`.

```bash
cd ~/ros2_ws

# Шаг 1 — собрать интерфейсы
colcon build --packages-select rover_interfaces
source install/setup.bash

# Шаг 2 — собрать ноды
colcon build --packages-select rover_nodes
source install/setup.bash

# Шаг 3 — собрать описание робота (для симуляции)
colcon build --packages-select rover_description
source install/setup.bash
```

**Пересборка после изменений:**

```bash
# Только изменённые пакеты (быстро)
colcon build --packages-select rover_nodes
source install/setup.bash

# Полная чистая пересборка (медленно, если что-то сломалось)
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

---

## 2.6 Проверка установки

```bash
# Пакеты найдены
ros2 pkg list | grep rover
# rover_interfaces
# rover_nodes
# rover_description

# Кастомные типы доступны
ros2 interface show rover_interfaces/msg/MotionCommand
ros2 interface show rover_interfaces/msg/WheelState
ros2 interface show rover_interfaces/srv/StartStraightTrajectory

# Ноды запускаются (без железа — STM32 будет в режиме симуляции)
ros2 run rover_nodes ackermann_calculator_node &
sleep 1
ros2 node list   # должен показать /ackermann_calculator_node
ros2 node kill /ackermann_calculator_node
```

---

## 2.7 Настройка .bashrc

```bash
cat >> ~/.bashrc << 'EOF'

# ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Рабочий стол
source ~/ros2_ws/install/setup.bash

# ROS Domain (должен совпадать с PC-оператором)
export ROS_DOMAIN_ID=42

# FastDDS профиль для сети Jetson ↔ PC (опционально)
# export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/src/Jetson_Nano_Rover_ROS2/robot_control_v3/ethernet/fastdds_profile.xml
EOF

source ~/.bashrc
```

---

## 2.8 Права на COM-порты

Без этого нода serial_controller не откроет `/dev/ttyROVER_WHEEL_*`:

```bash
sudo usermod -a -G dialout $USER

# Проверить текущие права
ls -la /dev/ttyROVER_WHEEL_*
# crw-rw---- 1 root dialout ... /dev/ttyROVER_WHEEL_1
```

> **Перелогинтесь** после `usermod` — изменения применяются только в новой сессии.

---

## 2.9 Решение типичных ошибок

| Ошибка | Причина | Решение |
|--------|---------|---------|
| `ModuleNotFoundError: rover_interfaces` | Не источника `install/setup.bash` | `source ~/ros2_ws/install/setup.bash` |
| `Package 'rover_nodes' not found` | Не собран | `colcon build --packages-select rover_nodes` |
| `Permission denied: /dev/ttyACM0` | Нет прав | `sudo usermod -a -G dialout $USER` |
| `[xacro] No module named 'lxml'` | Нет lxml | `pip3 install lxml` |
| Gazebo не запускается | OpenGL VM | `export LIBGL_ALWAYS_SOFTWARE=1; export MESA_GL_VERSION_OVERRIDE=3.3` |
| `colcon build` завис | colcon daemon | `colcon build --executor sequential` |

---

[◀ Аппаратура](01_hardware.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Ноды ▶](03_ros2_nodes.md)
