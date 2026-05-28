# robot_control_v3

ROS2 пакеты для управления 6-колёсным ровером с моделью Аккермана.

**Главные изменения относительно v2:**
- Удалены программные `estop_node` и `watchdog_node` (дублировали друг друга
  и не знали о реальном состоянии железа).
- Добавлен `flag_safety_node`, который читает аппаратный `usb_stop_flag` от
  STM32 (поле `SendData.flag` через USB CDC). Источник правды — нижний уровень.
- Добавлена папка `ethernet/` с готовыми скриптами для настройки сети
  PC ↔ Jetson Nano (Ethernet и WiFi).
- Полный туториал с побайтовой отладкой (`TUTORIAL.md`).

## Аппаратура

- 1 × Jetson Nano B01 dev kit (ROS2 Foxy на Ubuntu 20.04)
- 6 × STM32F103C8T6 (по одному на колесо, USB CDC)
- 6 × двигателей (с энкодерами)
- 4 × сервоприводов (передние и задние поворотные, средние — нет)
- 6 × датчиков тока

## Структура проекта

```
robot_control_v3/
├── README.md                                    # этот файл
├── TUTORIAL.md                                  # пошаговый чек-лист с байтами
│
├── rover_interfaces/                            # ROS2 типы (CMake)
│   ├── msg/
│   │   ├── MotionCommand.msg
│   │   ├── WheelCommand.msg
│   │   ├── WheelState.msg                       # содержит uint8 flag от STM32
│   │   ├── RoverWheelsState.msg                 # все 6 колёс именованными полями
│   │   ├── RoverStatus.msg                      # агрегат для удобного мониторинга
│   │   └── TrajectoryStatus.msg
│   └── srv/
│       ├── ResetOdometry.srv
│       └── StartStraightTrajectory.srv
│
├── rover_nodes/                                 # ROS2 узлы (Python)
│   ├── rover_nodes/
│   │   ├── flag_safety_node.py                  # 🆕 ЗАМЕНЯЕТ estop+watchdog
│   │   ├── ackermann_calculator_node.py
│   │   ├── serial_controller_node.py            # USB CDC мост (6 потоков)
│   │   ├── odometry_node.py
│   │   ├── keyboard_controller_node.py
│   │   ├── dualshock4_controller_node.py
│   │   ├── straight_trajectory_node.py
│   │   └── rover_status_node.py
│   ├── config/robot_params.yaml
│   └── launch/
│       ├── robot_bringup.launch.py              # основная архитектура
│       ├── keyboard_control.launch.py           # отдельно: клавиатура
│       ├── dualshock4_control.launch.py         # отдельно: геймпад
│       └── trajectory_control.launch.py         # отдельно: автодвижение
│
└── ethernet/                                    # 🆕 связь PC ↔ Jetson
    ├── README.md                                # настройка сети
    ├── netplan-pc.yaml                          # статический IP для PC
    ├── netplan-jetson.yaml                      # статический IP + WiFi
    ├── fastdds_profile.xml                      # привязка DDS к eth0
    ├── setup_network.sh                         # автонастройка
    ├── env_pc.sh / env_jetson.sh                # ROS2 переменные
    ├── ssh_robot.sh                             # удобный SSH
    └── verify.sh                                # диагностика
```

## Поток данных

```
[keyboard]     ┐
[dualshock4]   ├──→ /motion_commands ──→ flag_safety_node ←── /wheels/state.flag
[trajectory]   ┘                                   │           (от STM32!)
                                                   │
                                                   ▼
                              /motion_commands_safe ──→ ackermann_calculator_node
                                                                  │
                                                                  ▼
                                          /wheel/{name}/cmd × 6 ──→ serial_controller_node
                                                                              │
                                                                              ▼
                                                              USB CDC ↔ 6 × STM32
                                                                              │
                                                                              ▼
                                                /wheels/state ──→ odometry_node
                                                                              │
                                                                              ▼
                                                                       /odom + /tf
                                                                              │
                                                                              ▼
                                                                rover_status_node
                                                                              │
                                                                              ▼
                                                                  /rover/status
```

## Бинарный протокол STM32 (USB CDC)

Соответствует прошивке `Rover/main.c`:

```c
// Jetson → STM32 (RX), 6 байт
typedef struct {
  float    speed;     // 4 байта — Motor_velocity
  uint16_t angle;     // 2 байта — PWM_servo (0..180)
} WheelData;

// STM32 → Jetson (TX), 5 байт
typedef struct {
  float   speed;      // 4 байта — encoder_getVelocity()
  uint8_t flag;       // 1 байт  — usb_stop_flag (АППАРАТНАЯ ЗАЩИТА!)
} SendData;
```

В Python (`serial_controller_node`):
```python
TX_FORMAT = '<fH'   # struct.pack('<fH', speed, int(angle)) → 6 байт
RX_FORMAT = '<fB'   # struct.unpack('<fB', data) → (speed, flag)
```

Флаг `usb_stop_flag` поднимается прошивкой при превышении тока, потере
энкодера или другой аппаратной ошибке. Узел `flag_safety_node` подписан
на `/wheels/state` и моментально блокирует поток команд, если хотя бы
одно колесо подняло флаг.

## Сборка

```bash
mkdir -p ~/ros2_ws/src
cp -r robot_control_v3/* ~/ros2_ws/src/

# 1. Сначала интерфейсы:
cd ~/ros2_ws
colcon build --packages-select rover_interfaces
source install/setup.bash

# 2. Затем узлы (зависят от rover_interfaces):
colcon build --packages-select rover_nodes
source install/setup.bash

# Постоянно:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Зависимости

```bash
sudo apt install python3-serial python3-evdev ros-foxy-tf2-ros
pip3 install pyserial evdev numpy
```

## udev правила для STM32

`/etc/udev/rules.d/99-rover-wheels.rules` — см. `TUTORIAL.md` § 4.1.

## Запуск

### На Jetson:
```bash
ros2 launch rover_nodes robot_bringup.launch.py
```

В другом терминале — один из контроллеров:
```bash
ros2 launch rover_nodes keyboard_control.launch.py
ros2 launch rover_nodes dualshock4_control.launch.py
ros2 launch rover_nodes trajectory_control.launch.py
```

### Связь с PC:
```bash
# 1. Один раз настроить сеть (см. ethernet/README.md):
cd ethernet/
sudo ./setup_network.sh pc       # на PC
sudo ./setup_network.sh jetson   # на Jetson

# 2. На PC:
source ethernet/env_pc.sh
ros2 topic list                  # видит топики с Jetson
ros2 run rviz2 rviz2             # визуализация
```

## Удобные команды для отладки

```bash
ros2 topic echo /rover/status               # общий статус
ros2 topic echo /wheels/state               # все 6 колёс
ros2 topic echo /odom                       # одометрия
ros2 topic echo /safety/active_flags        # какие флаги от STM32

ros2 topic hz /odom                         # частота (~50 Гц)
ros2 topic hz /wheels/state                 # частота (~20 Гц)

ros2 service call /reset_odometry rover_interfaces/srv/ResetOdometry \
  "{x: 0.0, y: 0.0, yaw_deg: 0.0}"

ros2 service call /start_straight_trajectory \
  rover_interfaces/srv/StartStraightTrajectory \
  "{distance: 2.0, speed: 0.5}"
```

## Подробный туториал

См. `TUTORIAL.md` — пошаговая проверка от байтов на USB до управления с PC,
с командами для каждого уровня и шпаргалкой по диагностике.
