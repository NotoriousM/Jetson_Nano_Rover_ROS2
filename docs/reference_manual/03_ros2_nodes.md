# Глава 3. ROS2-ноды, топики и сервисы

[◀ Установка](02_installation.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Протокол STM32 ▶](04_protocol_stm32.md)

---

## Содержание

- [3.1 Граф нод](#31-граф-нод)
- [3.2 Ноды](#32-ноды)
  - [flag_safety_node](#flag_safety_node)
  - [ackermann_calculator_node](#ackermann_calculator_node)
  - [serial_controller_node](#serial_controller_node)
  - [odometry_node](#odometry_node)
  - [keyboard_controller_node](#keyboard_controller_node)
  - [dualshock4_controller_node](#dualshock4_controller_node)
  - [straight_trajectory_node](#straight_trajectory_node)
  - [rover_status_node](#rover_status_node)
- [3.3 Полная таблица топиков](#33-полная-таблица-топиков)
- [3.4 Сервисы](#34-сервисы)
- [3.5 Кастомные msg/srv типы](#35-кастомные-msgsrv-типы)
- [3.6 Launch-файлы](#36-launch-файлы)
- [3.7 Параметры нод](#37-параметры-нод)

---

## 3.1 Граф нод

```
Внешние источники управления:
  keyboard_controller_node ──┐
  dualshock4_controller_node─┤──► /motion_commands
  straight_trajectory_node ──┘          │
                                         ▼
                              ┌─────────────────────┐
                              │  flag_safety_node   │
                              │  Фильтр по STM32-   │
                              │  флагу usb_stop_flag│
                              └─────────┬───────────┘
                                        │ /motion_commands_safe
                                        ▼
                              ┌─────────────────────┐
                              │ackermann_calculator │
                              │   _node             │
                              │ Математика Аккермана│
                              └─────────┬───────────┘
                                        │ /wheel/{name}/cmd × 6
                                        ▼
                              ┌─────────────────────┐
                              │serial_controller    │
                              │   _node             │
                              │ Queue×6, Thread×6   │
                              └──────┬──────────────┘
                                     │           │
                         USB CDC ×6  │           │ /wheel/{name}/state
                                     ▼           ▼
                               [6×STM32]  ┌─────────────────┐
                                          │ odometry_node   │──► /odom + /tf
                                          ├─────────────────┤
                                          │ flag_safety_node│◄── /wheels/state
                                          ├─────────────────┤
                                          │rover_status_node│──► /rover/status
                                          └─────────────────┘
```

---

## 3.2 Ноды

### flag_safety_node

**Файл:** `rover_nodes/flag_safety_node.py`  
**Роль:** Аппаратная защита. Читает `usb_stop_flag` от STM32. Блокирует поток команд при аварии.

> Заменяет программные `estop_node` / `watchdog_node` (deprecated).  
> **Источник правды — нижний уровень:** STM32 выставляет `flag≠0` при перегреве, сбое энкодера, превышении тока.

**Подписки:**

| Топик | Тип | Описание |
|-------|-----|---------|
| `/motion_commands` | `MotionCommand` | Команды от операторов |
| `/wheels/state` | `RoverWheelsState` | Состояние колёс (читает `flag`) |
| `/safety/clear` | `std_msgs/Bool` | Ручной сброс блокировки |

**Публикации:**

| Топик | Тип | Описание |
|-------|-----|---------|
| `/motion_commands_safe` | `MotionCommand` | Прошедшая фильтрацию команда |
| `/safety/active` | `std_msgs/Bool` | Флаг блокировки |
| `/safety/active_flags` | `std_msgs/String` | Детали: какие колёса дали сигнал |

**Логика:**
```
flag≠0 у ЛЮБОГО колеса  →  /motion_commands_safe = (0.0, 0.0)
                        →  /safety/active = True
                        →  /safety/active_flags = "front_left=0x01 ..."

flag=0 у ВСЕХ колёс    →  пропустить команду (если auto_clear=true)
                        →  /safety/active = False
```

**Параметры:**
```yaml
auto_clear_when_flag_resets: true   # снимать блок автоматически
publish_rate_hz: 20.0
```

---

### ackermann_calculator_node

**Файл:** `rover_nodes/ackermann_calculator_node.py`  
**Роль:** Чистая математика. Нет I/O, нет потоков — только вычисление кинематики Аккермана.

**Подписки:**

| Топик | Тип |
|-------|-----|
| `/motion_commands_safe` | `MotionCommand` |

**Публикации:**

| Топик | Тип | Примечание |
|-------|-----|-----------|
| `/wheel/front_left/cmd` | `WheelCommand` | speed + angle серво |
| `/wheel/middle_left/cmd` | `WheelCommand` | speed + angle=90° (не поворотное) |
| `/wheel/rear_left/cmd` | `WheelCommand` | speed + angle серво |
| `/wheel/front_right/cmd` | `WheelCommand` | speed + angle серво |
| `/wheel/middle_right/cmd` | `WheelCommand` | speed + angle=90° |
| `/wheel/rear_right/cmd` | `WheelCommand` | speed + angle серво |

**Формулы:**
```
R      = a / tan(δ)                   # радиус ICR
α_L    = atan(a / (R + W/2))          # угол серво FL
α_R    = atan(a / (R - W/2))          # угол серво FR
β_L    = atan(b / (R + W/2))          # угол серво RL
β_R    = atan(b / (R - W/2))          # угол серво RR

R_i    = √((x_off_i)² + (y_off_i - R)²)  # радиус i-го колеса от ICR
v_i    = v · R_i / |R|               # скорость i-го колеса
```

**Параметры:**
```yaml
wheelbase:              0.807
track_width:            0.779
a_distance:             0.4035
b_distance:             0.4035
initial_pos_servo_deg:  90.0
max_speed:              2.0
max_steering_angle:     35.0
```

---

### serial_controller_node

**Файл:** `rover_nodes/serial_controller_node.py`  
**Роль:** Мост ROS2 ↔ 6×STM32 по USB CDC. Многопоточная архитектура.

**Подписки:**

| Топик | Тип |
|-------|-----|
| `/wheel/{name}/cmd` | `WheelCommand` |

**Публикации:**

| Топик | Тип | Частота |
|-------|-----|---------|
| `/wheel/{name}/state` | `WheelState` | 20 Гц |
| `/wheels/state` | `RoverWheelsState` | 20 Гц |

**Потоковая модель:**
```
Поток 0 (ROS Main Executor):
    create_timer(50ms) → _publish()
    _publish():
        snapshot = copy(_data)   # под _data_lock
        публикует WheelState × 6 и RoverWheelsState

Потоки 1-6 (daemon, по одному на STM32):
    while _running:
        если нет соединения → _connect() (retry 2с)
        TX: _cmd_queue.get_nowait() → pack('<fH') → ser.write()
        RX: ser.read(in_waiting)   → _rxbuf → _parse_packets()
        sleep(1ms)
```

**Queue(maxsize=1) — ключевая деталь:**
```python
def send_command(speed, angle):
    try: _cmd_queue.get_nowait()  # выбросить устаревшую
    except Empty: pass
    _cmd_queue.put_nowait((speed, angle))  # положить свежую
```
Это гарантирует: STM32 получает только **последнюю** команду, старые не накапливаются.

**Параметры:**
```yaml
baudrate: 115200
timeout:  0.1
initial_pos_servo_deg: 90.0
port_front_right:  /dev/ttyROVER_WHEEL_1
port_middle_right: /dev/ttyROVER_WHEEL_2
port_rear_right:   /dev/ttyROVER_WHEEL_3
port_front_left:   /dev/ttyROVER_WHEEL_4
port_middle_left:  /dev/ttyROVER_WHEEL_5
port_rear_left:    /dev/ttyROVER_WHEEL_6
```

---

### odometry_node

**Файл:** `rover_nodes/odometry_node.py`  
**Роль:** Дифференциальная одометрия по **средним** (неповоротным) колёсам.

> Используются `msg.middle_left.speed` и `msg.middle_right.speed` — именованные поля,
> а не индексы. Это исключает ошибку перепутанных позиций из старого кода.

**Подписки:**

| Топик | Тип |
|-------|-----|
| `/wheels/state` | `RoverWheelsState` |
| `/initialpose` | `PoseWithCovarianceStamped` (RViz "2D Pose Estimate") |

**Публикации:**

| Топик | Тип | Описание |
|-------|-----|---------|
| `/odom` | `nav_msgs/Odometry` | Поза + скорость в `odom` фрейме |
| `/tf` | TF2 | Transform `odom` → `base_link` |

**Сервисы:**

| Сервис | Тип |
|--------|-----|
| `/reset_odometry` | `ResetOdometry` |

**Алгоритм:**
```
v_left  = middle_left.speed
v_right = middle_right.speed

v     = (v_left + v_right) / 2        # линейная скорость
ω     = (v_right - v_left) / W        # угловая скорость

x  += v · cos(θ) · dt
y  += v · sin(θ) · dt
θ  += ω · dt                          # atan2(sin,cos) для нормализации
```

**Параметры:**
```yaml
track_width:             0.779
wheel_radius:            0.1
speed_conversion_factor: 1.0     # коэффициент к единицам энкодера
publish_rate:            50      # Гц
odom_frame_id:           odom
base_frame_id:           base_link
```

---

### keyboard_controller_node

**Файл:** `rover_nodes/keyboard_controller_node.py`  
**Роль:** Управление через stdin терминала.

**Публикации:**

| Топик | Тип |
|-------|-----|
| `/motion_commands` | `MotionCommand` |
| `/safety/clear` | `std_msgs/Bool` |

**Клавиши:**

| Клавиша | Действие |
|---------|---------|
| `W` / `S` | Скорость +0.2 / -0.2 м/с |
| `A` / `D` | Угол руля -3° / +3° |
| `C` | Центрировать руль (угол = 0) |
| `Space` | Полный стоп (v=0, угол=0) |
| `X` | Ручной блок защиты |
| `R` | Снять ручной блок |
| `Q` | Выход из ноды |

**Потоки:**
```
Поток 0 (ROS): таймер 20 Гц → publish(текущее состояние)
Поток 1 (stdin): блокирующий read → обновить состояние
```

---

### dualshock4_controller_node

**Файл:** `rover_nodes/dualshock4_controller_node.py`  
**Роль:** Управление геймпадом DualShock 4 через `/dev/input/js0`.

**Публикации:** `/motion_commands`, `/safety/clear`

**Оси геймпада:**

| Ось | Управление | Нелинейность |
|-----|-----------|-------------|
| Left stick Y | Скорость (±2.0 м/с) | линейная |
| Right stick X | Угол руля (±35°) | `steering_exponent=1.5` |
| L1 кнопка | Ручной блок | — |
| R1 кнопка | Снять блок | — |

**Параметры:**
```yaml
max_speed:           2.0
max_steering_angle:  30.0
deadzone:            0.1        # игнорировать стики < 10%
steering_exponent:   1.5        # нелинейность (1.0 = линейная)
publish_rate_hz:     20.0
```

---

### straight_trajectory_node

**Файл:** `rover_nodes/straight_trajectory_node.py`  
**Роль:** Автономное движение по прямой на заданную дистанцию с профилем торможения.

**Подписки:** `/odom` (`nav_msgs/Odometry`)  
**Публикации:** `/motion_commands`, `/trajectory/status`

**Сервисы:**

| Сервис | Тип |
|--------|-----|
| `/start_straight_trajectory` | `StartStraightTrajectory` |

```bash
# Пример вызова: проехать 2 метра вперёд со скоростью 0.5 м/с
ros2 service call /start_straight_trajectory \
  rover_interfaces/srv/StartStraightTrajectory \
  '{distance: 2.0, speed: 0.5}'

# Назад на 1 метр
ros2 service call /start_straight_trajectory \
  rover_interfaces/srv/StartStraightTrajectory \
  '{distance: -1.0, speed: 0.3}'
```

**Алгоритм:**
```
1. Запомнить (x₀, y₀) из /odom
2. Публиковать motion_command каждые 50мс
3. d = √((x-x₀)² + (y-y₀)²)  — пройденная дистанция
4. При d ≥ (target - brake_distance):
      скорость = max(brake_speed, v_cruise · (target-d)/brake_distance)
5. При d ≥ target: стоп
```

**Параметры:**
```yaml
publish_rate_hz:   20.0
brake_distance:    0.3      # начало замедления за 30 см до цели
brake_speed:       0.1      # скорость в зоне замедления
finish_tolerance:  0.05     # допуск окончания (5 см)
max_speed:         1.5      # ограничение скорости траектории
```

---

### rover_status_node

**Файл:** `rover_nodes/rover_status_node.py`  
**Роль:** Агрегирует данные всех нод в один топик для удобного мониторинга.

**Подписки:**
- `/odom`, `/wheels/state`, `/safety/active`, `/safety/active_flags`,
  `/motion_commands_safe`, `/trajectory/status`

**Публикации:**

| Топик | Тип | Частота |
|-------|-----|---------|
| `/rover/status` | `RoverStatus` | 1 Гц |

```bash
# Мониторинг одной командой
ros2 topic echo /rover/status
```

---

## 3.3 Полная таблица топиков

| Топик | Тип сообщения | Publisher | Subscriber(s) |
|-------|:-------------:|-----------|--------------|
| `/motion_commands` | `MotionCommand` | kbd / ds4 / traj | flag_safety |
| `/motion_commands_safe` | `MotionCommand` | flag_safety | ackermann_calc |
| `/wheel/{name}/cmd` | `WheelCommand` | ackermann_calc | serial_ctrl |
| `/wheel/{name}/state` | `WheelState` | serial_ctrl | flag_safety, rover_status |
| `/wheels/state` | `RoverWheelsState` | serial_ctrl | odometry, flag_safety, rover_status |
| `/odom` | `nav_msgs/Odometry` | odometry | straight_traj, rover_status, nav2 |
| `/tf` | TF2 | odometry | RViz2 |
| `/safety/active` | `std_msgs/Bool` | flag_safety | rover_status |
| `/safety/active_flags` | `std_msgs/String` | flag_safety | rover_status |
| `/safety/clear` | `std_msgs/Bool` | kbd / ds4 | flag_safety |
| `/trajectory/status` | `TrajectoryStatus` | straight_traj | rover_status |
| `/rover/status` | `RoverStatus` | rover_status | мониторинг |
| `/initialpose` | `PoseWithCovStamped` | RViz2 | odometry |

---

## 3.4 Сервисы

| Сервис | Тип | Параметры запроса | Ответ |
|--------|-----|-------------------|-------|
| `/reset_odometry` | `ResetOdometry` | `x`, `y`, `yaw_deg` | `success`, `message` |
| `/start_straight_trajectory` | `StartStraightTrajectory` | `distance`, `speed` | `success`, `message` |

---

## 3.5 Кастомные msg/srv типы

### MotionCommand.msg
```
float32 linear_velocity   # м/с, + вперёд, − назад, диапазон [-2.0, 2.0]
float32 steering_angle    # °,   + право,  − лево,  диапазон [-35.0, 35.0]
string  source            # 'keyboard' | 'ds4' | 'trajectory' | 'estop'
```

### WheelCommand.msg
```
string  wheel_name        # 'front_left'|'middle_left'|'rear_left'|...
float32 speed_cmd         # целевая скорость колеса (м/с)
float32 angle_cmd         # угол серво 0..180° (для middle_* = 90°)
```

### WheelState.msg
```
string  wheel_name
float32 speed             # скорость с энкодера (м/с)
float32 angle_cmd         # угол, который мы отправили серво
uint8   flag              # usb_stop_flag от STM32
bool    is_connected      # COM-порт подключён и работает
builtin_interfaces/Time stamp
```

### RoverWheelsState.msg
```
WheelState front_left
WheelState middle_left
WheelState rear_left
WheelState front_right
WheelState middle_right
WheelState rear_right
builtin_interfaces/Time stamp
uint8 connected_count     # 0..6
```

### RoverStatus.msg
```
string  state             # 'idle'|'driving'|'safety_block'|'trajectory'
bool    safety_active
string  safety_flags      # 'OK' | 'front_left=0x01 ...'
string  active_source     # 'keyboard'|'ds4'|'trajectory'|'safety'|'none'
uint8   wheels_connected
float32 current_speed
float32 current_steering
float32 distance_traveled
builtin_interfaces/Time stamp
```

### TrajectoryStatus.msg
```
string  state             # 'idle'|'running'|'finished'|'aborted'
float32 target_distance
float32 traveled_distance
float32 remaining_distance
float32 progress_percent  # 0..100
float32 elapsed_time
builtin_interfaces/Time stamp
```

### ResetOdometry.srv
```
# Запрос
float64 x
float64 y
float64 yaw_deg           # 0 = смотрит вдоль +X
---
# Ответ
bool   success
string message
```

### StartStraightTrajectory.srv
```
# Запрос
float32 distance          # >0 вперёд, <0 назад
float32 speed             # всегда положительная
---
# Ответ
bool   success
string message
```

---

## 3.6 Launch-файлы

### robot_bringup.launch.py

Запускает базовый стек. Порядок запуска с задержками:

```
t=0с:  flag_safety_node       (слушает /motion_commands сразу)
t=0с:  ackermann_calculator   (готов считать кинематику)
t=1с:  serial_controller_node (ждёт USB enum)
t=2с:  odometry_node          (ждёт первых /wheels/state)
t=3с:  rover_status_node      (агрегирует всё)
t=4с:  лог "✅ System READY"
```

```bash
ros2 launch rover_nodes robot_bringup.launch.py
ros2 launch rover_nodes robot_bringup.launch.py log_level:=debug
```

### keyboard_control.launch.py

```bash
ros2 launch rover_nodes keyboard_control.launch.py
```

Запускает `robot_bringup` + `keyboard_controller_node`.

### dualshock4_control.launch.py

```bash
ros2 launch rover_nodes dualshock4_control.launch.py
```

### trajectory_control.launch.py

```bash
ros2 launch rover_nodes trajectory_control.launch.py
```

---

## 3.7 Параметры нод

Все параметры в `robot_control_v3/rover_nodes/config/robot_params.yaml`.  
Каждый узел загружает этот файл через `parameters=[prms]` в launch.

Изменение параметра без пересборки:
```bash
# Поменять max_speed на лету (не рекомендуется для кинематических параметров)
ros2 param set /ackermann_calculator_node max_speed 1.0

# Просмотр всех параметров ноды
ros2 param list /serial_controller_node
ros2 param get  /serial_controller_node port_front_right
```

---

[◀ Установка](02_installation.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Протокол STM32 ▶](04_protocol_stm32.md)
