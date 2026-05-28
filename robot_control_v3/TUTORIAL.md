# TUTORIAL — robot_control_v3

Полное руководство по проверке работоспособности проекта на 6-колёсном
ровере с моделью Аккермана. Идём снизу вверх — от байтов на USB до ROS2
команд с PC.

---

## Содержание

1. [Что мы проверяем — карта системы](#1-карта-системы)
2. [Уровень 0: STM32 в одиночку (без Jetson)](#2-уровень-0-stm32)
3. [Уровень 1: один STM32 ↔ Jetson (USB CDC, побайтно)](#3-уровень-1-один-stm32--jetson)
4. [Уровень 2: все 6 STM32 ↔ Jetson (`serial_controller_node`)](#4-уровень-2-все-6-stm32)
5. [Уровень 3: вычисление кинематики (`ackermann_calculator_node`)](#5-уровень-3-вычисление-кинематики)
6. [Уровень 4: одометрия (`odometry_node`)](#6-уровень-4-одометрия)
7. [Уровень 5: защита от STM32 (`flag_safety_node`)](#7-уровень-5-защита-от-stm32)
8. [Уровень 6: контроллеры на Jetson (клавиатура / DS4 / траектория)](#8-уровень-6-контроллеры)
9. [Уровень 7: PC ↔ Jetson по сети (Ethernet/WiFi)](#9-уровень-7-pc--jetson)
10. [Полный сквозной тест](#10-полный-сквозной-тест)
11. [Шпаргалка по диагностике](#11-шпаргалка-по-диагностике)

---

## 1. Карта системы

```
┌─────────────────────────────────────────────────────────────────────────┐
│  PC (оператор)            Ethernet 1 Гбит                Jetson Nano     │
│  192.168.10.1     ◄─────────────────────────────►        192.168.10.2    │
│                          ROS_DOMAIN_ID=42                                 │
│                          FastDDS UDP 7400-7500                            │
└─────────────────────────────────────────────────────────────────────────┘
                                                              │ USB CDC
                                                              ▼
                                            ┌──────────────────────────────┐
                                            │  6 × STM32F103C8T6           │
                                            │  TX: 6 байт WheelData        │
                                            │  RX: 5 байт SendData         │
                                            └──────────────────────────────┘
```

Все 6 STM32 общаются с Jetson по USB Communication Device Class (CDC) —
для Linux это виртуальные последовательные порты `/dev/ttyACM0..5`.

---

## 2. Уровень 0: STM32

**Задача:** убедиться, что прошивка `main.c` работает корректно ДО подключения
к Jetson.

### Тест 2.1 — питание и прошивка

1. Подключи STM32 к питанию.
2. На отладочной плате должны мигать индикаторы.
3. Прошивка из `Rover/main.c` запускает в `main()`:
   - `MX_USB_DEVICE_Init()` — USB CDC появляется как `/dev/ttyACMx`
   - таймеры энкодера, ШИМ, ADC

### Тест 2.2 — STM32 видится в системе

С обычного Linux PC (или с Jetson) подключи **один** STM32 кабелем:

```bash
ls /dev/ttyACM*
# /dev/ttyACM0
dmesg | tail -20
# [12345.67] cdc_acm 1-2:1.0: ttyACM0: USB ACM device
```

Если устройство НЕ появилось:
- Проверь, что в прошивке STM32 включен USB (модуль `usb_device.c` в проекте Rover).
- Кабель должен быть data-cable, не charging-only.

### Тест 2.3 — структуры в прошивке (ссылка на `main.c`)

Прошивка определяет два бинарных пакета:

```c
// Jetson → STM32  (RX на стороне STM32) — 6 байт
#pragma pack(push, 1)
typedef struct {
  float    speed;    // 4 байта — целевая скорость колеса
  uint16_t angle;    // 2 байта — угол сервопривода (0..180)
} WheelData;
#pragma pack(pop)

// STM32 → Jetson  (TX на стороне STM32) — 5 байт
#pragma pack(push, 1)
typedef struct {
  float   speed;     // 4 байта — encoder_getVelocity()
  uint8_t flag;      // 1 байт  — usb_stop_flag (АППАРАТНАЯ ЗАЩИТА)
} SendData;
#pragma pack(pop)
```

`#pragma pack(push, 1)` отключает выравнивание — структура занимает РОВНО
6/5 байт без padding. Это критично, потому что Python `struct` тоже работает
без padding, и любая лишняя дырка ломала бы парсинг.

`flag` поднимает прошивка при превышении тока, потере энкодера или другой
аппаратной ошибке. Поэтому в robot_control_v3 нет программного e-stop —
есть `flag_safety_node`, который реагирует на этот реальный hardware-флаг.

---

## 3. Уровень 1: один STM32 ↔ Jetson

**Задача:** проверить байт-в-байт двусторонний обмен с одним колесом.

### Тест 3.1 — сырые байты от STM32

На Jetson:

```bash
sudo apt install python3-serial
sudo chmod 666 /dev/ttyACM0     # временно, для теста

# Сырое чтение 50 пакетов:
python3 << 'PY'
import serial, struct

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)

# Читаем 5 байт = SendData
for i in range(50):
    data = ser.read(5)
    if len(data) != 5:
        print(f"  short read: {len(data)} bytes — устройство не отвечает")
        continue

    # struct.unpack('<fB', ...) — little-endian, float (4) + uint8 (1)
    speed, flag = struct.unpack('<fB', data)
    print(f"#{i:02d}: bytes={data.hex(' ')}  speed={speed:+.4f}  flag=0x{flag:02X}")

ser.close()
PY
```

**Что должно быть на выходе:**
```
#00: bytes=00 00 00 00 00  speed=+0.0000  flag=0x00     ← мотор стоит
#01: bytes=00 00 00 00 00  speed=+0.0000  flag=0x00
...
```

Если получаешь `flag=0x01` или другое ненулевое значение — это нормально,
если STM32 видит реальную проблему (например, нет тока — мотор отключен).

**Если приходят случайные байты (не нули):**
- Проверь baudrate (115200).
- Проверь, что не запущен другой процесс, читающий тот же порт
  (`fuser /dev/ttyACM0`).

### Тест 3.2 — отправка команды на STM32

```bash
python3 << 'PY'
import serial, struct, time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)

# Команда: speed=0.0 м/с, angle=90° (нейтраль)
# struct.pack('<fH', ...) — little-endian, float (4) + uint16 (2)
pkt = struct.pack('<fH', 0.0, 90)
print(f"TX: {pkt.hex(' ')}  ({len(pkt)} bytes)")
# Ожидаемо: 00 00 00 00 5a 00  ← float 0.0 + uint16 90

ser.write(pkt)
ser.flush()
time.sleep(0.5)

# Сейчас STM32 должен ответить — серво должен встать на 90°,
# мотор остаться остановленным.

# Команда: speed=0.5 м/с, angle=120°
pkt = struct.pack('<fH', 0.5, 120)
print(f"TX: {pkt.hex(' ')}")
# Ожидаемо: 00 00 00 3f 78 00  ← float 0.5 + uint16 120
ser.write(pkt)
ser.flush()
time.sleep(2.0)
# Колесо должно крутиться, серво — на 120°!

# Стоп
ser.write(struct.pack('<fH', 0.0, 90))
ser.flush()
ser.close()
PY
```

**Что проверить:**
- Серво физически поворачивается в указанные углы.
- Мотор начинает крутиться при `speed=0.5`.
- В прошивке (см. `main.c`):
  ```c
  Motor_velocity = cmd->speed;     // ← наша скорость попала сюда
  PWM_servo = cmd->angle;          // ← наш угол попал сюда
  ```

### Тест 3.3 — что происходит при ошибочной длине

Если послать НЕ 6 байт, прошивка ответит ASCII-сообщением (см. `USBRxHandler`):

```python
ser.write(b'\x01\x02\x03')   # 3 байта — неправильно
data = ser.read(28)
print(data)  # b'Error: Invalid packet size\r\n'
```

Это полезно для отладки — если рассинхрон по байтам, прошивка явно скажет.

### Сводная таблица байтов TX/RX

| Направление | Размер | Формат  | Поля              | Пример                              |
|-------------|--------|---------|-------------------|-------------------------------------|
| Jetson→STM32| 6 байт | `<fH`   | float+uint16      | `00 00 00 3f 78 00` (v=0.5, a=120) |
| STM32→Jetson| 5 байт | `<fB`   | float+uint8       | `cd cc 4c 3e 00`    (v=0.2, fl=0)  |

Little-endian (`<`) — STM32 ARM Cortex-M3 в режиме little-endian, и обе стороны
должны это явно указать, иначе байты переставятся.

---

## 4. Уровень 2: все 6 STM32

**Задача:** убедиться, что `serial_controller_node` корректно работает со
всеми 6 портами одновременно — каждый в своём потоке.

### Тест 4.1 — udev симлинки для стабильных имён

Без udev порядок `/dev/ttyACM0..5` зависит от порядка подключения. Ставим
правила: каждое колесо получает фиксированное имя по уникальному serial
прошивки.

```bash
# Сначала узнаём serial каждого STM32:
udevadm info /dev/ttyACM0 | grep ID_SERIAL_SHORT
# E: ID_SERIAL_SHORT=ABCD1234   ← это и есть serial WHEEL_1

# Создаём правило:
sudo nano /etc/udev/rules.d/99-rover-wheels.rules
```

Содержимое:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="ABCD1234", SYMLINK+="ttyROVER_WHEEL_1", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="EFGH5678", SYMLINK+="ttyROVER_WHEEL_2", MODE="0666"
# ... и так до WHEEL_6
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/ttyROVER_*
# /dev/ttyROVER_WHEEL_1 -> ttyACM2
# /dev/ttyROVER_WHEEL_2 -> ttyACM0
# ...
```

Соответствие имён — в `config/robot_params.yaml`:
- WHEEL_1 = front_right
- WHEEL_2 = middle_right
- WHEEL_3 = rear_right
- WHEEL_4 = front_left
- WHEEL_5 = middle_left
- WHEEL_6 = rear_left

### Тест 4.2 — запуск только `serial_controller_node`

```bash
ros2 run rover_nodes serial_controller_node
```

В логе должно появиться:
```
[INFO]: Port: front_left → /dev/ttyROVER_WHEEL_4
[INFO]: Port: middle_left → /dev/ttyROVER_WHEEL_5
[INFO]: Port: rear_left → /dev/ttyROVER_WHEEL_6
[INFO]: Port: front_right → /dev/ttyROVER_WHEEL_1
[INFO]: Port: middle_right → /dev/ttyROVER_WHEEL_2
[INFO]: Port: rear_right → /dev/ttyROVER_WHEEL_3
[INFO]: SerialControllerNode ready
[front_left] Connected: /dev/ttyROVER_WHEEL_4
[middle_left] Connected: /dev/ttyROVER_WHEEL_5
...
[INFO]: All 6 wheels OK | TX=0 RX=120
```

### Тест 4.3 — наблюдение за состоянием колёс

В другом терминале:

```bash
ros2 topic echo /wheels/state
```

Что увидишь:
```yaml
front_left:
  wheel_name: front_left
  speed: 0.0          ← с энкодера
  angle_cmd: 90.0     ← последний угол что мы послали
  flag: 0             ← АППАРАТНЫЙ ФЛАГ STM32
  is_connected: true
  stamp: {sec: 1234567890, nanosec: 0}
front_right:
  ...
connected_count: 6
```

Поле `flag` — это и есть `usb_stop_flag` из прошивки, прокинутый через USB CDC
прямо в ROS2 топик.

### Тест 4.4 — отправка команды одному колесу вручную

```bash
ros2 topic pub --once /wheel/front_left/cmd \
  rover_interfaces/msg/WheelCommand \
  "{wheel_name: 'front_left', speed_cmd: 0.3, angle_cmd: 100.0}"
```

Колесо `front_left` должно:
1. Серво повернуться на 100° (на 10° от нейтрали 90°).
2. Мотор закрутиться на 0.3 м/с.
3. В `/wheels/state` поле `front_left.speed` стать ≈ 0.3.

---

## 5. Уровень 3: вычисление кинематики

**Задача:** проверить математику Аккермана отдельно от железа.

### Тест 5.1 — запуск `ackermann_calculator_node`

```bash
ros2 run rover_nodes ackermann_calculator_node
```

### Тест 5.2 — прямолинейное движение

```bash
ros2 topic pub --once /motion_commands_safe \
  rover_interfaces/msg/MotionCommand \
  "{linear_velocity: 1.0, steering_angle: 0.0, source: 'test'}"
```

В другом терминале наблюдай за командами для всех колёс:
```bash
ros2 topic echo /wheel/front_left/cmd
# speed_cmd: 1.0, angle_cmd: 90.0  ← все 6 колёс должны иметь
                                     одинаковую скорость и нейтральный угол
```

### Тест 5.3 — поворот направо на 30°

```bash
ros2 topic pub --once /motion_commands_safe \
  rover_interfaces/msg/MotionCommand \
  "{linear_velocity: 1.0, steering_angle: 30.0, source: 'test'}"
```

Проверяем все 6 колёс:
```bash
for w in front_left middle_left rear_left front_right middle_right rear_right; do
  echo "=== $w ==="
  timeout 1 ros2 topic echo /wheel/$w/cmd --once 2>/dev/null | head -4
done
```

**Ожидаемое поведение:**
- При повороте вправо: внешние (левые) колёса крутятся БЫСТРЕЕ внутренних (правых).
- Передние/задние поворотные колёса имеют разные углы (Аккерман).
- `middle_left` и `middle_right` — `angle_cmd: 90.0` (всегда нейтраль).

---

## 6. Уровень 4: одометрия

**Задача:** убедиться, что одометрия правильно интегрирует движение.

### Тест 6.1 — запуск с реальными колёсами

```bash
ros2 launch rover_nodes robot_bringup.launch.py
```

В RViz2 (см. § 9 для запуска):
```bash
ros2 run rviz2 rviz2
# Add → TF, Odometry. Fixed Frame: odom
```

### Тест 6.2 — сброс в ноль

```bash
ros2 service call /reset_odometry rover_interfaces/srv/ResetOdometry \
  "{x: 0.0, y: 0.0, yaw_deg: 0.0}"
```

Сейчас робот стоит в начале координат, смотрит вдоль оси X.

### Тест 6.3 — наблюдение за `/odom`

```bash
ros2 topic echo /odom --field pose.pose.position
```

Если толкать робот руками — координаты должны меняться согласованно с
направлением. Если наоборот (двигаешь вперёд, x уменьшается) — поменяй знак
в `speed_conversion_factor` в YAML.

### Тест 6.4 — частота публикации

```bash
ros2 topic hz /odom
# average rate: 50.0 Hz
```

Должно быть ровно 50 Гц (см. `publish_rate: 50` в YAML).

---

## 7. Уровень 5: защита от STM32

**Задача:** проверить, что `flag_safety_node` правильно реагирует на `flag != 0`
от любого из 6 колёс.

### Тест 7.1 — нормальное состояние

```bash
ros2 topic echo /safety/active_flags
# data: 'OK'         ← все колёса без флагов
ros2 topic echo /safety/active
# data: false        ← защита не активна, команды проходят
```

### Тест 7.2 — симуляция флага без реального железа

Если все STM32 уже подключены, `flag` управляется прошивкой. Но для теста
самой логики `flag_safety_node` можно опубликовать `RoverWheelsState` вручную:

```bash
ros2 topic pub --once /wheels/state rover_interfaces/msg/RoverWheelsState \
"{
  front_left:   {wheel_name: 'front_left',   is_connected: true, flag: 1, speed: 0.0, angle_cmd: 90.0, stamp: {sec: 0, nanosec: 0}},
  middle_left:  {wheel_name: 'middle_left',  is_connected: true, flag: 0, speed: 0.0, angle_cmd: 90.0, stamp: {sec: 0, nanosec: 0}},
  rear_left:    {wheel_name: 'rear_left',    is_connected: true, flag: 0, speed: 0.0, angle_cmd: 90.0, stamp: {sec: 0, nanosec: 0}},
  front_right:  {wheel_name: 'front_right',  is_connected: true, flag: 0, speed: 0.0, angle_cmd: 90.0, stamp: {sec: 0, nanosec: 0}},
  middle_right: {wheel_name: 'middle_right', is_connected: true, flag: 0, speed: 0.0, angle_cmd: 90.0, stamp: {sec: 0, nanosec: 0}},
  rear_right:   {wheel_name: 'rear_right',   is_connected: true, flag: 0, speed: 0.0, angle_cmd: 90.0, stamp: {sec: 0, nanosec: 0}},
  connected_count: 6,
  stamp: {sec: 0, nanosec: 0}
}"
```

В логах `flag_safety_node`:
```
⚠ Hardware flags raised: front_left=0x01
```

И `/safety/active` стал `true`. Любые команды от контроллеров теперь
блокируются и заменяются нулями (`source='safety'`).

### Тест 7.3 — ручной блок

```bash
# Заблокировать
ros2 topic pub --once /safety/clear std_msgs/msg/Bool "{data: true}"
# В логе: "Manual safety block ENGAGED by operator"

# Снять (только если STM32 не держит флаг!)
ros2 topic pub --once /safety/clear std_msgs/msg/Bool "{data: false}"
# Логи: "Manual safety block RELEASED — system OK"
# или:  "Cannot release: hardware flags still raised: ['front_left']"
```

Это правильное поведение: ты не можешь "продавить" защиту, пока STM32 не
сбросил свой флаг — нижний уровень всегда главный.

---

## 8. Уровень 6: контроллеры

### Тест 8.1 — клавиатура

```bash
# В одном терминале:
ros2 launch rover_nodes robot_bringup.launch.py

# В другом:
ros2 launch rover_nodes keyboard_control.launch.py
```

Нажми `W` несколько раз — наблюдай в третьем терминале:
```bash
ros2 topic echo /motion_commands         # сырые команды клавиатуры
# linear_velocity: 0.6, steering_angle: 0.0, source: 'keyboard'

ros2 topic echo /motion_commands_safe    # после flag_safety
# то же самое (если защита не активна)
```

### Тест 8.2 — DualShock 4

```bash
# Подключи геймпад (USB или Bluetooth)
ls /dev/input/by-id | grep -i sony
# usb-Sony_Computer_Entertainment_Wireless_Controller-event-joystick

# Тест без ROS:
sudo apt install evtest
sudo evtest /dev/input/by-id/usb-Sony*
# Подвигай стиками — должны лететь события EV_ABS

# Запускаем:
ros2 launch rover_nodes dualshock4_control.launch.py
```

### Тест 8.3 — траектория по прямой

```bash
ros2 launch rover_nodes trajectory_control.launch.py

# В другом терминале:
ros2 service call /start_straight_trajectory \
  rover_interfaces/srv/StartStraightTrajectory \
  "{distance: 2.0, speed: 0.5}"

# Наблюдаем выполнение:
ros2 topic echo /trajectory/status
# state: 'running', traveled_distance: 0.4, progress_percent: 20.0
# ...
# state: 'finished', traveled_distance: 2.0, progress_percent: 100.0
```

Робот должен проехать 2 метра вперёд, замедляясь за 0.3 м до цели.

---

## 9. Уровень 7: PC ↔ Jetson

**Задача:** связать ROS2 узлы между двумя машинами.

### Тест 9.1 — настройка сети (один раз)

На обеих машинах:
```bash
cd robot_control_v3/ethernet
sudo ./setup_network.sh pc       # на PC
sudo ./setup_network.sh jetson   # на Jetson

source env_pc.sh                 # на PC
source env_jetson.sh             # на Jetson

./verify.sh                      # диагностика
```

См. полные детали в `ethernet/README.md`.

### Тест 9.2 — узлы видят друг друга

На Jetson запусти `robot_bringup.launch.py`. На PC:
```bash
ros2 node list
# /flag_safety_node              ← с Jetson!
# /serial_controller_node
# /odometry_node
# ...

ros2 topic list
# /odom
# /wheels/state
# /motion_commands
# ...

ros2 topic echo /rover/status
# Видим состояние робота с PC через Ethernet
```

### Тест 9.3 — задержка по сети

```bash
# На PC:
ros2 topic hz /odom            # должно быть ~50 Гц как на Jetson
ros2 topic delay /odom         # average delay: 1-3 ms по Ethernet
                                # 10-50 ms по WiFi
```

### Тест 9.4 — управление с PC

С PC можно публиковать команды напрямую:
```bash
ros2 topic pub /motion_commands rover_interfaces/msg/MotionCommand \
  "{linear_velocity: 0.3, steering_angle: 0.0, source: 'pc_test'}" \
  --rate 10
```

Команда летит:
```
PC operator (pub) → Ethernet (1ms) → Jetson flag_safety_node (filter)
  → Jetson ackermann_calculator_node (kinematics)
  → Jetson serial_controller_node (USB CDC)
  → 6 × STM32 (servos + motors)
```

### Тест 9.5 — RViz2 на PC

```bash
# На PC:
ros2 run rviz2 rviz2
```

В RViz:
- Add → TF
- Add → Odometry, topic `/odom`
- Fixed frame: `odom`

Двигай робот — видишь его перемещение в реальном времени по сети.

---

## 10. Полный сквозной тест

Проверка всей цепочки от клавиатуры на PC до моторов:

```bash
# ── На Jetson (терминал 1) ────────────────────────────────────────────
source ~/ros2_ws/install/setup.bash
source /etc/rover/env.sh
ros2 launch rover_nodes robot_bringup.launch.py

# ── На PC (терминал 1) — клавиатурный контроллер ─────────────────────
ssh rover@192.168.10.2
source ~/ros2_ws/install/setup.bash
source /etc/rover/env.sh
ros2 launch rover_nodes keyboard_control.launch.py
# (нажимаем W — едем вперёд)

# ── На PC (терминал 2) — мониторинг ───────────────────────────────────
source /etc/rover/env.sh
ros2 topic echo /rover/status

# ── На PC (терминал 3) — RViz ─────────────────────────────────────────
ros2 run rviz2 rviz2
```

Что проверить:
1. ✅ Нажатие W в SSH-сессии вызывает движение моторов
2. ✅ В RViz робот двигается вперёд
3. ✅ `/rover/status` показывает `state: driving`, `source: keyboard`
4. ✅ Если выдернуть USB одного STM32 — `connected_count: 5`,
      `safety_flags` показывает обрыв конкретного колеса
5. ✅ Нажатие X на клавиатуре блокирует всё (`safety_active: true`)
6. ✅ Нажатие R снимает блок

---

## 11. Шпаргалка по диагностике

### Колесо не крутится

```bash
# 1. Проверь подключение:
ls /dev/ttyROVER_WHEEL_*

# 2. Сырое чтение:
python3 -c "import serial,struct;s=serial.Serial('/dev/ttyROVER_WHEEL_1',115200,timeout=1);print(struct.unpack('<fB',s.read(5)))"

# 3. ROS topic:
ros2 topic echo /wheel/front_right/state
# Если is_connected: false → проблема в serial_controller_node
# Если flag != 0          → проблема на STM32 (защита)
# Если speed остаётся 0   → проблема с энкодером или мотором
```

### Узлы с PC не видят узлы с Jetson

```bash
# 1. ROS_DOMAIN_ID совпадает?
echo $ROS_DOMAIN_ID  # на обеих машинах должно быть одинаково

# 2. Multicast проходит?
sudo tcpdump -i eth0 -n 'udp port 7400'   # должен быть трафик

# 3. Firewall открыт?
sudo ufw status | grep 7400

# 4. ping работает?
ping 192.168.10.2

# 5. RMW одинаковый?
echo $RMW_IMPLEMENTATION  # rmw_fastrtps_cpp на обеих сторонах
```

### Высокая задержка одометрии

```bash
ros2 topic delay /odom
# > 50 мс? Возможно DDS использует WiFi вместо Ethernet.

# Решение: явно привязать FastDDS к eth0:
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/rover/fastdds_profile.xml
# (создаётся автоматически setup_network.sh)
```

### STM32 шлёт флаг != 0, но физических проблем нет

Проверь в прошивке `Rover/main.c`:
```c
if (usb_stop_flag) {
    Motor_velocity = 0;
    servo1.driver._duty = 0;
}
```
И где `usb_stop_flag` устанавливается. Возможные причины:
- Превышение тока (`fltCurFeedback > RatedCurrent`)
- Watchdog в самом STM32 (если реализован)
- Команда стоп пришла через USB и не сбросилась

Если флаг постоянно поднят без причины — это бага в прошивке, не в Jetson.

### Сводная таблица топиков и их частот

| Топик                       | Частота | Источник                  | Назначение            |
|-----------------------------|---------|----------------------------|-----------------------|
| `/motion_commands`          | 20 Гц   | controllers                | сырые команды         |
| `/motion_commands_safe`     | 20 Гц   | flag_safety_node           | после фильтра         |
| `/wheel/{name}/cmd`         | по запросу| ackermann_calculator_node| команды колёсам       |
| `/wheel/{name}/state`       | 20 Гц   | serial_controller_node     | состояние с STM32     |
| `/wheels/state`             | 20 Гц   | serial_controller_node     | агрегат всех колёс    |
| `/odom`                     | 50 Гц   | odometry_node              | позиция               |
| `/tf`                       | 50 Гц   | odometry_node              | трансформация         |
| `/safety/active`            | 20 Гц   | flag_safety_node           | защита активна?       |
| `/safety/active_flags`      | 20 Гц   | flag_safety_node           | какие флаги           |
| `/safety/clear`             | по событию | controllers             | ручной блок/снятие    |
| `/rover/status`             | 1 Гц    | rover_status_node          | агрегированный статус |
| `/trajectory/status`        | 20 Гц   | straight_trajectory_node   | прогресс траектории   |

---

**Итог:** если все 11 разделов проходят без ошибок — система работает.
Сохрани этот документ и используй как чек-лист при сборке нового робота.
