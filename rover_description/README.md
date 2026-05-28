# rover_description

URDF/Xacro модель шестиколёсного ровера с rocker-bogie подвеской.

**Самостоятельный пакет** — содержит ТОЛЬКО описание робота. Никаких сенсоров,
никаких Gazebo-плагинов, никаких контроллеров. Их можно добавить отдельно
поверх этой модели.

## Геометрия

Точно соответствует `robot_params.yaml` из `robot_control_v3`:

| Параметр              | Значение  | Источник                                |
|-----------------------|-----------|------------------------------------------|
| `wheelbase`           | 0.807 м   | `robot_params.yaml`                      |
| `track_width`         | 0.779 м   | `robot_params.yaml`                      |
| `a_distance`          | 0.4035 м  | `robot_params.yaml`                      |
| `b_distance`          | 0.4035 м  | `robot_params.yaml`                      |
| `wheel_radius`        | 0.10 м    | требование (диаметр 0.2 м)              |
| `body_length`         | 0.50 м    | пропорционально фото                     |
| `body_width`          | 0.40 м    | пропорционально фото                     |
| `body_height`         | 0.15 м    | пропорционально фото                     |
| `ground_clearance`    | 0.20 м    | пропорционально фото                     |
| `max_steering_angle`  | ±35°      | `robot_params.yaml` (0.610 рад)          |
| `rocker_limit`        | ±25°      | подвес rocker-bogie (0.436 рад)          |

## Архитектура суставов (joints)

```
base_footprint
└─ base_link  (рама-корпус)
   ├─ rocker_left_joint        revolute  Y-axis  ±25°  ← подвижный rocker левый
   │  └─ rocker_left_link
   │     ├─ steer_FL_joint     revolute  Z-axis  ±35°  ← серво front_left
   │     │  └─ steer_FL_link
   │     │     └─ drive_FL_joint  continuous  Y-axis  ← мотор front_left
   │     │        └─ wheel_FL_link
   │     └─ drive_ML_joint     continuous  Y-axis     ← мотор middle_left
   │        └─ wheel_ML_link               (без серво — неповоротное)
   │
   ├─ rocker_right_joint       revolute  Y-axis  ±25°  ← rocker правый
   │  └─ ...                                              (зеркально)
   │
   ├─ bogie_left_joint         revolute  Y-axis  ±25°  ← bogie левый (задний)
   │  └─ bogie_left_link
   │     └─ steer_RL_joint     revolute  Z-axis  ±35°  ← серво rear_left
   │        └─ steer_RL_link
   │           └─ drive_RL_joint  continuous  Y-axis    ← мотор rear_left
   │              └─ wheel_RL_link
   │
   └─ bogie_right_joint        revolute  Y-axis  ±25°  ← bogie правый
      └─ ...                                              (зеркально)
```

**Итого в модели:**
- 16 links (1 footprint + 1 base + 4 rocker/bogie + 4 steer + 6 wheels)
- 15 joints:
  - 1 fixed (footprint→base_link)
  - 4 revolute для подвески (rocker_left/right, bogie_left/right)
  - 4 revolute для серво (steer_FL, FR, RL, RR) — диапазон ±35°
  - 6 continuous для приводов колёс (drive_FL, FR, ML, MR, RL, RR)

## Сборка

```bash
mkdir -p ~/ros2_ws/src
cp -r rover_description ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select rover_description
source install/setup.bash
```

## Запуск 1 — Визуализация в RViz с ползунками

```bash
ros2 launch rover_description display.launch.py
```

В окне `joint_state_publisher_gui` появятся ползунки для всех 14 подвижных
суставов: 4 серво, 6 моторов, 4 рычага подвески. Двигай их — модель
обновляется в RViz.

## Запуск 2 — Только publisher для rosbridge / Foxglove

```bash
# Терминал 1: запустить robot_state_publisher
ros2 launch rover_description robot_description.launch.py

# Терминал 2: rosbridge для веб-доступа
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

После этого в браузере / Foxglove Studio:
- URL: `ws://<host-ip>:9090`
- Topic: `/robot_description` (URDF как строка)
- Можно визуализировать через 3D-плагин Foxglove

## Интеграция с robot_control_v3

Чтобы /tf обновлялся реальным движением робота, нужен узел, который
читает данные с STM32 и публикует `/joint_states`. Это узел добавляется
**отдельно** (не в этот пакет) — например, расширением `serial_controller_node`.

Минимальный пример такого моста (псевдокод):

```python
# В serial_controller_node._publish() добавить:
from sensor_msgs.msg import JointState

self._js_pub = self.create_publisher(JointState, '/joint_states', 10)

# В таймере:
js = JointState()
js.header.stamp = self.get_clock().now().to_msg()
js.name = [
    'drive_FL_joint', 'drive_FR_joint',
    'drive_ML_joint', 'drive_MR_joint',
    'drive_RL_joint', 'drive_RR_joint',
    'steer_FL_joint', 'steer_FR_joint',
    'steer_RL_joint', 'steer_RR_joint',
    # rocker/bogie joints публиковать как 0.0 если нет сенсора
    'rocker_left_joint', 'rocker_right_joint',
    'bogie_left_joint',  'bogie_right_joint',
]
js.position = [
    self._wheel_pos_FL, ..., self._steer_FL_rad, ..., 0.0, 0.0, 0.0, 0.0
]
js.velocity = [self._wheel_vel_FL, ...]
self._js_pub.publish(js)
```

## Что НЕ входит в пакет

Пакет специально минималистичный. Если нужно — добавь отдельно:

- **Сенсоры** (LiDAR, IMU, камера) — добавить в новый xacro-файл и `<xacro:include>`
- **Gazebo плагины** (`<gazebo>` теги) — отдельный xacro `rover.gazebo.xacro`
- **ros2_control** — отдельный xacro с `<ros2_control>` блоками
- **Меши** (.dae/.stl) — заменить `<box>`/`<cylinder>` на `<mesh>` в visual

Все эти расширения легко добавляются поверх готовой модели.

## Проверка

```bash
# Сгенерировать URDF из xacro:
xacro install/rover_description/share/rover_description/urdf/rover.urdf.xacro \
  > /tmp/rover.urdf

# Проверить корректность:
check_urdf /tmp/rover.urdf

# Должно вывести дерево всех 16 links без ошибок.
```
