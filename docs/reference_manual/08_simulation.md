# Глава 8. Симуляция: RViz2 и Gazebo

[◀ Сеть](07_network_setup.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Траектории ▶](09_trajectories.md)

---

## Содержание

- [8.1 Пакет rover_description](#81-пакет-rover_description)
- [8.2 Структура URDF/Xacro](#82-структура-urdfxacro)
- [8.3 RViz2 — статика (display.launch.py)](#83-rviz2--статика-displaylaunchpy)
- [8.4 RViz2 — симуляция движения (sim.launch.py)](#84-rviz2--симуляция-движения-simlaunchpy)
- [8.5 ackermann_sim_node](#85-ackermann_sim_node)
- [8.6 Gazebo](#86-gazebo)
- [8.7 Отладка URDF](#87-отладка-urdf)
- [8.8 Добавление сенсоров](#88-добавление-сенсоров)

---

## 8.1 Пакет rover_description

**Самостоятельный, минималистичный пакет.** Содержит ТОЛЬКО описание робота.  
Никаких сенсоров, контроллеров, Gazebo-плагинов — всё добавляется поверх.

```
rover_description/
├── urdf/
│   ├── rover.urdf.xacro        ← основная модель
│   └── gazebo_plugins.xacro    ← плагины Gazebo (опционально)
├── meshes/
│   └── *.STL, *.dae            ← геометрия всех звеньев
├── launch/
│   ├── display.launch.py       ← статическая визуализация
│   ├── sim.launch.py           ← симуляция движения (RViz2)
│   ├── figure_eight.launch.py  ← восьмёрка в RViz2
│   ├── figure_lemniscate.launch.py ← лемниската в RViz2
│   └── gazebo_sim.launch.py    ← Gazebo
├── scripts/
│   ├── ackermann_sim_node.py       ← симулятор кинематики
│   ├── figure_lemniscate_rviz.py   ← генератор лемнискаты
│   └── robot_keyboard_controller.py
├── config/
│   └── controllers.yaml        ← конфиг ros2_control (для Gazebo)
└── rviz/
    ├── rover.rviz              ← статическая конфигурация
    └── rover_sim.rviz          ← конфигурация для симуляции
```

---

## 8.2 Структура URDF/Xacro

```
base_footprint (корень, fixed)
└── base_link (рама-корпус)
    ├── Differ (дифференциал)
    │   ├── Rocker_L (левый рокер, revolute ±25°)
    │   │   ├── Bogie_L (левый богги, revolute ±25°)
    │   │   │   ├── Rorate_L_REAR → WHEEL_L_REAR (continuous)
    │   │   │   └── Wheel_L_MIDDLE (continuous, нет серво)
    │   │   └── Tiaga_L
    │   │       └── Rotate__L_FRONT → Wheel_L_FRONT (continuous)
    │   └── Rocker_R (правый рокер, повёрнут на 180° в URDF)
    │       └── ... (зеркально)
    ├── Tiaga_L, Tiaga_R (тяги рулевого механизма)
    └── Differ_L, Differ_R
```

**Итого:** 19 links, 18 joints (4 рулевых revolute + 6 ведущих continuous + 8 подвески + 1 fixed)

---

## 8.3 RViz2 — статика (display.launch.py)

```bash
ros2 launch rover_description display.launch.py
```

Запускает:
- `robot_state_publisher` с моделью из xacro
- `joint_state_publisher_gui` — окно с ползунками для всех 14 подвижных суставов

Настройка RViz2 вручную (если конфиг не загрузился):
1. `Global Options` → `Fixed Frame` = `base_footprint`
2. `Add` → `RobotModel`
3. `Add` → `TF`

---

## 8.4 RViz2 — симуляция движения (sim.launch.py)

```bash
ros2 launch rover_description sim.launch.py
```

Запускает:
- `robot_state_publisher`
- `ackermann_sim_node` — симулирует физику движения
- `path_publisher` — рисует след в RViz2
- `robot_keyboard_controller` — управление с клавиатуры
- `rviz2` с конфигом `rover_sim.rviz`

Настройка RViz2:
1. `Fixed Frame` = `odom`
2. `Add` → `RobotModel` (Topic: `/robot_description`)
3. `Add` → `Path` → Topic: `/rover_path`
4. `Add` → `Odometry` → Topic: `/odom`

---

## 8.5 ackermann_sim_node

**Файл:** `scripts/ackermann_sim_node.py`

Симулирует движение ровера без реального железа. Принимает команды через `/motion_commands`,  
вычисляет кинематику и публикует `/joint_states`, `/odom`, TF.

```
/motion_commands [Float32MultiArray: speed, delta_deg]
        │
        ▼
  ackermann_sim_node
  (кинематика RK4, 50 Гц)
        │
        ├──► /joint_states  (углы и скорости 14 суставов)
        ├──► /odom          (позиция в odom)
        └──► /tf            (odom → base_footprint)
```

**Параметры:**
```yaml
a_distance:    0.4035
b_distance:    0.4035
track_width:   0.779
wheel_radius:  0.097
publish_rate:  50.0
```

---

## 8.6 Gazebo

```bash
# Для виртуальных машин (без аппаратного GPU)
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/install/rover_description/share

# Убить зависшие процессы перед запуском
pkill -9 -f gzserver; sleep 1

# Запуск
ros2 launch rover_description gazebo_sim.launch.py
```

### Ожидание запуска controller_manager

В Foxy `spawner.py` не поддерживает `--controller-manager-timeout`.  
Gazebo стартует медленно (~90-120 сек), поэтому в launch используется bash-цикл:

```python
# В gazebo_sim.launch.py:
from launch.actions import ExecuteProcess

wait_and_spawn = ExecuteProcess(cmd=[
    'bash', '-c',
    'until ros2 service list | grep list_controllers; do sleep 2; done; '
    'ros2 run controller_manager spawner joint_state_broadcaster'
])
```

---

## 8.7 Отладка URDF

```bash
# Сгенерировать URDF из xacro
xacro ~/ros2_ws/install/rover_description/share/rover_description/urdf/rover.urdf.xacro \
  > /tmp/rover.urdf

# Проверить синтаксис
check_urdf /tmp/rover.urdf
# ✓ Нет ошибок — показывает дерево всех links

# Просмотреть TF-дерево в реальном времени
ros2 run tf2_tools view_frames
# Генерирует frames.pdf

# Список суставов в /joint_states
ros2 topic echo /joint_states --once | grep name
```

---

## 8.8 Добавление сенсоров

Пакет намеренно минималистичен. Добавление сенсоров — создать новый xacro файл:

```xml
<!-- sensors.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- LiDAR на кузове -->
  <link name="lidar_link">
    <visual><geometry><cylinder radius="0.05" length="0.07"/></geometry></visual>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.15 0 0.1"/>
  </joint>

  <!-- Плагин Gazebo для LiDAR -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar">...</sensor>
  </gazebo>

</robot>
```

Подключить в `rover.urdf.xacro`:
```xml
<xacro:include filename="$(find rover_description)/urdf/sensors.xacro"/>
```

---

[◀ Сеть](07_network_setup.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Траектории ▶](09_trajectories.md)
