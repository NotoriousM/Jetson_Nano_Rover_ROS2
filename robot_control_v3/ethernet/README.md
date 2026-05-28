# ethernet/ — коммуникация PC ↔ Jetson Nano по сети

Эта папка содержит всё необходимое для настройки связи между Jetson Nano (бортовой
одноплатник робота) и PC (рабочее место оператора) по Ethernet и WiFi через ROS2.

## Содержимое

```
ethernet/
├── README.md                    # этот файл
├── netplan-pc.yaml              # статический IP для PC (eth0)
├── netplan-jetson.yaml          # статический IP для Jetson (eth0 + wlan0)
├── fastdds_profile.xml          # принудительная привязка DDS к интерфейсу
├── setup_network.sh             # автоматическая настройка (запускать на каждой машине)
├── env_pc.sh                    # ROS2 переменные окружения для PC
├── env_jetson.sh                # ROS2 переменные окружения для Jetson
├── ssh_robot.sh                 # удобная команда для SSH на Jetson по WiFi
└── verify.sh                    # диагностика: что видится в сети
```

## Архитектура связи

```
┌─────────────────────────┐                    ┌─────────────────────────┐
│ PC (оператор)           │                    │ Jetson Nano             │
│ Ubuntu 22.04            │   Ethernet 1 Gbit  │ Ubuntu 20.04            │
│ ROS2 Humble             │ ◄──────────────────►│ ROS2 Foxy               │
│ 192.168.10.1/24         │                    │ 192.168.10.2/24         │
│                         │                    │                         │
│ • RViz2                 │   WiFi (резерв)    │ • flag_safety_node      │
│ • monitor_node          │ ◄══════════════════►│ • ackermann_node        │
│ • operator_node         │  192.168.1.x       │ • serial_controller_node│
│                         │                    │ • odometry_node         │
└─────────────────────────┘                    │ • rover_status_node     │
                                                └────────────┬────────────┘
                                                             │ USB CDC
                                                  ┌──────────▼──────────┐
                                                  │  6 × STM32F103C8T6  │
                                                  └─────────────────────┘
```

**Два канала:**
- **Ethernet** (основной) — прямое соединение кабелем cat5e/cat6.
  Задержка ~0.1 мс, гарантированная пропускная способность 1 Гбит/с.
  Используется для всего: одометрия, видео, команды.

- **WiFi** (резервный) — через домашний роутер, для SSH и отладки в поле.
  Задержка ~5–50 мс. ROS2 топики тоже работают, но менее стабильно.

## Быстрый запуск

```bash
# На PC:
cd ethernet/
sudo ./setup_network.sh pc
source env_pc.sh

# На Jetson Nano (по SSH или с подключённой клавиатурой):
cd ethernet/
sudo ./setup_network.sh jetson
source env_jetson.sh

# Проверить связь с обеих сторон:
./verify.sh
```

## Принцип работы ROS2 в сети

ROS2 использует **DDS (Data Distribution Service)** в качестве транспорта.
DDS не требует центрального брокера (как ROS1 master), узлы находят друг друга
автоматически через **multicast discovery**.

Ключевые переменные окружения:
- `ROS_DOMAIN_ID=42` — изолирует группу узлов в сети.
  Узлы с разным DOMAIN_ID **не видят** друг друга.
  На PC и Jetson значение ДОЛЖНО совпадать.

- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` — конкретная реализация DDS
  (FastDDS от eProsima). Обе стороны должны использовать одинаковый RMW.

- `FASTRTPS_DEFAULT_PROFILES_FILE=/path/fastdds_profile.xml` — принудительно
  привязывает DDS к конкретному сетевому интерфейсу. Полезно, когда у Jetson
  активны и Ethernet, и WiFi одновременно — без этой переменной DDS может
  выбрать WiFi и работать медленно.

## SSH на робота по WiFi

```bash
# С PC, после настройки SSH-ключа:
./ssh_robot.sh                      # подключиться
./ssh_robot.sh "ros2 topic list"    # выполнить команду удалённо
```

## OSI по уровням для каждого канала

| Уровень | Ethernet PC↔Jetson      | WiFi PC↔Jetson         | USB CDC Jetson↔STM32 |
|---------|--------------------------|-------------------------|----------------------|
| L7 Application | ROS2 узлы         | ROS2 узлы              | —                    |
| L6 Presentation| CDR (rosidl)      | CDR (rosidl)           | struct.pack          |
| L5 Session     | RTPS Discovery    | RTPS Discovery         | —                    |
| L4 Transport   | UDP                | UDP                    | —                    |
| L3 Network     | IPv4               | IPv4                   | —                    |
| L2 Data Link   | Ethernet 802.3     | WiFi 802.11            | USB Frame + CRC      |
| L1 Physical    | RJ45 cat5e/6       | 2.4/5 ГГц радио        | USB D+/D− (NRZI)     |

`serial_controller_node` — единственная точка, где встречаются все 7 уровней:
данные приходят с верхов (L4–L7 ROS2 DDS) и опускаются в L1–L2 USB CDC.

## Диагностика

См. `verify.sh` для готового скрипта проверки. Полезные команды:

```bash
ping 192.168.10.2                    # связь по сети
ros2 node list                       # узлы видны (с обеих сторон)
ros2 topic list                      # все топики из сети
ros2 topic hz /odom                  # частота прихода одометрии (~50 Гц)
ros2 topic delay /odom               # задержка (Ethernet: <5 мс)
sudo tcpdump -i eth0 udp port 7400  # multicast discovery
sudo ufw status                      # firewall (порты 7400-7500/udp должны быть открыты)
```
