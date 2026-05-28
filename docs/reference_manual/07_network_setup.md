# Глава 7. Сеть PC ↔ Jetson Nano

[◀ Одометрия](06_odometry.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Симуляция ▶](08_simulation.md)

---

## Содержание

- [7.1 Топология](#71-топология)
- [7.2 Ethernet (рекомендуется)](#72-ethernet-рекомендуется)
- [7.3 FastDDS профиль](#73-fastdds-профиль)
- [7.4 Готовые скрипты (ethernet/)](#74-готовые-скрипты-ethernet)
- [7.5 WiFi](#75-wifi)
- [7.6 SSH доступ](#76-ssh-доступ)
- [7.7 Проверка связи](#77-проверка-связи)
- [7.8 Решение проблем](#78-решение-проблем)

---

## 7.1 Топология

```
┌──────────────────────────────────────────────────────────────────┐
│  PC-оператор (192.168.10.1)    Jetson Nano (192.168.10.2)        │
│                                                                   │
│  ros2 topic echo /odom        ros2 launch rover_nodes bringup    │
│  ros2 launch ... keyboard     serial_controller_node             │
│                                                                   │
│  ◄──────── Ethernet 1 Гбит Cat5e / Cat6 ────────►               │
│            ROS_DOMAIN_ID = 42                                     │
│            FastDDS UDP 7400-7500                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 7.2 Ethernet (рекомендуется)

### На Jetson Nano

```bash
# Вариант 1 — через netplan
sudo cp robot_control_v3/ethernet/netplan-jetson.yaml /etc/netplan/99-rover.yaml
sudo netplan apply

# Вариант 2 — временно (сбрасывается при reboot)
sudo ip addr add 192.168.10.2/24 dev eth0
sudo ip link set eth0 up

# Проверить
ip addr show eth0
```

### На PC

```bash
sudo cp robot_control_v3/ethernet/netplan-pc.yaml /etc/netplan/99-rover.yaml
sudo netplan apply

# или временно
sudo ip addr add 192.168.10.1/24 dev eth0
```

---

## 7.3 FastDDS профиль

Файл `ethernet/fastdds_profile.xml` настроен для прямого unicast (без multicast):

```xml
<!-- Упрощённый фрагмент fastdds_profile.xml -->
<profiles>
  <participant profile_name="default_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator><udpv4><address>192.168.10.2</address></udpv4></locator>
          <locator><udpv4><address>192.168.10.1</address></udpv4></locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator><udpv4><address>192.168.10.1</address><port>7410</port></udpv4></locator>
          <locator><udpv4><address>192.168.10.2</address><port>7410</port></udpv4></locator>
        </initialPeersList>
        <use_WriterLivelinessProtocol>false</use_WriterLivelinessProtocol>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

Применить:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(realpath robot_control_v3/ethernet/fastdds_profile.xml)
```

---

## 7.4 Готовые скрипты (ethernet/)

| Файл | Назначение |
|------|-----------|
| `env_jetson.sh` | Установить переменные окружения на Jetson |
| `env_pc.sh` | Установить переменные окружения на PC |
| `netplan-jetson.yaml` | Конфиг сети для Jetson |
| `netplan-pc.yaml` | Конфиг сети для PC |
| `setup_network.sh` | Настройка одной командой |
| `ssh_robot.sh` | SSH в Jetson (192.168.10.2) |
| `verify.sh` | Проверка связи ROS2 |
| `fastdds_profile.xml` | Профиль FastDDS |

```bash
# Использование:
# На Jetson
cd robot_control_v3/ethernet
source env_jetson.sh
ros2 launch rover_nodes robot_bringup.launch.py

# На PC
source robot_control_v3/ethernet/env_pc.sh
ros2 topic list           # видим топики с Jetson
```

---

## 7.5 WiFi

```bash
# На обоих устройствах
export ROS_DOMAIN_ID=42

# Если multicast блокируется (корпоративная сеть / роутер):
export FASTRTPS_DEFAULT_PROFILES_FILE=.../fastdds_profile.xml

# Убедиться что оба в одной подсети
ip addr show wlan0
```

---

## 7.6 SSH доступ

```bash
# Простое подключение
ssh jetson@192.168.10.2

# С X11 forwarding (для GUI: RViz2, Gazebo на Jetson)
ssh -X jetson@192.168.10.2

# Скрипт из репозитория
bash robot_control_v3/ethernet/ssh_robot.sh

# SCP — копировать файл на Jetson
scp my_file.py jetson@192.168.10.2:~/ros2_ws/src/
```

---

## 7.7 Проверка связи

```bash
# 1. Физический уровень
ping 192.168.10.2     # с PC → Jetson
ping 192.168.10.1     # с Jetson → PC

# 2. ROS2 уровень — скрипт из репозитория
bash robot_control_v3/ethernet/verify.sh

# 3. Вручную
#    Jetson: запустить ноду
ros2 run demo_nodes_cpp talker

#    PC: подписаться
ros2 topic echo /chatter
# Должны появляться "Hello World: N" каждую секунду
```

---

## 7.8 Решение проблем

| Проблема | Диагностика | Решение |
|---------|-------------|---------|
| `ros2 topic list` пустой с PC | Разные `ROS_DOMAIN_ID` | `export ROS_DOMAIN_ID=42` на обоих |
| Топики видны но сообщения не идут | Firewall / multicast | Применить `fastdds_profile.xml` |
| SSH отказывает | Нет сети | `ping 192.168.10.2` → проверить IP |
| Высокая латентность | WiFi помехи | Использовать Ethernet |
| `source env_jetson.sh` не меняет DOMAIN | Файл не sourced | `source`, не `bash ./env_jetson.sh` |

---

[◀ Одометрия](06_odometry.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Симуляция ▶](08_simulation.md)
