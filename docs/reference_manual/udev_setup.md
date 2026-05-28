# udev — Стабильные симлинки для STM32

[◀ На главную](../../README.md) · [Содержание](../../README.md#12-документация)

---

## Проблема

Без udev-правил при каждой перезагрузке порты STM32 меняются:
```
# Вчера:
/dev/ttyACM0 → STM32 front_right
/dev/ttyACM1 → STM32 middle_right

# Сегодня (другой порядок USB enum):
/dev/ttyACM0 → STM32 rear_left
/dev/ttyACM1 → STM32 front_right
```

Это приводит к тому, что `serial_controller_node` управляет не теми колёсами.

---

## Решение

Создать udev-правила, которые назначают стабильные симлинки по серийному номеру устройства:
```
/dev/ttyROVER_WHEEL_1 → всегда STM32 front_right
/dev/ttyROVER_WHEEL_2 → всегда STM32 middle_right
...
```

---

## Шаг 1 — Найти серийные номера всех STM32

Подключайте STM32 **по одному**, записывая серийный номер каждого.

```bash
# После подключения одного STM32:
ls /dev/ttyACM*
# /dev/ttyACM0

# Найти серийный номер:
udevadm info -a -n /dev/ttyACM0 | grep -E 'ATTRS{serial}|ATTRS{idVendor}|ATTRS{idProduct}'

# Пример вывода:
#   ATTRS{idVendor}=="0483"        ← STMicroelectronics VID
#   ATTRS{idProduct}=="5740"       ← USB CDC VCP
#   ATTRS{serial}=="3276356C3134"  ← УНИКАЛЬНЫЙ серийник этого STM32
```

Запишите таблицу:

| Колесо | Серийный номер | Симлинк |
|--------|--------------|---------|
| front_right | `ваш_серийник_1` | `ttyROVER_WHEEL_1` |
| middle_right | `ваш_серийник_2` | `ttyROVER_WHEEL_2` |
| rear_right | `ваш_серийник_3` | `ttyROVER_WHEEL_3` |
| front_left | `ваш_серийник_4` | `ttyROVER_WHEEL_4` |
| middle_left | `ваш_серийник_5` | `ttyROVER_WHEEL_5` |
| rear_left | `ваш_серийник_6` | `ttyROVER_WHEEL_6` |

---

## Шаг 2 — Создать файл правил

```bash
sudo nano /etc/udev/rules.d/99-rover-wheels.rules
```

Содержимое (замените серийные номера на ваши реальные):

```udev
# Rover 6-wheel STM32 USB CDC — стабильные симлинки
# Найти серийники: udevadm info -a -n /dev/ttyACM0 | grep serial
#
# idVendor=0483 (STMicroelectronics), idProduct=5740 (USB CDC VCP)

SUBSYSTEM=="tty", \
  ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="ЗАМЕНИТЕ_СЕРИЙНИК_1", \
  SYMLINK+="ttyROVER_WHEEL_1", \
  MODE="0666"

SUBSYSTEM=="tty", \
  ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="ЗАМЕНИТЕ_СЕРИЙНИК_2", \
  SYMLINK+="ttyROVER_WHEEL_2", \
  MODE="0666"

SUBSYSTEM=="tty", \
  ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="ЗАМЕНИТЕ_СЕРИЙНИК_3", \
  SYMLINK+="ttyROVER_WHEEL_3", \
  MODE="0666"

SUBSYSTEM=="tty", \
  ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="ЗАМЕНИТЕ_СЕРИЙНИК_4", \
  SYMLINK+="ttyROVER_WHEEL_4", \
  MODE="0666"

SUBSYSTEM=="tty", \
  ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="ЗАМЕНИТЕ_СЕРИЙНИК_5", \
  SYMLINK+="ttyROVER_WHEEL_5", \
  MODE="0666"

SUBSYSTEM=="tty", \
  ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="ЗАМЕНИТЕ_СЕРИЙНИК_6", \
  SYMLINK+="ttyROVER_WHEEL_6", \
  MODE="0666"
```

---

## Шаг 3 — Применить правила

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger

# Переподключить все USB (или перезагрузиться)
```

---

## Шаг 4 — Проверить

```bash
ls -la /dev/ttyROVER_WHEEL_*
# lrwxrwxrwx 1 root root 7 ... /dev/ttyROVER_WHEEL_1 -> ttyACM2
# lrwxrwxrwx 1 root root 7 ... /dev/ttyROVER_WHEEL_2 -> ttyACM0
# lrwxrwxrwx 1 root root 7 ... /dev/ttyROVER_WHEEL_3 -> ttyACM4
# ...

# Проверить что порт открывается
python3 -c "
import serial
for i in range(1, 7):
    try:
        s = serial.Serial(f'/dev/ttyROVER_WHEEL_{i}', 115200, timeout=0.1)
        print(f'WHEEL_{i}: OK ({s.name})')
        s.close()
    except Exception as e:
        print(f'WHEEL_{i}: ОШИБКА — {e}')
"
```

---

## Шаг 5 — Права на порты

```bash
# Добавить пользователя в группу dialout
sudo usermod -a -G dialout $USER

# ВАЖНО: перелогиниться или выполнить:
newgrp dialout

# Проверить права
id $USER | grep dialout
# ... groups=20(dialout) ...
```

---

## Диагностика

```bash
# Смотреть события udev в реальном времени при подключении
udevadm monitor --environment --udev | grep tty

# Тест правила для конкретного устройства
udevadm test /sys/class/tty/ttyACM0

# Полная информация об устройстве
udevadm info -a -n /dev/ttyACM0
```

---

[◀ На главную](../../README.md) · [Содержание](../../README.md#12-документация)
