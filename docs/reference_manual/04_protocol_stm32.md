# Глава 4. Протокол USB CDC — STM32 ↔ Jetson

[◀ Ноды](03_ros2_nodes.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Кинематика ▶](05_ackermann_kinematics.md)

---

## Содержание

- [4.1 Физический уровень](#41-физический-уровень)
- [4.2 Структуры TX и RX](#42-структуры-tx-и-rx)
- [4.3 C-код в прошивке STM32](#43-c-код-в-прошивке-stm32)
- [4.4 Python-реализация](#44-python-реализация)
- [4.5 Временна́я диаграмма](#45-временна́я-диаграмма)
- [4.6 usb_stop_flag — детально](#46-usb_stop_flag--детально)
- [4.7 Диагностика без ROS2](#47-диагностика-без-ros2)
- [4.8 Типичные ошибки](#48-типичные-ошибки)

---

## 4.1 Физический уровень

| Параметр | Значение |
|----------|---------|
| Интерфейс | USB Communication Device Class (CDC) |
| OS-драйвер | `cdc_acm` (встроен в Linux, не нужно устанавливать) |
| Имя устройства | `/dev/ttyACM{n}` → через udev: `/dev/ttyROVER_WHEEL_{n}` |
| Baudrate | 115200 (USB CDC игнорирует baudrate, ставим для совместимости) |
| Порядок байт | Little-Endian |
| Выравнивание | Нет padding (`#pragma pack(1)`) |
| Заголовки фреймов | Нет — фиксированная длина пакетов |
| CRC | Нет |
| Кол-во устройств | 6 STM32, каждый на отдельном порту |

---

## 4.2 Структуры TX и RX

### TX: Jetson → STM32 (6 байт)

```
Offset  Размер  Тип       Поле     Описание
0       4       float32   speed    Целевая скорость колеса (м/с)
                                   + вперёд, − назад, диапазон [−7.0, 7.0]
4       2       uint16    angle    Угол сервопривода (целые градусы)
                                   0 = крайнее левое, 90 = нейтраль, 180 = крайнее правое
```

`sizeof = 6` = `struct.calcsize('<fH')`

### RX: STM32 → Jetson (5 байт)

```
Offset  Размер  Тип       Поле     Описание
0       4       float32   speed    Измеренная скорость (encoder_getVelocity, м/с)
4       1       uint8     flag     Состояние: 0=OK, 1+=аварийный стоп
```

`sizeof = 5` = `struct.calcsize('<fB')`

---

## 4.3 C-код в прошивке STM32

```c
/* main.c — прошивка Rover */

/* КРИТИЧНО: #pragma pack(1) — без него компилятор добавит байты padding! */

#pragma pack(push, 1)
typedef struct {
    float    speed;    /* 4 байта — целевая скорость (м/с) */
    uint16_t angle;    /* 2 байта — угол серво 0..180° */
} WheelData;           /* sizeof = 6 */
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    float   speed;     /* 4 байта — encoder_getVelocity() */
    uint8_t flag;      /* 1 байт  — usb_stop_flag */
} SendData;            /* sizeof = 5 */
#pragma pack(pop)

/* Приём команды: */
WheelData cmd;
uint8_t buf[sizeof(WheelData)];  /* = 6 байт */
CDC_Receive_FS(buf, sizeof(buf));
memcpy(&cmd, buf, sizeof(cmd));

/* Отправка состояния: */
SendData state;
state.speed = encoder_getVelocity();
state.flag  = usb_stop_flag;
CDC_Transmit_FS((uint8_t*)&state, sizeof(state));
```

---

## 4.4 Python-реализация

```python
import struct

# Форматы (L = Little-Endian)
TX_FORMAT = '<fH'    # float32 + uint16
RX_FORMAT = '<fB'    # float32 + uint8

TX_SIZE = struct.calcsize(TX_FORMAT)  # 6 байт
RX_SIZE = struct.calcsize(RX_FORMAT)  # 5 байт

# ── Отправка команды ──────────────────────────────────────────────────
def send_command(ser, speed: float, angle: int) -> None:
    """Упаковать и отправить 6 байт в STM32."""
    pkt = struct.pack(TX_FORMAT, float(speed), int(angle))
    # pkt = b'\x00\x00\x00\x00\x5a\x00' для (0.0, 90)
    ser.write(pkt)
    ser.flush()

# ── Приём и парсинг ───────────────────────────────────────────────────
rxbuf = bytearray()

def receive(ser):
    """Прочитать и разобрать накопленные пакеты из STM32."""
    global rxbuf
    if ser.in_waiting > 0:
        rxbuf.extend(ser.read(ser.in_waiting))
    
    results = []
    while len(rxbuf) >= RX_SIZE:
        pkt = bytes(rxbuf[:RX_SIZE])
        rxbuf = rxbuf[RX_SIZE:]
        speed, flag = struct.unpack(RX_FORMAT, pkt)
        results.append({'speed': float(speed), 'flag': int(flag)})
    return results

# ── Пример полного цикла ─────────────────────────────────────────────
import serial

ser = serial.Serial('/dev/ttyROVER_WHEEL_1', 115200, timeout=0.1)

# Отправить команду: 0.5 м/с, серво 90°
send_command(ser, speed=0.5, angle=90)

# Прочитать ответ
import time; time.sleep(0.05)
for state in receive(ser):
    print(f"speed={state['speed']:.3f}  flag=0x{state['flag']:02X}")
```

---

## 4.5 Временна́я диаграмма

```
Jetson                      STM32
  │                           │
  │── pack('<fH', 0.5, 90) ──►│  [0x00, 0x00, 0x00, 0x3F, 0x5A, 0x00]
  │                           │  ← WheelData.speed = 0.5, angle = 90
  │                           │
  │◄── pack('<fB', v, flag) ──│  [0x00, 0x00, 0x20, 0x3F, 0x00]
  │                           │  ← SendData.speed = 0.625, flag = 0
  │                           │
  │  ... повторяется ~20 Гц ..│
```

---

## 4.6 usb_stop_flag — детально

STM32 выставляет `flag ≠ 0` при:

| Бит | Значение | Причина |
|-----|---------|---------|
| `0x01` | Превышение тока | ACS712 сигнализирует перегрузку |
| `0x02` | Сбой энкодера | Потеря сигнала энкодера |
| `0x04` | Стоп оператора | Физическая кнопка STOP |
| `0x08..` | Зависит от прошивки | Другие аппаратные события |

**Обработка в ROS2:**
```
STM32 flag ≠ 0
  → WheelState.flag ≠ 0
  → flag_safety_node обнаруживает
  → /safety/active = True
  → /motion_commands_safe = MotionCommand(0.0, 0.0)
  → все колёса получают нулевую команду
```

Снятие блока (если STM32 сбросил флаг сам):
```bash
# Автоматически (если auto_clear_when_flag_resets=true в params)

# Вручную через клавиатуру: клавиша R
# Вручную через топик:
ros2 topic pub --once /safety/clear std_msgs/msg/Bool '{data: false}'
```

---

## 4.7 Диагностика без ROS2

```bash
# Проверить что STM32 видится в системе
ls /dev/ttyACM* /dev/ttyROVER_WHEEL_* 2>/dev/null
dmesg | grep cdc_acm | tail -10

# Hex-дамп сырых байт (что присылает STM32)
cat /dev/ttyROVER_WHEEL_1 | xxd | head -5

# Полный тест одного колеса через Python
python3 << 'PYEOF'
import struct, serial, time

ser = serial.Serial('/dev/ttyROVER_WHEEL_1', 115200, timeout=0.2)
print(f"Открыт: {ser.name}")

# Init-пакет: стоп
ser.write(struct.pack('<fH', 0.0, 90))

time.sleep(0.1)
data = ser.read(5)
if len(data) == 5:
    speed, flag = struct.unpack('<fB', data)
    print(f"Ответ: speed={speed:.3f} м/с, flag=0x{flag:02X}")
else:
    print(f"Получено {len(data)} байт (ожидалось 5)")

ser.close()
PYEOF
```

---

## 4.8 Типичные ошибки

| Симптом | Причина | Решение |
|---------|---------|---------|
| `/dev/ttyACM*` не появляется | Кабель charging-only | Заменить на data-кабель |
| RX: всегда 0 байт | STM32 не прошит / нет USB CDC | Перепрошить, добавить `MX_USB_DEVICE_Init()` |
| Неверные значения speed | Нет `#pragma pack(1)` в прошивке | Добавить pack-директивы в `main.c` |
| `Permission denied` | Нет прав на порт | `sudo usermod -a -G dialout $USER` |
| Порт меняется после reboot | Нет udev-правил | [udev_setup.md](udev_setup.md) |
| Задержка ~100мс | `timeout=0.1` в `serial.Serial` | Нормально, можно снизить до 0.01 |

---

[◀ Ноды](03_ros2_nodes.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Кинематика ▶](05_ackermann_kinematics.md)
