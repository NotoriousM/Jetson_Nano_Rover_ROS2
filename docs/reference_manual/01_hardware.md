# Глава 1. Аппаратная часть

[◀ На главную](../../README.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Установка ▶](02_installation.md)

---

## Содержание

- [1.1 Компоненты](#11-компоненты)
- [1.2 Схема USB-подключения](#12-схема-usb-подключения)
- [1.3 udev — стабильные симлинки](#13-udev--стабильные-симлинки)
- [1.4 Питание](#14-питание)
- [1.5 Геометрические параметры](#15-геометрические-параметры)
- [1.6 Конструкция подвески rocker-bogie](#16-конструкция-подвески-rocker-bogie)

---

## 1.1 Компоненты

| Компонент | Кол-во | Спецификация | Роль в системе |
|-----------|:------:|-------------|---------------|
| **NVIDIA Jetson Nano B01** | 1 | 4×ARM A57 @1.43GHz, 4GB LPDDR4 | Основной компьютер, ROS2 |
| **STM32F103C8T6** (BluePill) | 6 | 72MHz, USB CDC | Контроллер одного колеса |
| **DC мотор + энкодер** | 6 | 12V, PWM управление | Приводные колёса |
| **Сервопривод** | 4 | 5V, 180° | Рулевые: FL, FR, RL, RR |
| **Датчик тока** | 6 | ACS712 | Защита моторов → `usb_stop_flag` |
| **USB 3.0 хаб** | 1 | 7-port, внешнее питание | 6 STM32 → Jetson |
| **LiPo аккумулятор** | 1 | 3S 11.1V, 5000мАч | Основное питание |

---

## 1.2 Схема USB-подключения

```
┌─────────────────────────────────────────────────────────┐
│                    Jetson Nano B01                        │
│  USB 3.0 ────────────────────────────────────────────    │
└────────────────────────┬────────────────────────────────┘
                         │
                    USB 3.0 Hub (7 port)
         ┌───────────────┼───────────────┐
         │               │               │
   STM32            STM32           STM32
   front_right      middle_right    rear_right
   /dev/ttyACM0     /dev/ttyACM1    /dev/ttyACM2
         │               │               │
   symlink             symlink         symlink
   ttyROVER_WHEEL_1   _WHEEL_2        _WHEEL_3
         │               │               │
   STM32            STM32           STM32
   front_left       middle_left     rear_left
   /dev/ttyACM3     /dev/ttyACM4    /dev/ttyACM5
   ttyROVER_WHEEL_4  _WHEEL_5        _WHEEL_6
```

> **Важно:** Без udev-правил номера `/dev/ttyACM*` меняются при каждой перезагрузке.  
> Симлинки `ttyROVER_WHEEL_{1-6}` остаются постоянными.  
> Настройка → [udev_setup.md](udev_setup.md)

---

## 1.3 udev — стабильные симлинки

Пошаговая инструкция → [udev_setup.md](udev_setup.md)

Краткий рецепт:

```bash
# 1. Найти серийный номер каждого STM32
udevadm info -a -n /dev/ttyACM0 | grep -E 'serial|idVendor|idProduct'

# 2. Создать правила
sudo nano /etc/udev/rules.d/99-rover-wheels.rules

# 3. Применить
sudo udevadm control --reload-rules && sudo udevadm trigger

# 4. Проверить
ls -la /dev/ttyROVER_WHEEL_*
```

---

## 1.4 Питание

```
LiPo 3S (11.1V 5Ah)
    │
    ├── DC-DC 5V 4A ──────────► Jetson Nano (5V/4A barrel)
    │
    ├── DC-DC 12V ────────────► 6× STM32 + H-bridge + моторы
    │
    └── Servo Rail 5V ────────► 4× сервопривода (FL, FR, RL, RR)
```

> **Предупреждение:** Jetson Nano требует стабильный источник 5V/4A.  
> Питание от USB-хаба или маломощного адаптера приведёт к нестабильной работе.

---

## 1.5 Геометрические параметры

Все значения соответствуют `robot_params.yaml`:

| Параметр | YAML-ключ | Значение | Описание |
|----------|-----------|---------|----------|
| Колёсная база | `wheelbase` | **0.807 м** | Расстояние FL–RL |
| Ширина колеи | `track_width` | **0.779 м** | Расстояние L–R |
| Передняя полубаза | `a_distance` | **0.4035 м** | Центр масс → FL/FR |
| Задняя полубаза | `b_distance` | **0.4035 м** | Центр масс → RL/RR |
| Радиус колеса | `wheel_radius` | **0.10 м** | Диаметр 200 мм |
| Макс. угол руля | `max_steering_angle` | **35°** | Ограничение серво |
| Ход подвески | `rocker_limit` | **±25°** | Рычаг rocker |
| Клиренс | `ground_clearance` | **~0.20 м** | При нейтральной подвеске |

---

## 1.6 Конструкция подвески rocker-bogie

```
                    base_link
                        │
            ┌───────────┴───────────┐
            │                       │
        rocker_L               rocker_R
       /        \             /        \
   bogie_L    steer_FL   steer_FR   bogie_R
   /     \       │           │      /     \
steer_RL  ML   wheel_FL  wheel_FR  MR   steer_RR
   │                                        │
wheel_RL                                wheel_RR
```

**Суставы:**
- `rocker_*_joint`: revolute, ось Y, ±25° — адаптация к рельефу
- `steer_{FL,FR,RL,RR}_joint`: revolute, ось Z, ±35° — рулевые
- `drive_{FL,ML,RL,FR,MR,RR}_joint`: continuous, ось X — приводные

> Средние колёса (ML, MR) **не поворотные** — серво всегда в нейтрали 90°.

---

[◀ На главную](../../README.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Установка ▶](02_installation.md)
