# Глава 6. Одометрия

[◀ Кинематика](05_ackermann_kinematics.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Сеть ▶](07_network_setup.md)

---

## Содержание

- [6.1 Принцип работы](#61-принцип-работы)
- [6.2 Алгоритм](#62-алгоритм)
- [6.3 Публикуемые данные](#63-публикуемые-данные)
- [6.4 Сброс одометрии](#64-сброс-одометрии)
- [6.5 Накопление ошибки](#65-накопление-ошибки)

---

## 6.1 Принцип работы

`odometry_node` реализует **дифференциальную одометрию** по средним (неповоротным) колёсам.

Средние колёса выбраны потому что:
- Не поворотные — нет ошибки из-за угла серво
- Расположены ближе к центру масс
- Меньше проскальзывают при поворотах

Используются именованные поля `msg.middle_left.speed` и `msg.middle_right.speed` из `RoverWheelsState` — это исключает ошибку перепутанных индексов.

---

## 6.2 Алгоритм

```python
# Подписка на /wheels/state (50 Гц)
def wheels_callback(msg: RoverWheelsState):
    v_l = msg.middle_left.speed   * k_cvt   # коэффициент преобразования
    v_r = msg.middle_right.speed  * k_cvt

    v   = (v_l + v_r) / 2.0                 # линейная скорость
    omega = (v_r - v_l) / W                  # угловая скорость

    # Интегрирование (Эйлер, dt = 1/50 = 0.02 с)
    dt = current_time - last_time
    x   += v * cos(theta) * dt
    y   += v * sin(theta) * dt
    theta += omega * dt

    # Нормализация угла [-π, π]
    theta = atan2(sin(theta), cos(theta))
```

---

## 6.3 Публикуемые данные

**Топик `/odom` (`nav_msgs/Odometry`):**
```
header.frame_id: "odom"
child_frame_id:  "base_link"

pose.pose.position:    {x, y, z=0}
pose.pose.orientation: {quaternion из theta}
twist.twist.linear:    {x=v, y=0, z=0}
twist.twist.angular:   {z=omega}

pose.covariance:  диагональ [0.01, 0.01, 0, 0, 0, 0.01]  (упрощённо)
```

**TF `odom` → `base_link`:** публикуется синхронно с /odom через `TransformBroadcaster`.

**Частота публикации:** 50 Гц (параметр `publish_rate`).

---

## 6.4 Сброс одометрии

```bash
# Сбросить в начало координат (0, 0, 0°)
ros2 service call /reset_odometry \
  rover_interfaces/srv/ResetOdometry \
  '{x: 0.0, y: 0.0, yaw_deg: 0.0}'

# Установить произвольную стартовую позицию
ros2 service call /reset_odometry \
  rover_interfaces/srv/ResetOdometry \
  '{x: 1.5, y: 2.0, yaw_deg: 90.0}'

# Из RViz2: кнопка "2D Pose Estimate" → публикует /initialpose → одометрия сбрасывается
```

---

## 6.5 Накопление ошибки

Одометрия по колёсам накапливает ошибку со временем:

| Источник ошибки | Величина | Митигация |
|----------------|---------|-----------|
| Проскальзывание колёс | ~2-5% на скользкой поверхности | Твёрдая поверхность |
| Погрешность энкодера | ~0.1% | `speed_conversion_factor` |
| Численное интегрирование | ~0.001% | RK4 вместо Эйлера |
| Неточность геометрии | ~1-2% | Калибровка `track_width` |

> Для точной локализации рекомендуется EKF с IMU или SLAM.  
> `odometry_node` решает задачу базовой навигации для коротких дистанций.

---

[◀ Кинематика](05_ackermann_kinematics.md) · [Содержание](../../README.md#12-документация) · [Вперёд: Сеть ▶](07_network_setup.md)
