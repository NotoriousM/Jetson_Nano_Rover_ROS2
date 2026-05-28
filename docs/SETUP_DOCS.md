# SETUP_DOCS.md — Как применить документацию к репозиторию

Этот файл описывает **пошаговые команды** для размещения всей документации  
в ваш репозиторий https://github.com/NotoriousM/Jetson_Nano_Rover_ROS2

---

## Шаг 1 — Создать структуру папок

```bash
# В корне репозитория:
cd ~/ros2_ws/src/Jetson_Nano_Rover_ROS2

mkdir -p docs/images docs/gif docs/reference_manual
```

**Итоговая структура:**
```
Jetson_Nano_Rover_ROS2/
├── README.md               ← ЗАМЕНИТЬ главным README
├── docs/
│   ├── images/             ← фото, скриншоты, схемы
│   │   └── README.md
│   ├── gif/                ← анимации
│   │   └── README.md
│   └── reference_manual/   ← технические главы
│       ├── 01_hardware.md
│       ├── 02_installation.md
│       ├── 03_ros2_nodes.md
│       ├── 04_protocol_stm32.md
│       ├── 05_ackermann_kinematics.md
│       ├── 06_odometry.md
│       ├── 07_network_setup.md
│       ├── 08_simulation.md
│       ├── 09_trajectories.md
│       └── udev_setup.md
├── robot_control_v3/       ← существующий код
└── rover_description/      ← существующий код
```

---

## Шаг 2 — Скопировать файлы документации

```bash
# Все файлы из этого архива лежат в той же структуре.
# Просто скопируйте их в репозиторий:

cp README.md ~/ros2_ws/src/Jetson_Nano_Rover_ROS2/
cp -r docs/ ~/ros2_ws/src/Jetson_Nano_Rover_ROS2/
```

---

## Шаг 3 — Добавить изображения

Пока папка `docs/images/` пустая (кроме README.md).  
Добавьте реальные файлы согласно инструкции в [`docs/images/README.md`](docs/images/README.md).

**Минимально необходимые для красивого README:**
- `docs/images/banner.png` — баннер (создайте в draw.io)
- `docs/images/rviz_screenshot.png` — скриншот из RViz2 симуляции

---

## Шаг 4 — Добавить GIF

Аналогично для `docs/gif/` — инструкция в [`docs/gif/README.md`](docs/gif/README.md).

**Минимально:** `docs/gif/keyboard_control.gif`

---

## Шаг 5 — Закоммитить

```bash
cd ~/ros2_ws/src/Jetson_Nano_Rover_ROS2

git add README.md docs/
git commit -m "docs: добавлена полная документация репозитория

- Главный README.md с содержанием-навигацией
- docs/reference_manual/ — 9 технических глав
- docs/images/ и docs/gif/ со структурой и инструкциями"

git push origin main
```

---

## Шаг 6 — Проверить отображение на GitHub

Открыть https://github.com/NotoriousM/Jetson_Nano_Rover_ROS2

Убедиться что:
- [ ] Главный README отображается с таблицей содержания
- [ ] Ссылки в таблице работают (ведут в `docs/reference_manual/`)
- [ ] Бейджи отображаются (нужны действующие ссылки)
- [ ] Изображения в `docs/images/` и `docs/gif/` видны

---

## Как обновить один раздел

```bash
# Пример: обновить главу про протокол
nano docs/reference_manual/04_protocol_stm32.md

git add docs/reference_manual/04_protocol_stm32.md
git commit -m "docs: обновлена глава 4 — протокол STM32"
git push
```

---

## Автоматическая генерация документации (опционально)

Если хотите автогенерацию с MkDocs:

```bash
pip3 install mkdocs mkdocs-material

# Создать mkdocs.yml
cat > mkdocs.yml << 'MKEOF'
site_name: Jetson Nano Rover — Документация
theme:
  name: material
  language: ru
nav:
  - Главная: README.md
  - Документация:
    - Аппаратура: docs/reference_manual/01_hardware.md
    - Установка: docs/reference_manual/02_installation.md
    - Ноды ROS2: docs/reference_manual/03_ros2_nodes.md
    - Протокол STM32: docs/reference_manual/04_protocol_stm32.md
    - Кинематика: docs/reference_manual/05_ackermann_kinematics.md
    - Одометрия: docs/reference_manual/06_odometry.md
    - Сеть: docs/reference_manual/07_network_setup.md
    - Симуляция: docs/reference_manual/08_simulation.md
    - Траектории: docs/reference_manual/09_trajectories.md
    - udev Setup: docs/reference_manual/udev_setup.md
MKEOF

# Превью локально
mkdocs serve
# Открыть http://localhost:8000

# Публикация на GitHub Pages
mkdocs gh-deploy
```
