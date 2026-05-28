# docs/gif/ — Анимированные демонстрации

Папка для GIF-анимаций, встроенных в README.

## Список файлов

| Файл | Описание | Размер | Длина |
|------|---------|:------:|:-----:|
| `keyboard_control.gif` | Управление с клавиатуры в RViz2 | ≤5 МБ | ~8 с |
| `lemniscate.gif` | Лемниската ∞ в RViz2 | ≤5 МБ | ~15 с |
| `rviz_joints.gif` | Ползунки суставов в RViz2 | ≤5 МБ | ~6 с |

## Запись GIF — метод 1: ffmpeg (рекомендуется)

```bash
# Установка
sudo apt install ffmpeg

# Шаг 1: записать видео (Ctrl+C для остановки)
ffmpeg -video_size 1920x1080 -framerate 25 -f x11grab -i :0.0 /tmp/screen.mp4

# Шаг 2: обрезать нужный фрагмент (с 2 по 12 секунду)
ffmpeg -i /tmp/screen.mp4 -ss 00:00:02 -t 00:00:10 /tmp/clip.mp4

# Шаг 3: конвертировать в GIF (800px ширина, 12 fps, оптимальная палитра)
ffmpeg -i /tmp/clip.mp4 \
  -vf "fps=12,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
  docs/gif/keyboard_control.gif
```

## Запись GIF — метод 2: Peek (GUI)

```bash
sudo apt install peek
peek
# Выбрать область экрана → Start → Stop → сохранить как GIF
```

## Оптимизация (обязательно!)

GIF-файлы занимают много места. После создания обязательно оптимизировать:

```bash
sudo apt install gifsicle
gifsicle -O3 --lossy=50 docs/gif/keyboard_control.gif \
  -o docs/gif/keyboard_control.gif

# Проверить размер
ls -lh docs/gif/
```

## Рекомендации

- Максимум 5 МБ на файл (GitHub показывает предупреждение для больших GIF)
- 12-15 fps — достаточно, 25 fps излишне
- Ширина 800-900px — хорошо читается на мобильных
- Убрать начальное ожидание/загрузку перед нарезкой
- Добавить текст командой `drawtext` в ffmpeg если нужно пояснить действие
