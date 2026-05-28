# Общий статус робота для диагностики и удобного мониторинга
# Используется: для удобного просмотра одной командой ros2 topic echo /rover/status
string  state                  # 'idle' | 'driving' | 'safety_block' | 'trajectory'
bool    safety_active          # Защита активна (от STM32 или вручную)
string  safety_flags           # Расшифровка: 'OK' | 'manual_block' | 'front_left=0x01 ...'
string  active_source          # текущий источник: 'keyboard'/'ds4'/'trajectory'/'safety'/'none'
uint8   wheels_connected       # 0..6
float32 current_speed          # Текущая скорость робота (м/с)
float32 current_steering       # Текущий угол поворота (°)
float32 distance_traveled      # Пройдено всего (м)
builtin_interfaces/Time stamp
