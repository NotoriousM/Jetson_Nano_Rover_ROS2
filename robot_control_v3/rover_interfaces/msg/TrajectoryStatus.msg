# Статус выполнения траектории (от trajectory_node)
# Используется: ros2 topic echo /trajectory/status
string  state                  # 'idle' | 'running' | 'finished' | 'aborted'
float32 target_distance        # Целевая дистанция (м)
float32 traveled_distance      # Пройдено (м)
float32 remaining_distance     # Осталось (м)
float32 progress_percent       # 0..100
float32 elapsed_time           # с
builtin_interfaces/Time stamp
