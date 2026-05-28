# Состояние всех 6 колёс одним сообщением — именованные поля!
# Решает проблему перепутанных индексов (msg.data[1] vs msg.middle_left.speed)
# Используется: serial_controller_node → odometry_node, monitoring
WheelState front_left
WheelState middle_left
WheelState rear_left
WheelState front_right
WheelState middle_right
WheelState rear_right
builtin_interfaces/Time stamp  # Общее время снимка
uint8 connected_count          # Сколько колёс подключено (0..6)
