# Команда индивидуальному колесу (после кинематики Аккермана)
# Используется: ackermann_calculator_node → serial_controller_node
string  wheel_name          # 'front_left'|'middle_left'|'rear_left'|'front_right'|'middle_right'|'rear_right'
float32 speed_cmd           # Целевая скорость колеса (м/с)
float32 angle_cmd           # Целевой угол сервопривода (°), 0..180. Для middle_* всегда initial_pos.
