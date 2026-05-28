# Команда движения от любого источника управления
# Используется: keyboard_node / dualshock4_node / trajectory_node → ackermann_calculator_node
float32 linear_velocity     # Линейная скорость (м/с): + вперёд, − назад
float32 steering_angle      # Угол поворота передней оси (°): + право, − лево
string  source              # Источник: 'keyboard' | 'ds4' | 'trajectory' | 'estop'
