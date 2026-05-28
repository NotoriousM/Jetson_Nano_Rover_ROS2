# Состояние одного колеса (от STM32, через USB CDC)
# Используется: serial_controller_node → /wheel/{name}/state
string  wheel_name          # имя колеса (см. WheelCommand)
float32 speed               # Реальная скорость с энкодера (м/с)
float32 angle_cmd           # Угол который мы послали сервоприводу (°)
uint8   flag                # Флаг состояния от STM32 (usb_stop_flag)
bool    is_connected        # COM-порт подключен и работает
builtin_interfaces/Time stamp  # Время получения данных от STM32
