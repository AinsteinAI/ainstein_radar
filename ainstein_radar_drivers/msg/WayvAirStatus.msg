# This message describes the Wayv Air radar device status.

std_msgs/Header header
float32 board_temp     # Board temperature, *C
float32[] tx_temps     # Tx temperatures, *C
float32 pm_temp        # PM temperature, *C
float32 voltage        # Voltage, V
float32 power          # Power, mW
