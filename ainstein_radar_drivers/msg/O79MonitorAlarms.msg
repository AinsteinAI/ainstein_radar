# This message describes the O-79 monitor alarms.

uint8 TEMP_DISABLED=0
uint8 TEMP_HIGH=1
uint8 TEMP_HIGH_WARN=2
uint8 TEMP_NORMAL=3
uint8 TEMP_LOW_WARN=4
uint8 TEMP_LOW=5

uint8 FRAME_TIME_DISABLED=0
uint8 FRAME_TIME_NORMAL=0
uint8 FRAME_TIME_FAILED_HIGH=0

uint8 BLOCKED_DISABLED=0
uint8 BLOCKED_NORMAL=0
uint8 BLOCKED_FAILED=0

uint8 MONITOR_DISABLED=0
uint8 MONITOR_NORMAL=0
uint8 MONITOR_FAILED=0

std_msgs/Header header
uint8 temp
uint8 frame_time
uint8 blocked_radar
uint8 mmwave_monitor
