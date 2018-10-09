# This message described composite RADAR data, including both raw and tracked targets as well
# as general information about the sensor, if applicable.
std_msgs/Header header

RadarTarget[] raw_targets
RadarTarget[] tracked_targets
RadarAlarm[] alarms
