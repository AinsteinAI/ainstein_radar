## radar_ros_interface:
ROS nodes for interfacing with various radars based on a generic interface and message format common to all radars.

Currently supported radars include:

- T79 with BSD firmware
- K77 with BSD firmware (using the T79 BSD node interface)

Future supported radars include:

- K79 (currently, a test UDP client is implemented but not based on the generic radar interface)