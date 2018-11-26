## radar_ros_interface:
ROS nodes for interfacing with various radars based on a generic interface and message format common to all radars.

Currently supported radars include:

- T79 with BSD firmware
- K77 with BSD firmware (using the T79 BSD node interface)
- K79

### K79:

The file radar_node_k79.cpp implements a ROS node using the RadarNodeK79 class to create a UDP socket bound to the host IP address and port (which must match the radar's configuration), launch a thread to read and publish data to RadarData messages.

Command line usage:	

```bash
rosrun radar_ros_interface radar_node_k79 --ip IP_ADDRESS [--port UDP_PORT] [--name RADAR_NAME] [--frame RADAR_FRAME_ID]
```
