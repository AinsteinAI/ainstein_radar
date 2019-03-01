# radar_ros_interface:
ROS nodes for interfacing with various radars based on a generic interface message format common to all radars.

Currently supported radars include:

- K79 (tested Feb. 2019)
- K77 (tested Feb. 2019)
- T79 with BSD firmware (tested Sept. 2018)

## Package Dependencies

[radar_sensor_msgs](https://github.com/AinsteinAI/radar_sensor_msgs) : Package defining custom radar message types. 

## Package Structure

```
radar_ros_interface/
├── CMakeLists.txt
├── include
│   └── radar_ros_interface
│       ├── config_t79_bsd.h
│       ├── radardata_to_laserscan.h
│       ├── radardata_to_pointcloud.h
│       ├── radar_interface.h
│       ├── radar_interface_k79.h
│       └── radar_interface_t79_bsd.h
├── launch
    ├── k77_node.launch
    ├── k79_node.launch
│   └── t79_bsd_node.launch
├── LICENSE
├── package.xml
├── readme.md
├── scripts
│   └── k79_gui.py
└── src
    ├── k79_node.cpp
    ├── radardata_to_laserscan.cpp
    ├── radardata_to_laserscan_node.cpp
    ├── radardata_to_pointcloud.cpp
    ├── radardata_to_pointcloud_node.cpp
    ├── radar_interface_k79.cpp
    ├── radar_interface_t79_bsd.cpp
    └── t79_bsd_node.cpp
```				

## Supported Radars and Usage

### K79:

The file k79_node.cpp implements a ROS node using the RadarInterfaceK79 interface class to create a UDP socket bound to the host IP address and port (which must match the radar's configuration), launch a thread to read and publish data to the RadarData message type.

**Command line usage:**	

The optional parameters are scoped private, eg host IP is read from /k79_node/host_ip in this case:

```bash
rosrun radar_ros_interface k79_node [_host_ip:=HOST_IP_ADDRESS] [_host_port:=HOST_UDP_PORT] [_radar_ip:=RADAR_IP_ADDRESS] [_radar_port:=RADAR_UDP_PORT] [_frame_id:=RADAR_FRAME_ID]
```

If unspecified, ```host_ip``` defaults to ```10.0.0.75```, ```radar_ip``` defaults to ```10.0.0.10```, ```radar_port``` defaults to ```7```, ```host_port``` defaults to ```1024``` and ```frame_id``` defaults to ```map```.

**Example launch file usage (from launch/k79_node.launch):**

```xml
<launch>
  <node name="k79_node" pkg="radar_ros_interface" type="k79_node" output="screen" required="true" >
    <param name="host_ip" value="10.0.0.75" />
    <param name="host_port" value="1024" />
    <param name="radar_ip" value="10.0.0.10" />
    <param name="radar_port" value="7" />
  </node>
</launch>
```

### K77 and T79+BSD:

The file ```src/t79_bsd_node.cpp``` implements a ROS node for both K77 and T79 using the RadarInterfaceT79BSD interface class.  With the current 4+1 firmware, this node works for both K77 and T79 *by specifying the radar type*.

These CAN radars require a [socketcan_bridge](http://wiki.ros.org/socketcan_bridge) node publishing CAN frames to the ```/received_messages``` ROS topic (see the launch file below for an example).  This package can be installed with:

```bash
sudo apt install ros-kinetic-socketcan-bridge
```

**Command line usage:**	

The optional parameters are scoped private, eg host IP is read from /k77_node/radar_type in this case:

```bash
rosrun radar_ros_interface t79_bsd_node [_radar_type:=RADAR_TYPE] [_frame_id:=RADAR_FRAME_ID]
```

where ```RADAR_TYPE``` is to be specified according to the following enum:

```
enum RadarType
{
    KANZA = 0,
    TIPI_79_FL,
    TIPI_79_FR,
    TIPI_79_RL,
    TIPI_79_RR,
    N_RADARS
};
```

For example, K77 is type ```0``` (KANZA), T79 on the front-left corner of a vehicle is type ```1``` (TIPI_79_FL) and so on.  If unspecified, ```radar_type``` defaults to ```0``` and ```frame_id``` defaults to ```map```.

**Example launch file usage (from launch/k77_node.launch):**

```xml
<launch>
  <node name="socketcan_bridge" pkg="socketcan_bridge" type="socketcan_bridge_node"  required="true" >
    <param name="can_device" value="can0" />
  </node>
  <node name="k77_node" pkg="radar_ros_interface" type="t79_bsd_node" required="true" >
    <param name="radar_type" value="0" />
  </node>
</launch>
```

**Example launch file usage (from launch/t79_bsd_node.launch):**

```xml
<launch>
  <node name="socketcan_bridge" pkg="socketcan_bridge" type="socketcan_bridge_node"  required="true" >
    <param name="can_device" value="can0" />
  </node>
  <node name="t79_node" pkg="radar_ros_interface" type="t79_bsd_node" required="true" >
    <param name="radar_type" value="1" />
  </node>
</launch>
```
