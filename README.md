# gazebo_radar_sensor_plugin

This package is intended to contain a collection of radar sensor plugins for different
Ainstein products. All radar plugins are based on the existing laser "Ray" plugin with
additional processing to convert ray detections to radar targets and publish them as
RadarData messages (this message type is available in the radar_ros_interface package).

While the libgazebo_radar_sensor_plugin.so shared library (plugin) is intended to be
generic enough to simulate any Ainstein radar, it currently only is valid for 1d radars
(a vertical scan dimension could be added and used to fill the elevation angle data),
does not support target speed (this might be able to be computed numerically from
successive scans) and the SNR is filled from the returned ray intensity (this has not
really been tested and may be a bad idea).

INSTALL INSTRUCTIONS:

Once the package has been built (as normal in the catkin workspace), copy the folder(s)
inside the models/ directory to somewhere on your GAZEBO_MODEL_PATH, eg ~/.gazebo/models

Also, make sure that ~/catkin_ws/devel/lib/ is on your GAZEBO_PLUGIN_PATH as this is where
the shared library will be built.

To test that the sensor is able to be loaded correctly in Gazebo (before adding it to another
model), use the provided test.launch file to attempt to start Gazebo with the radar sensor. If
you see an error about not finding the .so file, check your GAZEBO_PLUGIN_PATH.