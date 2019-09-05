# This message defines metainformation for a radar sensor.

std_msgs/Header header

#######################################################################
#                    Physical sensing limits                          #
#######################################################################

# The minimum and maximum range, in meters, of detections (targets)
# reported by the sensor.
float64 range_min
float64 range_max

# The minimum and maximum range, in meters per second, of detections
# (targets) reported by the sensor.
float64 speed_min
float64 speed_max

# The total field of view of the sensor in azimuth and elevation angles
# reported in degrees and assumed symmetrical.
float64 fov_azimuth  
float64 fov_elevation
