# This message describes a target (detection) from a RADAR sensor.

uint16 id                    # Unique object identifier

float64 speed                # Doppler Index
float64 peakVal              # PeakVal
geometry_msgs/Pose pose      # Cartesian position and orientation
geometry_msgs/Twist velocity # Cartesian linear and angular velocity (twist)
