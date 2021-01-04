# This message describes a tracked object from a RADAR sensor.

RadarTarget target           # Object information in spherical coordinates

geometry_msgs/Pose pose      # Cartesian position and orientation
geometry_msgs/Twist velocity # Cartesian linear and angular velocity (twist)

BoundingBox box              # Bounding box of associated raw detections