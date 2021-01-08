# This message describes a tracked object from a RADAR sensor.

uint16 id                    # Unique object identifier

geometry_msgs/Pose pose      # Cartesian position and orientation
geometry_msgs/Twist velocity # Cartesian linear and angular velocity (twist)

BoundingBox box              # Bounding box of associated raw detections