#ifndef RADAR_TARGET_CARTESIAN_H_
#define RADAR_TARGET_CARTESIAN_H_

#include <Eigen/Eigen>

namespace ainstein_radar_drivers
{
  // Simple ROS-independent Cartesian radar target class
  class RadarTargetCartesian
  {
  public:

  RadarTargetCartesian( int id, Eigen::Vector3d &pos,
			Eigen::Vector3d &vel ) :
    pos( pos ),
    vel( vel )
    {
    }
    RadarTargetCartesian( void )
    {
      id = 0;
      pos.setZero();
      vel.setZero();
      }
    ~RadarTargetCartesian( void )
      {
      }
    int id;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
  };

} // namespace ainstein_radar_drivers

#endif // RADAR_TARGET_CARTESIAN_H_
