#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include <Eigen/Eigen>

namespace ainstein_radar_drivers
{
  // Simple ROS-independent bounding box class
  class BoundingBox
  {
  public:

  BoundingBox( const Eigen::Affine3d& pose,
	       const Eigen::Vector3d& dimensions ) :
      pose( pose ),
      dimensions( dimensions )
    {
    }
    BoundingBox( void )
    {
      pose = Eigen::Affine3d::Identity();
      dimensions = Eigen::Vector3d::Ones();
      }
    ~BoundingBox( void )
      {
      }


    Eigen::Affine3d pose;
    Eigen::Vector3d dimensions;
  };

} // namespace ainstein_radar_drivers

#endif // BOUNDING_BOX_H_
