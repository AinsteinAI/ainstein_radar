#ifndef RADAR_DATA_UTILITIES_H_
#define RADAR_DATA_UTILITIES_H_

#include <cmath>

#include <ainstein_radar_filters/data_conversions.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_msgs/BoundingBoxArray.h>
#include <tf2_eigen/tf2_eigen.h>

namespace ainstein_radar_filters
{
namespace utilities
{
void getTargetsBoundingBox(const ainstein_radar_msgs::RadarTargetArray& targets, ainstein_radar_msgs::BoundingBox& box)
{
  // Find the bounding box dimensions:
  Eigen::Vector3d min_point =
      Eigen::Vector3d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_point =
      Eigen::Vector3d(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                      -std::numeric_limits<double>::infinity());

  if (targets.targets.size() > 0)
  {
    for (const auto& t : targets.targets)
    {
      Eigen::Vector3d target_point;
      data_conversions::sphericalToCartesian(t.range, (M_PI / 180.0) * t.azimuth, (M_PI / 180.0) * t.elevation,
                                             target_point);
      min_point = min_point.cwiseMin(target_point);
      max_point = max_point.cwiseMax(target_point);
    }
  }

  // Check for the case in which the box is degenerative:
  if ((max_point - min_point).norm() < 1e-6)
  {
    min_point -= 0.1 * Eigen::Vector3d::Ones();
    max_point += 0.1 * Eigen::Vector3d::Ones();
  }

  // Compute box pose (identity orientation, geometric center is position):
  Eigen::Affine3d box_pose;
  box_pose.linear() = Eigen::Matrix3d::Identity();
  box_pose.translation() = min_point + (0.5 * (max_point - min_point));

  // Form the box message:
  box.pose = tf2::toMsg(box_pose);

  box.dimensions.x = max_point.x() - min_point.x();
  box.dimensions.y = max_point.y() - min_point.y();
  box.dimensions.z = 0.1;
}

}  // namespace utilities

}  // namespace ainstein_radar_filters

#endif
