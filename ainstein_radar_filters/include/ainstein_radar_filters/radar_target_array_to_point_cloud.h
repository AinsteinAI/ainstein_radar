#ifndef RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H
#define RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_filters/pcl_point_radar_target.h>

namespace ainstein_radar_filters
{

class RadarTargetArrayToPointCloud
{
public:
  RadarTargetArrayToPointCloud( ros::NodeHandle node_handle,
				ros::NodeHandle node_handle_private );
  ~RadarTargetArrayToPointCloud(){}

  // Made the static to expose radar data to 3d point conversion, however this should
  // probably be moved to a utilities class with other such simple conversions.
  static void radarTargetToPclPoint( const ainstein_radar_msgs::RadarTarget &target,
				     PointRadarTarget& pcl_point )
  {
    pcl_point.x = cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
      * target.range;
    pcl_point.y = sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
      * target.range;
    pcl_point.z = sin( ( M_PI / 180.0 ) * target.elevation ) * target.range;

    pcl_point.snr = target.snr;
    pcl_point.range = target.range;
    pcl_point.speed = target.speed;
    pcl_point.azimuth = target.azimuth;
    pcl_point.elevation = target.elevation;
  }
  
  void radarTargetArrayCallback( const ainstein_radar_msgs::RadarTargetArray &msg );
  
private:
  pcl::PointCloud<PointRadarTarget> pcl_cloud_;
  sensor_msgs::PointCloud2 cloud_msg_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_radar_target_array_;
  ros::Publisher pub_cloud_;
};

} // namespace ainstein_radar_filters

#endif // RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H
