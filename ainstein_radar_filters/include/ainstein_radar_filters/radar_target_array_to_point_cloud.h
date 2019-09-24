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

  void radarTargetToPclPoint( const ainstein_radar_msgs::RadarTarget &target,
			      PointRadarTarget& pcl_point );
  
  void radarTargetArrayCallback( const ainstein_radar_msgs::RadarTargetArray &msg );
  
private:
  pcl::PointCloud<PointRadarTarget> pcl_cloud_;
  sensor_msgs::PointCloud2 cloud_msg_;
    
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_radar_target_array_;
  ros::Publisher pub_cloud_;
  
  tf2_ros::TransformListener listen_tf_;
  tf2_ros::Buffer buffer_tf_;
};

} // namespace ainstein_radar_filters

#endif // RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H
