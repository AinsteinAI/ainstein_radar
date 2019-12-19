#ifndef RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H
#define RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_filters/data_conversions.h>

namespace ainstein_radar_filters
{
  
class RadarTargetArrayToPointCloud
{
public:
  RadarTargetArrayToPointCloud( ros::NodeHandle node_handle,
				ros::NodeHandle node_handle_private ) :
    nh_( node_handle ),
    nh_private_( node_handle_private )
  {
    pub_cloud_ = nh_private_.advertise<sensor_msgs::PointCloud2>( "cloud_out", 10 );
    sub_radar_target_array_ = nh_.subscribe( "radar_in", 10,
					     &RadarTargetArrayToPointCloud::radarTargetArrayCallback,
					     this );
    

  }
  ~RadarTargetArrayToPointCloud() {}

  void radarTargetArrayCallback( const ainstein_radar_msgs::RadarTargetArray &radar_msg )
  {
    sensor_msgs::PointCloud2 cloud_msg;
    data_conversions::radarTargetArrayToROSCloud( radar_msg, cloud_msg );
    pub_cloud_.publish( cloud_msg );
  }

private:
  ros::NodeHandle nh_, nh_private_;
  ros::Publisher pub_cloud_;
  ros::Subscriber sub_radar_target_array_;
  
};
  
} // namespace ainstein_radar_filters

#endif // RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H
