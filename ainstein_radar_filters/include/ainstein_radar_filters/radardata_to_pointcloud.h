#ifndef RADAR_DATA_TO_POINT_CLOUD_H_
#define RADAR_DATA_TO_POINT_CLOUD_H_

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_filters
{

class RadarDataToPointCloud
{
public:
  RadarDataToPointCloud( ros::NodeHandle node_handle,
			 ros::NodeHandle node_handle_private );
  ~RadarDataToPointCloud(){}

  pcl::PointXYZ radarDataToPclPoint( const ainstein_radar_msgs::RadarTarget &target );

  void radarVelCallback( const geometry_msgs::Twist &msg );     
  void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray &msg );

  double solveForAngle( double x, double y, double z );
  
private:
  pcl::PointCloud<pcl::PointXYZ> pcl_;
  sensor_msgs::PointCloud2 cloud_msg_;
    
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_radar_data_;
  ros::Publisher pub_pcl_;

  ros::Subscriber sub_radar_vel_;
  bool is_vel_available_;
  Eigen::Vector3d vel_world_;
  
  bool filter_stationary_;
  double min_speed_thresh_;

  bool filter_moving_;
  double max_speed_thresh_;

  double min_dist_thresh_;
  double max_dist_thresh_;
  bool compute_3d_;
  bool is_rotated_;
  
  tf2_ros::TransformListener listen_tf_;
  tf2_ros::Buffer buffer_tf_;

};

} // namespace ainstein_radar_filters

#endif // RADAR_DATA_TO_POINT_CLOUD_H_
