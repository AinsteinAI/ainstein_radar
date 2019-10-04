#ifndef RADAR_TARGET_ARRAY_SPEED_FILTER_H_
#define RADAR_TARGET_ARRAY_SPEED_FILTER_H_

#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_filters
{

class RadarTargetArraySpeedFilter
{
public:
  RadarTargetArraySpeedFilter( ros::NodeHandle node_handle,
			       ros::NodeHandle node_handle_private );
  ~RadarTargetArraySpeedFilter(){}

  void radarVelCallback( const geometry_msgs::Twist &msg );     
  void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray &msg );

  double solveForAngle( double x, double y, double z );
  
private:
  ainstein_radar_msgs::RadarTargetArray msg_filtered_;
  
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_radar_data_;
  ros::Publisher pub_radar_data_;

  ros::Subscriber sub_radar_vel_;
  bool is_vel_available_;
  Eigen::Vector3d vel_world_;
  
  bool filter_stationary_;
  double min_speed_thresh_;

  bool filter_moving_;
  double max_speed_thresh_;

  bool compute_3d_;
  bool is_rotated_;
  
  tf2_ros::TransformListener listen_tf_;
  tf2_ros::Buffer buffer_tf_;

};

} // namespace ainstein_radar_filters

#endif // RADAR_TARGET_ARRAY_SPEED_FILTER_H_
