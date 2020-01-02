#ifndef RADAR_TARGET_ARRAY_TO_LASER_SCAN_H_
#define RADAR_TARGET_ARRAY_TO_LASER_SCAN_H_

#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_filters
{
  class RadarTargetArrayToLaserScan
  {
  public:
    RadarTargetArrayToLaserScan( ros::NodeHandle node_handle,
				 ros::NodeHandle node_handle_private );
    ~RadarTargetArrayToLaserScan(){}
  
    void radarVelCallback( const geometry_msgs::Twist &msg );     
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray &msg );
  
    bool useTarget( const ainstein_radar_msgs::RadarTarget &t );
  
  private:
    std::string data_topic_;
    std::string vel_topic_;
    std::string laser_scan_topic_;
    std::string frame_id_;

    int laser_scan_length_;
    sensor_msgs::LaserScan laser_scan_msg_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber sub_radar_data_;
    ros::Publisher pub_laser_scan_;

    ros::Subscriber sub_radar_vel_;
    bool is_vel_available_;
    Eigen::Vector3d vel_world_;
    double rel_speed_thresh_;
    double min_dist_thresh_;
    double max_dist_thresh_;
  };
 
} // namespace ainstein_radar_filters

#endif // RADAR_TARGET_ARRAY_TO_LASER_SCAN_H_
