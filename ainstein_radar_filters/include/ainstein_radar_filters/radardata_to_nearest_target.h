#ifndef RADAR_DATA_TO_NEAREST_TARGET_H_
#define RADAR_DATA_TO_NEAREST_TARGET_H_

#include <ros/ros.h>

#include <ainstein_radar_msgs/RadarTargetStamped.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_filters
{
  class RadarDataToNearestTarget
  {
  public:
    RadarDataToNearestTarget( ros::NodeHandle node_handle,
			      ros::NodeHandle node_handle_private );
    ~RadarDataToNearestTarget(){}

    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::Ptr &msg );
  
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber sub_radar_data_;
    ros::Publisher pub_nearest_target_;
    ros::Publisher pub_nearest_target_data_;

    ainstein_radar_msgs::RadarTargetStamped nearest_target_msg_;
    ainstein_radar_msgs::RadarTargetArray nearest_target_array_msg_;

    ros::Time time_prev_;

    // Parameters:
    std::string target_type_;

    double min_dist_thresh_;
    double max_dist_thresh_;

    bool filter_data_;
    bool is_filter_init_;
    double data_lpf_alpha_;
    double data_lpf_timeout_;
  };

} // namespace ainstein_radar_filters

#endif
