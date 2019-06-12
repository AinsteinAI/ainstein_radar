#ifndef RADAR_DATA_RANGE_FILTER_H_
#define RADAR_DATA_RANGE_FILTER_H_

#include <ros/ros.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_drivers
{
  class RadarDataRangeFilter
  {
  public:
    RadarDataRangeFilter( const ros::NodeHandle& node_handle,
			  const ros::NodeHandle& node_handle_private );
    ~RadarDataRangeFilter(){}

    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray& msg );
  
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
  
    ros::Subscriber sub_radar_data_;
    ros::Publisher pub_radar_data_;
    ainstein_radar_msgs::RadarTargetArray msg_filtered_;
  
    double min_range_;
    double max_range_;
  };

} // namespace ainstein_radar_drivers

#endif // RADAR_DATA_RANGE_FILTER_H_
