#ifndef RADAR_DATA_RANGE_FILTER_H_
#define RADAR_DATA_RANGE_FILTER_H_

#include <ros/ros.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <dynamic_reconfigure/server.h>
#include <ainstein_radar_drivers/RangeFilterConfig.h>

namespace ainstein_radar_drivers
{
  class RadarDataRangeFilter
  {
  public:
    RadarDataRangeFilter( const ros::NodeHandle& node_handle,
			  const ros::NodeHandle& node_handle_private );
    ~RadarDataRangeFilter(){}
    
    void dynConfigCallback( const ainstein_radar_drivers::RangeFilterConfig& config, uint32_t level )
    {
      // Copy the configuration:
      config_ = config;
    }
    
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray& msg );
  
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
  
    ros::Subscriber sub_radar_data_;
    ros::Publisher pub_radar_data_;
    ainstein_radar_msgs::RadarTargetArray msg_filtered_;

    // Parameters:
    dynamic_reconfigure::Server<ainstein_radar_drivers::RangeFilterConfig> dyn_config_server_;
    ainstein_radar_drivers::RangeFilterConfig config_;
  };

} // namespace ainstein_radar_drivers

#endif // RADAR_DATA_RANGE_FILTER_H_
