#ifndef RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H_
#define RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H_

#include <ros/ros.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_filters/data_conversions.h>

namespace ainstein_radar_filters
{
  
  class RadarTargetArrayToPointCloud
  {
  public:
    RadarTargetArrayToPointCloud( ros::NodeHandle node_handle,
				  ros::NodeHandle node_handle_private );
    ~RadarTargetArrayToPointCloud() {}
    
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr &msg );
    
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber sub_radar_data_;
    ros::Publisher pub_cloud_;
    
  };
  
} // namespace ainstein_radar_filters

#endif // RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H_
