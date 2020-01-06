#ifndef RADAR_PASSTHROUGH_FILTER_H_
#define RADAR_PASSTHROUGH_FILTER_H_

#define PCL_NO_PRECOMPILE
#include <ros/ros.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <dynamic_reconfigure/server.h>
#include <ainstein_radar_filters/PassthroughFilterConfig.h>
#include <ainstein_radar_filters/radar_target_array_to_point_cloud.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/filters/passthrough.h>

namespace ainstein_radar_filters
{
  class RadarPassthroughFilter
  {
  public:
    RadarPassthroughFilter( const ros::NodeHandle& node_handle,
			    const ros::NodeHandle& node_handle_private );
    ~RadarPassthroughFilter(){}

    void dynConfigCallback( const ainstein_radar_filters::PassthroughFilterConfig& config,
			    uint32_t level )
    {
      // Copy the configuration:
      config_ = config;

      // Set the filter parameters
      passthrough_filt_.setFilterFieldName( config_.filter_field_name );
      passthrough_filt_.setFilterLimits( config_.filter_limit_min,
					 config_.filter_limit_max );
      passthrough_filt_.setFilterLimitsNegative( config_.filter_limit_negative );
    }
    
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg );
  
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
  
    ros::Subscriber sub_radar_data_;
    ros::Publisher pub_radar_data_;

    pcl::PassThrough<PointRadarTarget> passthrough_filt_;
    
    tf2_ros::TransformListener listen_tf_;
    tf2_ros::Buffer buffer_tf_;

    // Parameters:
    std::string input_frame_, output_frame_;
    dynamic_reconfigure::Server<ainstein_radar_filters::PassthroughFilterConfig> dyn_config_server_;
    ainstein_radar_filters::PassthroughFilterConfig config_;
  };

} // namespace ainstein_radar_filters


#endif // RADAR_PASSTHROUGH_FILTER_H_
