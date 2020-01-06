#ifndef RADAR_COMBINE_FILTER_H_
#define RADAR_COMBINE_FILTER_H_

#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#include <ainstein_radar_filters/CombineFilterConfig.h>
#include <ainstein_radar_filters/radar_target_array_to_point_cloud.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_filters
{
  typedef message_filters::sync_policies::ApproximateTime<ainstein_radar_msgs::RadarTargetArray,
							 ainstein_radar_msgs::RadarTargetArray> RadarSyncPolicy;
					 
  class RadarCombineFilter
  {
  public:
    RadarCombineFilter( const ros::NodeHandle& node_handle,
			const ros::NodeHandle& node_handle_private );
    ~RadarCombineFilter(){}
    
    void dynConfigCallback( const ainstein_radar_filters::CombineFilterConfig& config,
			    uint32_t level )
    {
      // Copy the configuration:
      config_ = config;

      // Set the filter parameters
      sync_policy_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
    }
    
  void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg_A,
			  const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg_B );
  
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    std::string output_frame_id_;
    message_filters::Subscriber<ainstein_radar_msgs::RadarTargetArray> sub_radar_data_A_;
    message_filters::Subscriber<ainstein_radar_msgs::RadarTargetArray> sub_radar_data_B_;
    std::unique_ptr<RadarSyncPolicy> sync_policy_;
    std::unique_ptr<message_filters::Synchronizer<RadarSyncPolicy>> sync_;
    ros::Publisher pub_radar_data_;

    tf2_ros::TransformListener listen_tf_;
    tf2_ros::Buffer buffer_tf_;

    // Parameters:
    dynamic_reconfigure::Server<ainstein_radar_filters::CombineFilterConfig> dyn_config_server_;
    ainstein_radar_filters::CombineFilterConfig config_;
  };

} // namespace ainstein_radar_filters


#endif // RADAR_COMBINE_FILTER_H_
