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
  using namespace message_filters;
  typedef sync_policies::ApproximateTime<ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray> RadarSyncPolicy2;
  typedef sync_policies::ApproximateTime<ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray> RadarSyncPolicy3;
					 
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

      // Set the filter parameters (set for all topic numbers for now)
      // sync_policy_2_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
      // sync_policy_3_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
    }

    void registerSubscribers( Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub1,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub2 )
    {
      sync_policy_2_.reset( new RadarSyncPolicy2( 10 ) );
      sync_2_.reset( new Synchronizer<RadarSyncPolicy2>( static_cast<const RadarSyncPolicy2&>( *sync_policy_2_ ),
    						      sub1, sub2 ) );
      sync_2_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2 ) );
    }
   
    void registerSubscribers( Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub1,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub2,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub3 )
    {
      sync_policy_3_.reset( new RadarSyncPolicy3( 10 ) );
      sync_3_.reset( new Synchronizer<RadarSyncPolicy3>( static_cast<const RadarSyncPolicy3&>( *sync_policy_3_ ),
    						      sub1, sub2, sub3 ) );
      sync_3_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2, _3 ) );
    }
    
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
    			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2 );

    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3 );

    void combineMsgs( const std::vector<ainstein_radar_msgs::RadarTargetArray>& msg_arr,
		      ainstein_radar_msgs::RadarTargetArray& msg_combined );

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    std::string output_frame_id_;
    std::vector<std::unique_ptr<Subscriber<ainstein_radar_msgs::RadarTargetArray>>> sub_radar_data_;
    std::unique_ptr<RadarSyncPolicy2> sync_policy_2_;
    std::unique_ptr<Synchronizer<RadarSyncPolicy2>> sync_2_;
    std::unique_ptr<RadarSyncPolicy3> sync_policy_3_;
    std::unique_ptr<Synchronizer<RadarSyncPolicy3>> sync_3_;
    ros::Publisher pub_radar_data_;

    tf2_ros::TransformListener listen_tf_;
    tf2_ros::Buffer buffer_tf_;

    // Parameters:
    dynamic_reconfigure::Server<ainstein_radar_filters::CombineFilterConfig> dyn_config_server_;
    ainstein_radar_filters::CombineFilterConfig config_;
  };

} // namespace ainstein_radar_filters


#endif // RADAR_COMBINE_FILTER_H_
