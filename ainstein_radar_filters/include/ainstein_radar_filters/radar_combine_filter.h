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
  typedef sync_policies::ApproximateTime<ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray> RadarSyncPolicy4;
  typedef sync_policies::ApproximateTime<ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray> RadarSyncPolicy5;
  typedef sync_policies::ApproximateTime<ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray> RadarSyncPolicy6;
  typedef sync_policies::ApproximateTime<ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray> RadarSyncPolicy7;
  typedef sync_policies::ApproximateTime<ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
					 ainstein_radar_msgs::RadarTargetArray,
  					 ainstein_radar_msgs::RadarTargetArray> RadarSyncPolicy8;
					 
  class RadarCombineFilter
  {
  public:
    RadarCombineFilter( const ros::NodeHandle& node_handle,
			const ros::NodeHandle& node_handle_private );
    ~RadarCombineFilter(){}
    
    void dynConfigCallback( const ainstein_radar_filters::CombineFilterConfig& config,
			    uint32_t level )
    {
      // Copy the configuration
      config_ = config;

      // Set the filter parameters
      switch( n_topics_ )
	{
	case 1:
	  ROS_ERROR_STREAM( "Combine filter should only be used for 2+ topics." );
	  break;
	  
	case 2:
	  sync_policy_2_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
	  break;
	  
	case 3:
	  sync_policy_3_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
	  break;
	
      case 4:
	sync_policy_4_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
      	break;
	
      case 5:
	sync_policy_5_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
      	break;
	
      case 6:
	sync_policy_6_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
      	break;
	
      case 7:
      	sync_policy_7_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
	break;
	
	case 8:
	  sync_policy_8_->setMaxIntervalDuration( ros::Duration( config_.slop_duration ) );
	break;

      default:
	ROS_ERROR_STREAM( "Unsupported number of topics (" << sub_radar_data_.size() << ") specified." );
	}
    }

    // Register the callback to the input subscribers for synchronizing TWO topics
    void registerSubscribers( Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub1,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub2 )
    {
      sync_policy_2_.reset( new RadarSyncPolicy2( 10 ) );
      sync_2_.reset( new Synchronizer<RadarSyncPolicy2>( static_cast<const RadarSyncPolicy2&>( *sync_policy_2_ ),
    						      sub1, sub2 ) );
      sync_2_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2 ) );
    }
   
    // Register the callback to the input subscribers for synchronizing THREE topics
    void registerSubscribers( Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub1,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub2,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub3 )
    {
      sync_policy_3_.reset( new RadarSyncPolicy3( 10 ) );
      sync_3_.reset( new Synchronizer<RadarSyncPolicy3>( static_cast<const RadarSyncPolicy3&>( *sync_policy_3_ ),
    						      sub1, sub2, sub3 ) );
      sync_3_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2, _3 ) );
    }

    // Register the callback to the input subscribers for synchronizing FOUR topics
    void registerSubscribers( Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub1,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub2,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub3,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub4 )
    {
      sync_policy_4_.reset( new RadarSyncPolicy4( 10 ) );
      sync_4_.reset( new Synchronizer<RadarSyncPolicy4>( static_cast<const RadarSyncPolicy4&>( *sync_policy_4_ ),
							 sub1, sub2, sub3, sub4 ) );
      sync_4_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2, _3, _4 ) );
    }

    // Register the callback to the input subscribers for synchronizing FIVE topics
    void registerSubscribers( Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub1,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub2,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub3,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub4,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub5 )
    {
      sync_policy_5_.reset( new RadarSyncPolicy5( 10 ) );
      sync_5_.reset( new Synchronizer<RadarSyncPolicy5>( static_cast<const RadarSyncPolicy5&>( *sync_policy_5_ ),
							 sub1, sub2, sub3, sub4, sub5 ) );
      sync_5_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2, _3, _4, _5 ) );
    }

    // Register the callback to the input subscribers for synchronizing SIX topics
    void registerSubscribers( Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub1,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub2,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub3,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub4,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub5,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub6 )
    {
      sync_policy_6_.reset( new RadarSyncPolicy6( 10 ) );
      sync_6_.reset( new Synchronizer<RadarSyncPolicy6>( static_cast<const RadarSyncPolicy6&>( *sync_policy_6_ ),
							 sub1, sub2, sub3, sub4, sub5, sub6 ) );
      sync_6_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2, _3, _4, _5, _6 ) );
    }

    // Register the callback to the input subscribers for synchronizing SEVEN topics
    void registerSubscribers( Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub1,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub2,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub3,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub4,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub5,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub6,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub7 )
    {
      sync_policy_7_.reset( new RadarSyncPolicy7( 10 ) );
      sync_7_.reset( new Synchronizer<RadarSyncPolicy7>( static_cast<const RadarSyncPolicy7&>( *sync_policy_7_ ),
							 sub1, sub2, sub3, sub4, sub5, sub6, sub7 ) );
      sync_7_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2, _3, _4, _5, _6, _7 ) );
    }

    // Register the callback to the input subscribers for synchronizing EIGHT topics
    void registerSubscribers( Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub1,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub2,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub3,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub4,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub5,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub6,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub7,
    			      Subscriber<ainstein_radar_msgs::RadarTargetArray>& sub8 )
    {
      sync_policy_8_.reset( new RadarSyncPolicy8( 10 ) );
      sync_8_.reset( new Synchronizer<RadarSyncPolicy8>( static_cast<const RadarSyncPolicy8&>( *sync_policy_8_ ),
							 sub1, sub2, sub3, sub4, sub5, sub6, sub7, sub8 ) );
      sync_8_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2, _3, _4, _5, _6, _7, _8 ) );
    }

    // Data callback for TWO topics
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
    			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2 );

    // Data callback for THREE topics
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3 );
    
    // Data callback for FOUR topics
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4 );

    // Data callback for FIVE topics
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg5 );

    // Data callback for SIX topics
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg5,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg6 );

    // Data callback for SEVEN topics
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg5,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg6,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg7 );

    // Data callback for EIGHT topics
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg5,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg6,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg7,
			    const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg8 );

    void combineMsgs( const std::vector<ainstein_radar_msgs::RadarTargetArray>& msg_arr,
		      ainstein_radar_msgs::RadarTargetArray& msg_combined );

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    int n_topics_;
    
    std::string output_frame_id_;
    std::vector<std::unique_ptr<Subscriber<ainstein_radar_msgs::RadarTargetArray>>> sub_radar_data_;
    std::unique_ptr<RadarSyncPolicy2> sync_policy_2_;
    std::unique_ptr<Synchronizer<RadarSyncPolicy2>> sync_2_;
    std::unique_ptr<RadarSyncPolicy3> sync_policy_3_;
    std::unique_ptr<Synchronizer<RadarSyncPolicy3>> sync_3_;
    std::unique_ptr<RadarSyncPolicy4> sync_policy_4_;
    std::unique_ptr<Synchronizer<RadarSyncPolicy4>> sync_4_;
    std::unique_ptr<RadarSyncPolicy5> sync_policy_5_;
    std::unique_ptr<Synchronizer<RadarSyncPolicy5>> sync_5_;
    std::unique_ptr<RadarSyncPolicy6> sync_policy_6_;
    std::unique_ptr<Synchronizer<RadarSyncPolicy6>> sync_6_;
    std::unique_ptr<RadarSyncPolicy7> sync_policy_7_;
    std::unique_ptr<Synchronizer<RadarSyncPolicy7>> sync_7_;
    std::unique_ptr<RadarSyncPolicy8> sync_policy_8_;
    std::unique_ptr<Synchronizer<RadarSyncPolicy8>> sync_8_;
    ros::Publisher pub_radar_data_;

    tf2_ros::TransformListener listen_tf_;
    tf2_ros::Buffer buffer_tf_;

    // Parameters:
    dynamic_reconfigure::Server<ainstein_radar_filters::CombineFilterConfig> dyn_config_server_;
    ainstein_radar_filters::CombineFilterConfig config_;
  };

} // namespace ainstein_radar_filters


#endif // RADAR_COMBINE_FILTER_H_
