/*
  Copyright <2018-2019> <Ainstein, Inc.>

  Redistribution and use in source and binary forms, with or without modification, are permitted
  provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this list of
  conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this list of
  conditions and the following disclaimer in the documentation and/or other materials provided
  with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ainstein_radar_drivers/radar_interface_t79.h"

namespace ainstein_radar_drivers
{
  
  RadarInterfaceT79::RadarInterfaceT79( ros::NodeHandle node_handle,
					ros::NodeHandle node_handle_private ) :
    RadarInterface<can_msgs::Frame>( node_handle,
				     node_handle_private,
				     ros::this_node::getName(),
				     "received_messages",
				     "sent_messages" ),
    radar_info_msg_ptr_( new ainstein_radar_msgs::RadarInfo )
  {
    // Store the radar CAN ID:
    nh_private_.param( "can_id", can_id_, 0 );
  
    // Store the radar data frame ID:
    nh_private_.param( "frame_id", frame_id_, std::string( "map" ) );

    // Set the frame ID:
    radar_data_msg_ptr_raw_->header.frame_id = frame_id_;
    radar_data_msg_ptr_tracked_->header.frame_id = frame_id_;

    // Publish the RadarInfo message:
    publishRadarInfo();
    
    // Set up the CAN frame message:
    can_frame_msg_.header.frame_id = "0";
    can_frame_msg_.is_rtr = false;
    can_frame_msg_.is_extended = false;
    can_frame_msg_.is_error = false;
    can_frame_msg_.dlc = 8;

    // Set up dynamic reconfigure:
    dynamic_reconfigure::Server<ainstein_radar_drivers::ZoneOfInterestT79Config>::CallbackType f;
    f = boost::bind(&RadarInterfaceT79::dynConfigCallback, this, _1, _2);
    dyn_config_server_.setCallback(f);  
  
    startRadar();
  }

  void RadarInterfaceT79::dynConfigCallback( const ainstein_radar_drivers::ZoneOfInterestT79Config& config, uint32_t level )
  {
    updateZOI( config.min_range, config.max_range, config.min_angle, config.max_angle );
  }

  void RadarInterfaceT79::updateZOI( double range_min, double range_max,
				     double azimuth_min, double azimuth_max )
  {
    // Send the message to set the ZOI. Note that min and max azimuth are swapped comapred
    // to the ZOI message documentation because the opposite convention is held in ROS.
    can_frame_msg_.header.stamp = ros::Time::now();
    can_frame_msg_.id = RadarInterfaceT79::RADAR_COMMAND;
    can_frame_msg_.data[0] = RadarInterfaceT79::RADAR_SET_ZOI;
    can_frame_msg_.data[1] = can_id_; // radar CAN ID
    can_frame_msg_.data[2] = static_cast<uint8_t>( range_min * 3.0 );
    can_frame_msg_.data[3] = static_cast<uint8_t>( range_max * 3.0 );
    can_frame_msg_.data[4] = static_cast<uint8_t>( azimuth_max );
    can_frame_msg_.data[5] = static_cast<uint8_t>( azimuth_min );
    can_frame_msg_.data[6] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[7] = RadarInterfaceT79::RESERVED;

    pub_radar_cmd_.publish( can_frame_msg_ );  
  }
  
  void RadarInterfaceT79::startRadar( void )
  {
    // Send the start command:
    can_frame_msg_.header.stamp = ros::Time::now();
    can_frame_msg_.id = RadarInterfaceT79::RADAR_COMMAND;
    can_frame_msg_.data[0] = RadarInterfaceT79::RADAR_START;
    can_frame_msg_.data[1] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[2] = RadarInterfaceT79::RADAR_SEND_RAW |
      RadarInterfaceT79::RADAR_SEND_TRACKED;
    can_frame_msg_.data[3] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[4] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[5] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[6] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[7] = RadarInterfaceT79::RESERVED;

    ROS_DEBUG( "starting data streaming for radar with CAN ID %d", can_id_ );
    pub_radar_cmd_.publish( can_frame_msg_ );
  }

  void RadarInterfaceT79::stopRadar( void )
  {
    can_frame_msg_.header.stamp = ros::Time::now();
    can_frame_msg_.id = RadarInterfaceT79::RADAR_COMMAND;
    can_frame_msg_.data[0] = RadarInterfaceT79::RADAR_STOP;
    can_frame_msg_.data[1] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[2] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[3] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[4] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[5] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[6] = RadarInterfaceT79::RESERVED;
    can_frame_msg_.data[7] = RadarInterfaceT79::RESERVED;

    ROS_DEBUG( "stopping data streaming for radar with CAN ID %d", can_id_ );
    pub_radar_cmd_.publish( can_frame_msg_ );
  }

  void RadarInterfaceT79::dataMsgCallback( const can_msgs::Frame &msg )
  {
    // Parse out radar command response messages:
    if( msg.id == RadarInterfaceT79::RADAR_COMMAND_RET )
      {
        // Check whether the response is for a start or stop radar message:
        uint8_t command_type = msg.data[0];
        switch( command_type )
	  {
	  case RadarInterfaceT79::RADAR_START:
            ROS_DEBUG( "received radar start from radar with CAN ID %d", can_id_ );
            break;

	  case RadarInterfaceT79::RADAR_STOP:
            ROS_DEBUG( "received radar stop from radar with CAN ID %d", can_id_ );
            break;

	  case RadarInterfaceT79::RADAR_SET_PARAMS:
            ROS_DEBUG( "received radar parameters config from radar with CAN ID %d", can_id_ );
            break;

	  case RadarInterfaceT79::RADAR_SET_ZOI:
            ROS_DEBUG( "received radar ZOI config from radar with CAN ID %d", can_id_ );
            break;

	  default:
            ROS_ERROR( "received unknown radar start/stop from radar with CAN ID %d", can_id_ );
            break;
	  }
      }
    // Parse out start of frame messages:
    else if( msg.id == ( RadarInterfaceT79::RADAR_START_FRAME + can_id_ ) )
      {
        ROS_DEBUG( "received start frame from radar with CAN ID %d", can_id_ );
        // clear radar data message arrays here
        radar_data_msg_ptr_raw_->header.stamp = ros::Time::now();
        radar_data_msg_ptr_tracked_->header.stamp = ros::Time::now();

        radar_data_msg_ptr_raw_->targets.clear();
        radar_data_msg_ptr_tracked_->targets.clear();
      }
    // Parse out end of frame messages:
    else if( msg.id == ( RadarInterfaceT79::RADAR_STOP_FRAME + can_id_ ) )
      {
        ROS_DEBUG( "received stop frame from radar with CAN ID %d", can_id_ );
        pub_radar_data_raw_.publish( radar_data_msg_ptr_raw_ );
        pub_radar_data_tracked_.publish( radar_data_msg_ptr_tracked_ );
      }
    // Parse out raw target data messages:
    else if( msg.id == ( RadarInterfaceT79::RADAR_RAW_TARGET + can_id_ ) )
      {
        ROS_DEBUG( "received raw target from radar with CAN ID %d", can_id_ );

        // Extract the target ID and data from the message:
        ainstein_radar_msgs::RadarTarget target;
        target.target_id = msg.data[0];
        target.snr = msg.data[1];

	// Range scaling is 0.01m per count:
	target.range = (int16_t)( ( msg.data[2] << 8 ) + msg.data[3] ) / 100.0;

	// Speed scaling is 0.01m/s per count, +ve AWAY from radar, -ve TOWARDS:
	target.speed = (int16_t)( ( msg.data[4] << 8 ) + msg.data[5] ) / 100.0;

	// Azimuth angle scaling is -0.01rad per count: 
	target.azimuth = (int16_t)( ( msg.data[6] << 8 ) + msg.data[7] ) / 100.0 * -1;

	// Elevation angle is unused for T79:
	target.elevation = 0.0;

        radar_data_msg_ptr_raw_->targets.push_back( target );
      }
    // Parse out tracked target data messages:
    else if( msg.id == ( RadarInterfaceT79::RADAR_TRACKED_TARGET + can_id_ ) )
      {
        ROS_DEBUG( "received tracked target from radar with CAN ID %d", can_id_ );

        // Extract the target ID and data from the message:
        ainstein_radar_msgs::RadarTarget target;
        target.target_id = msg.data[0];
        target.snr = msg.data[1];
        target.range = (int16_t)( ( msg.data[2] << 8 ) + msg.data[3] ) / 100.0;
        target.speed = (int16_t)( ( msg.data[4] << 8 ) + msg.data[5] ) / 100.0;
        target.azimuth = (int16_t)( ( msg.data[6] << 8 ) + msg.data[7] ) / 100.0 * -1;
        target.elevation = 0.0;

        radar_data_msg_ptr_tracked_->targets.push_back( target );
      }
    else
      {
        ROS_DEBUG( "received message with unknown id: %02x", msg.id );
      }
  }

  void RadarInterfaceT79::publishRadarInfo( void )
  {    
    // Advertise the T79 sensor info (LATCHED):
    pub_radar_info_ = nh_private_.advertise<ainstein_radar_msgs::RadarInfo>( "radar_info", 10, true );

    // Form the RadarInfo message which is fixed for a given sensor:
    radar_info_msg_ptr_->header.stamp = ros::Time::now();
    radar_info_msg_ptr_->header.frame_id = frame_id_;

    radar_info_msg_ptr_->update_rate = UPDATE_RATE;
    radar_info_msg_ptr_->max_num_targets = MAX_NUM_TARGETS;
  
    radar_info_msg_ptr_->range_min = RANGE_MIN;
    radar_info_msg_ptr_->range_max = RANGE_MAX;
  
    radar_info_msg_ptr_->speed_min = SPEED_MIN;
    radar_info_msg_ptr_->speed_max = SPEED_MAX;

    radar_info_msg_ptr_->azimuth_min = AZIMUTH_MIN;
    radar_info_msg_ptr_->azimuth_max = AZIMUTH_MAX;

    radar_info_msg_ptr_->elevation_min = ELEVATION_MIN;
    radar_info_msg_ptr_->elevation_max = ELEVATION_MAX;

    radar_info_msg_ptr_->range_resolution = RANGE_RES;
    radar_info_msg_ptr_->range_accuracy = RANGE_ACC;
  
    radar_info_msg_ptr_->speed_resolution = SPEED_RES;
    radar_info_msg_ptr_->speed_accuracy = SPEED_ACC;

    radar_info_msg_ptr_->azimuth_resolution = AZIMUTH_RES;
    radar_info_msg_ptr_->azimuth_accuracy = AZIMUTH_ACC;

    radar_info_msg_ptr_->elevation_resolution = ELEVATION_RES;
    radar_info_msg_ptr_->elevation_accuracy = ELEVATION_ACC;
  
    // Publish the RadarInfo message once since it's latched:
    pub_radar_info_.publish( radar_info_msg_ptr_ );
  }

} // namespace ainstein_drivers
