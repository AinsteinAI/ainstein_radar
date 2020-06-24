/*
  Copyright <2020> <Ainstein, Inc.>

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

#include "ainstein_radar_drivers/radar_interface_o79_can.h"

namespace ainstein_radar_drivers
{
  
  RadarInterfaceO79CAN::RadarInterfaceO79CAN( ros::NodeHandle node_handle,
					      ros::NodeHandle node_handle_private ) :
    RadarInterface<can_msgs::Frame>( node_handle,
				     node_handle_private,
				     ros::this_node::getName(),
				     "received_messages",
				     "sent_messages" ),
    radar_info_msg_ptr_( new ainstein_radar_msgs::RadarInfo )
  {
    // Store the radar data frame ID:
    nh_private_.param( "can_id", can_id_str_, std::string( "0x18FFB24D" ) );
    nh_private_.param( "frame_id", frame_id_, std::string( "map" ) );

    // Convert the CAN ID string to an int:
    can_id_ = std::stoul( can_id_str_, nullptr, 16 );
    
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
  }

  void RadarInterfaceO79CAN::dataMsgCallback( const can_msgs::Frame &msg )
  {
    if( msg.id == can_id_ )
      {
	// Parse out start of frame messages:
	if( msg.data[4]==0xFF && msg.data[5]==0xFF && msg.data[6]==0xFF && msg.data[7]==0xFF )
	  {
	    ROS_DEBUG( "received start frame from radar" );
	    // clear radar data message arrays here
	    radar_data_msg_ptr_raw_->header.stamp = ros::Time::now();
	    radar_data_msg_ptr_tracked_->header.stamp = ros::Time::now();

	    radar_data_msg_ptr_raw_->targets.clear();
	    radar_data_msg_ptr_tracked_->targets.clear();
	  }
	// Parse out end of frame messages:
	else if( msg.data[0]==0xFF && msg.data[1]==0xFF && msg.data[2]==0xFF && msg.data[3]==0xFF )
	  {
	    ROS_DEBUG( "received stop frame from radar" );
	    if( radar_data_msg_ptr_raw_->targets.size() > 0 )
	      {
		pub_radar_data_raw_.publish( radar_data_msg_ptr_raw_ );
	      }
	    if( radar_data_msg_ptr_tracked_->targets.size() > 0 )
	      {
		pub_radar_data_tracked_.publish( radar_data_msg_ptr_tracked_ );
	      }
	  }
	// Parse out raw target data messages:
	else if( msg.data[0] == 0x00 )
	  {
	    ROS_DEBUG( "received raw target from radar" );
	
	    // Extract the target ID and data from the message:
	    ainstein_radar_msgs::RadarTarget target;
	    target.target_id = msg.data[0];
	    target.snr = msg.data[1];

	    // Range scaling is 0.1m per count:
	    target.range = (uint16_t)( ( msg.data[2] << 8 ) + msg.data[3] ) / 10.0;

	    // Speed scaling is 0.01m/s per count, +ve AWAY from radar, -ve TOWARDS:
	    target.speed = (int16_t)( ( msg.data[4] << 8 ) + msg.data[5] ) * 0.045;

	    // Azimuth angle scaling is -0.01rad per count: 
	    target.azimuth = (int8_t)( msg.data[6] );

	    // Elevation angle is unused for O79:
	    target.elevation = (int8_t)( msg.data[7] );

	    radar_data_msg_ptr_raw_->targets.push_back( target );
	  }
	// Parse out tracked target data messages:
	else if( msg.data[0] == 0x01 )
	  {
	    ROS_DEBUG( "received tracked target from radar" );

	    // Extract the target ID and data from the message:
	    ainstein_radar_msgs::RadarTarget target;
	    target.target_id = msg.data[0];
	    target.snr = msg.data[1];

	    // Range scaling is 0.01m per count:
	    target.range = (int16_t)( ( msg.data[2] << 8 ) + msg.data[3] ) / 10.0;

	    // Speed scaling is 0.01m/s per count, +ve AWAY from radar, -ve TOWARDS:
	    target.speed = (int16_t)( ( msg.data[4] << 8 ) + msg.data[5] ) * 0.045;

	    // Azimuth angle scaling is -0.01rad per count: 
	    target.azimuth = (int8_t)( msg.data[6] );

	    // Elevation angle is unused for O79:
	    target.elevation = (int8_t)( msg.data[7] );

	    radar_data_msg_ptr_tracked_->targets.push_back( target );
	  }
	else
	  {
	    ROS_DEBUG( "received message with unknown id: %02x", msg.id );
	  }
      }
  }

  void RadarInterfaceO79CAN::publishRadarInfo( void )
  {    
    // Advertise the O79 sensor info (LATCHED):
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
