/*
  Copyright <2018-2019> <Ainstein, Inc.>

  Rrosedistribution and use in source and binary forms, with or without modification, are permitted
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

#include "ainstein_radar_drivers/radar_interface_t79_bsd.h"

namespace ainstein_radar_drivers
{

RadarInterfaceT79BSD::RadarInterfaceT79BSD( ros::NodeHandle node_handle,
					    ros::NodeHandle node_handle_private ) :
  RadarInterface<can_msgs::Frame>( node_handle,
				   node_handle_private,
				   ros::this_node::getName(),
				   "received_messages",
				   "sent_messages" )
{
  // Store the radar type:
  int radar_type;
  nh_private_.param( "radar_type", radar_type, static_cast<int>( ConfigT79BSD::TIPI_79_FL ) );
  type_ = static_cast<ConfigT79BSD::RadarType>( radar_type );
  
  // Store the radar data frame ID:
  nh_private_.param( "frame_id", frame_id_, std::string( "map" ) );

  // Set the frame ID:
  radar_data_msg_ptr_raw_->header.frame_id = frame_id_;
  radar_data_msg_ptr_tracked_->header.frame_id = frame_id_;
  radar_data_msg_ptr_alarms_->header.frame_id = frame_id_;
  
  name_ = ConfigT79BSD::radar_names.at( type_ );
  startRadar();
}

void RadarInterfaceT79BSD::startRadar( void )
{
    // Send the start command:
    can_msgs::Frame can_frame;
    can_frame.header.frame_id = "0";
    can_frame.header.stamp = ros::Time::now();
    can_frame.is_rtr = false;
    can_frame.is_extended = false;
    can_frame.is_error = false;
    can_frame.dlc = 8;
    can_frame.id = ConfigT79BSD::RADAR_START_STOP;
    can_frame.data[0] = ConfigT79BSD::RADAR_START;
    can_frame.data[1] = ConfigT79BSD::RADAR_CYCLES;
    can_frame.data[2] = ConfigT79BSD::RESERVED;
    can_frame.data[3] = ConfigT79BSD::RESERVED;
    can_frame.data[4] = ConfigT79BSD::RESERVED;
    can_frame.data[5] = ConfigT79BSD::RESERVED;
    can_frame.data[6] = ConfigT79BSD::RESERVED;
    can_frame.data[7] = ConfigT79BSD::RESERVED;

    ROS_DEBUG( "starting data streaming for %s", name_.c_str() );
    pub_radar_cmd_.publish( can_frame );
}

void RadarInterfaceT79BSD::stopRadar( void )
{
    can_msgs::Frame can_frame;
    can_frame.header.frame_id = "0";
    can_frame.header.stamp = ros::Time::now();
    can_frame.is_rtr = false;
    can_frame.is_extended = false;
    can_frame.is_error = false;
    can_frame.dlc = 8;
    can_frame.id = ConfigT79BSD::RADAR_START_STOP;
    can_frame.data[0] = ConfigT79BSD::RADAR_STOP;
    can_frame.data[1] = ConfigT79BSD::RADAR_CYCLES;
    can_frame.data[2] = ConfigT79BSD::RESERVED;
    can_frame.data[3] = ConfigT79BSD::RESERVED;
    can_frame.data[4] = ConfigT79BSD::RESERVED;
    can_frame.data[5] = ConfigT79BSD::RESERVED;
    can_frame.data[6] = ConfigT79BSD::RESERVED;
    can_frame.data[7] = ConfigT79BSD::RESERVED;

    ROS_DEBUG( "stopping data streaming for %s", name_.c_str() );
    pub_radar_cmd_.publish( can_frame );
}

void RadarInterfaceT79BSD::dataMsgCallback( const can_msgs::Frame &msg )
{
    // Parse out heartbeat frame 1 messages:
    if( msg.id == ConfigT79BSD::heartbeat_1.at( type_ ) )
    {
      ROS_DEBUG( "received hearbeat frame 1 from %s", name_.c_str() );
    }
    // Parse out heartbeat frame 2 messages:
    if( msg.id == ConfigT79BSD::heartbeat_2.at( type_ ) )
    {
      ROS_DEBUG( "received hearbeat frame 2 from %s", name_.c_str() );
    }
    // Parse out start radar response messages:
    if( msg.id == ConfigT79BSD::start_stop_ret.at( type_ ) )
    {
        // Check whether the response is for a start or stop radar message:
        uint8_t start_stop_byte = msg.data[0];
        switch( start_stop_byte )
        {
        case ConfigT79BSD::RADAR_START:
            ROS_DEBUG( "received radar start from %s", name_.c_str() );
            break;

        case ConfigT79BSD::RADAR_STOP:
            ROS_DEBUG( "received radar stop from %s", name_.c_str() );
            break;

        default:
            ROS_ERROR( "received unknown radar start/stop from %s", name_.c_str() );
            break;
        }
    }
    // Parse out start of frame messages (KANZA comes first in BSD firmware):
    else if( msg.id == ConfigT79BSD::start_frame.at( type_ ) )
    {
        ROS_DEBUG( "received start frame from %s", name_.c_str() );
        // clear radar data message arrays here
        radar_data_msg_ptr_raw_->header.stamp = ros::Time::now();
        radar_data_msg_ptr_tracked_->header.stamp = ros::Time::now();
        radar_data_msg_ptr_alarms_->header.stamp = ros::Time::now();

        radar_data_msg_ptr_raw_->targets.clear();
        radar_data_msg_ptr_tracked_->targets.clear();
        radar_data_msg_ptr_alarms_->alarms.clear();
    }
    // Parse out end of frame messages:
    else if( msg.id == ConfigT79BSD::stop_frame.at( type_ ) )
    {
        ROS_DEBUG( "received stop frame from %s", name_.c_str() );
        pub_radar_data_raw_.publish( radar_data_msg_ptr_raw_ );
        pub_radar_data_tracked_.publish( radar_data_msg_ptr_tracked_ );
        pub_radar_data_alarms_.publish( radar_data_msg_ptr_alarms_ );
    }
    // Parse out raw target data messages:
    else if( msg.id == ConfigT79BSD::raw_id.at( type_ ) )
    {
        ROS_DEBUG( "received raw target from %s", name_.c_str() );

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
    else if( msg.id == ConfigT79BSD::tracked_id.at( type_ ) )
    {
        ROS_DEBUG( "received tracked target from %s", name_.c_str() );

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
    // Parse out BSD data messages:
    else if( msg.id == ConfigT79BSD::bsd_id.at( type_ ) )
    {
        ROS_DEBUG( "received BSD from %s", name_.c_str() );

        // Extract alarm data from the message:
        ainstein_radar_msgs::RadarAlarm alarms;
        alarms.LCA_alarm = ( 1UL << 6 ) & msg.data[1];
        alarms.CVW_alarm = ( 1UL << 4 ) & msg.data[1];
        alarms.BSD_alarm = ( 1UL << 2 ) & msg.data[1];

        radar_data_msg_ptr_alarms_->alarms.push_back( alarms );
    }
    else
    {
      ROS_DEBUG( "received message with unknown id: %02x", msg.id );
    }
}

} // namespace ainstein_drivers
