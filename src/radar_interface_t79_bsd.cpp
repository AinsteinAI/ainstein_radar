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

#include "radar_ros_interface/radar_interface_t79_bsd.h"

RadarInterfaceT79BSD::RadarInterfaceT79BSD( void ) :
  RadarInterface<can_msgs::Frame>( ros::this_node::getName(),
				   "received_messages",
				   "sent_messages" )
{
  // Store the radar type:
  int radar_type;
  nh_private_.param( "radar_type", radar_type, static_cast<int>( ConfigT79BSD::TIPI_79_FL ) );
  type_ = static_cast<ConfigT79BSD::RadarType>( radar_type );
  
  // Store the radar data frame ID:
  nh_private_.param( "frame_id", radar_data_msg_.header.frame_id, std::string( "map" ) );

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
    can_frame.data[1] = 0xff;
    can_frame.data[2] = 0xff;
    can_frame.data[3] = 0xff;
    can_frame.data[4] = 0xff;
    can_frame.data[5] = 0xff;
    can_frame.data[6] = 0xff;
    can_frame.data[7] = 0xff;

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
    can_frame.data[1] = 0x00;
    can_frame.data[2] = 0xff;
    can_frame.data[3] = 0xff;
    can_frame.data[4] = 0xff;
    can_frame.data[5] = 0xff;
    can_frame.data[6] = 0xff;
    can_frame.data[7] = 0xff;

    pub_radar_cmd_.publish( can_frame );
}

void RadarInterfaceT79BSD::dataMsgCallback( const can_msgs::Frame &msg )
{
    // Parse out start radar response messages:
    if( msg.id == ConfigT79BSD::stop_ret.at( type_ ) )
    {
        // Check whether the response is for a start or stop radar message:
        uint8_t start_stop_byte = msg.data[0];
        switch( start_stop_byte )
        {
        case ConfigT79BSD::RADAR_START:
            ROS_INFO( "received radar start from %s", name_.c_str() );
            break;

        case ConfigT79BSD::RADAR_STOP:
            ROS_INFO( "received radar stop from %s", name_.c_str() );
            break;

        default:
            ROS_ERROR( "received unknown radar start/stop from %s", name_.c_str() );
            break;
        }
    }
    // Parse out start of frame messages (KANZA comes first in BSD firmware):
    else if( msg.id == ConfigT79BSD::start_frame.at( type_ ) )
    {
        ROS_INFO( "received start frame from %s", name_.c_str() );
        // clear radar data message arrays here
        radar_data_msg_.header.stamp = ros::Time::now();
        radar_data_msg_.raw_targets.clear();
        radar_data_msg_.tracked_targets.clear();
        radar_data_msg_.alarms.clear();
    }
    // Parse out end of frame messages:
    else if( msg.id == ConfigT79BSD::stop_frame.at( type_ ) )
    {
        ROS_INFO( "received stop frame from %s", name_.c_str() );
        pub_radar_data_.publish( radar_data_msg_ );
    }
    // Parse out raw target data messages:
    else if( msg.id == ConfigT79BSD::raw_id.at( type_ ) )
    {
        ROS_INFO( "received raw target from %s", name_.c_str() );

        // Extract the target ID and data from the message:
        radar_sensor_msgs::RadarTarget target;
        target.target_id = msg.data[0];
        target.snr = msg.data[1];
        target.range = (int16_t)( ( msg.data[2] << 8 ) + msg.data[3] ) / 100.0;
        target.speed = (int16_t)( ( msg.data[4] << 8 ) + msg.data[5] ) / 100.0;
        target.azimuth = (int16_t)( ( msg.data[6] << 8 ) + msg.data[7] ) / 100.0 * -1;
        target.elevation = 0.0;

        radar_data_msg_.raw_targets.push_back( target );
    }
    // Parse out tracked target data messages:
    else if( msg.id == ConfigT79BSD::tracked_id.at( type_ ) )
    {
        ROS_INFO( "received tracked target from %s", name_.c_str() );

        // Extract the target ID and data from the message:
        radar_sensor_msgs::RadarTarget target;
        target.target_id = msg.data[0];
        target.snr = msg.data[1];
        target.range = (int16_t)( ( msg.data[2] << 8 ) + msg.data[3] ) / 100.0;
        target.speed = (int16_t)( ( msg.data[4] << 8 ) + msg.data[5] ) / 100.0;
        target.azimuth = (int16_t)( ( msg.data[6] << 8 ) + msg.data[7] ) / 100.0 * -1;
        target.elevation = 0.0;

        radar_data_msg_.tracked_targets.push_back( target );
    }
    // Parse out BSD data messages:
    else if( msg.id == ConfigT79BSD::bsd_id.at( type_ ) )
    {
        ROS_INFO( "received BSD from %s", name_.c_str() );

        // Extract alarm data from the message:
        radar_sensor_msgs::RadarAlarm alarms;
        alarms.LCA_alarm = ( 1UL << 6 ) & msg.data[1];
        alarms.CVW_alarm = ( 1UL << 4 ) & msg.data[1];
        alarms.BSD_alarm = ( 1UL << 2 ) & msg.data[1];

        radar_data_msg_.alarms.push_back( alarms );
    }
    else
    {
        //ROS_ERROR( "received message with unknown id: %02x", msg.id );
    }
}
