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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <iostream>
#include <cstdint>
#include <cerrno>

#include "ainstein_radar_drivers/radar_interface_k79.h"

const std::string RadarInterfaceK79::connect_cmd_str = std::string( "connect" );
const unsigned int RadarInterfaceK79::connect_res_len = 18;

const std::string RadarInterfaceK79::run_cmd_str = std::string( "run" );

const unsigned int RadarInterfaceK79::radar_msg_len = 1000;
const unsigned int RadarInterfaceK79::target_msg_len = 8;

RadarInterfaceK79::RadarInterfaceK79( ros::NodeHandle node_handle,
				      ros::NodeHandle node_handle_private ) :
  nh_( node_handle ),
  nh_private_( node_handle_private ),
  radar_data_msg_ptr_( new ainstein_radar_msgs::RadarData )
{
  // Store the host IP and port:
  nh_private_.param( "host_ip", host_ip_addr_, std::string( "10.0.0.75" ) );
  nh_private_.param( "host_port", host_port_, 1024 );

  // Store the radar IP and port:
  nh_private_.param( "radar_ip", radar_ip_addr_, std::string( "10.0.0.10" ) );
  nh_private_.param( "radar_port", radar_port_, 7 );

  // Store the radar data frame ID:
  nh_private_.param( "frame_id", radar_data_msg_ptr_->header.frame_id, std::string( "map" ) );
}

RadarInterfaceK79::~RadarInterfaceK79(void)
{
  mutex_.lock();
  is_running_ = false;
  mutex_.unlock();

  thread_->join();

  close( sockfd_ );
}

bool RadarInterfaceK79::connect(void)
{
  // Create the host UDP socket:
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if( sockfd_ < 0 )
    {
      std::cout << "ERROR >> Failed to create socket." << std::endl;
      return false;
    }

  // Configure the host sockaddr using the host IP and port:
  memset( &sockaddr_, 0, sizeof( sockaddr_ ) );
  sockaddr_.sin_family = AF_INET;
  sockaddr_.sin_port = htons( host_port_ );
  sockaddr_.sin_addr.s_addr = inet_addr( host_ip_addr_.c_str() );

  // Configure the radar sockaddr using the host IP and port:
  memset( &destaddr_, 0, sizeof( destaddr_ ) );
  destaddr_.sin_port = htons( radar_port_ );
  destaddr_.sin_addr.s_addr = inet_addr( radar_ip_addr_.c_str() );

  // Set socket options:
  int reuseaddr = 1;
  int res = setsockopt( sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr) );
  if( res < 0 )
    {
      std::cout << "ERROR >> Failed to set socket options: " << std::strerror( errno ) << std::endl;
      return false;
    }

  // Set socket timeout:
  struct timeval tv;
  tv.tv_sec = 3; // 3 second timeout
  tv.tv_usec = 0;
  res  = setsockopt( sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( tv ) );
  if( res < 0 )
    {
      std::cout << "ERROR >> Failed to set socket timeout: " << std::strerror( errno ) << std::endl;
      return false;
    }
    
  // Explicitly bind the host UDP socket:
  res = bind( sockfd_, (struct sockaddr *)( &sockaddr_ ), sizeof( sockaddr_ ) );
  if( res < 0 )
    {
      std::cout << "ERROR >> Failed to bind socket: " << std::strerror( errno ) << std::endl;
      return false;
    }

  // Try to receive data until the timeout to see if the K79 is already running:
  struct sockaddr_storage src_addr;
  socklen_t src_addr_len = sizeof( src_addr );
  res = recvfrom( sockfd_, (char* )buffer_, RadarInterfaceK79::radar_msg_len, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );
  if( res < 0 )
    {
      // If blocking recvfrom times out, errno is set to EAGAIN:
      if( errno == EAGAIN )
	{
	  // Send the connect command to the radar:
	  RadarInterfaceK79::connect_cmd_str.copy( buffer_, RadarInterfaceK79::connect_cmd_str.length() );
	  res = sendto( sockfd_, (char* )buffer_, RadarInterfaceK79::connect_cmd_str.length(), 0, ( struct sockaddr *)( &destaddr_ ), sizeof( destaddr_ ) );
	  if( res < 0 )
	    {
	      std::cout << "ERROR >> Failed to send connect command to radar: " << std::strerror( errno ) << std::endl;
	      return false;
	    }

	  // Wait for a response to the connect command:
	  res = recvfrom( sockfd_, (char* )buffer_, RadarInterfaceK79::connect_res_len, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );
	  if( res < 0 )
	    {
	      std::cout << "ERROR >> Failed to receive connect response from radar: " << std::strerror( errno ) << std::endl;
	      return false;
	    }
	  else
	    {
	      // TODO: print settings stored on radar from connect response message
	    }

	  // Send the run command to the radar:
	  RadarInterfaceK79::run_cmd_str.copy( buffer_, RadarInterfaceK79::run_cmd_str.length() );
	  res = sendto( sockfd_, (char* )buffer_, RadarInterfaceK79::run_cmd_str.length(), 0, ( struct sockaddr *)( &destaddr_ ), sizeof( destaddr_ ) );
	  if( res < 0 )
	    {
	      std::cout << "ERROR >> Failed to send run command to radar: " << std::strerror( errno ) << std::endl;
	      return false;
	    }
	}
      else // encountered some error other than timeout
	{
	  std::cout << "ERROR >> Failed when attempting to detect whether radar is running: " << std::strerror( errno ) << std::endl;
	  return false;
	}
    }
  
  // Start the data collection thread:
  thread_ = std::unique_ptr<std::thread>( new std::thread( &RadarInterfaceK79::mainLoop, this ) );
  mutex_.lock();
  is_running_ = true;
  mutex_.unlock();

  // Advertise the K-79 data using the ROS node handle:
  pub_radar_data_ = nh_private_.advertise<ainstein_radar_msgs::RadarData>( "data", 10 );
  
  return true;
}

void RadarInterfaceK79::mainLoop(void)
{
  // Received message length:
  int msg_len;

  // Structures to store where the UDP message came from:
  struct sockaddr_storage src_addr;
  socklen_t src_addr_len = sizeof( src_addr );

  // Enter the main data receiving loop:
  bool running = true;
  while( running && !ros::isShuttingDown() )
    {
      // Call to block until data has been received:
      msg_len = recvfrom( sockfd_, (char* )buffer_, RadarInterfaceK79::radar_msg_len, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );

      if( msg_len < 0 )
	{
	  std::cout << "WARNING >> Failed to read data: " << std::strerror( errno ) << std::endl;
	}
      else
	{
	  // Extract the sender's IP address:
	  struct sockaddr_in* sin = (struct sockaddr_in* )&src_addr;
	  unsigned char* src_ip = (unsigned char*)(&sin->sin_addr.s_addr);
	  // printf("source IP: %d.%d.%d.%d\n", src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
      
	  // Prepare the radar targets message:
	  radar_data_msg_ptr_->header.stamp = ros::Time(0); // ros::Time::now();
	  radar_data_msg_ptr_->raw_targets.clear();
	  radar_data_msg_ptr_->tracked_targets.clear();
	  radar_data_msg_ptr_->alarms.clear();

	  // Extract the target ID and data from the message:
	  if( ( msg_len % RadarInterfaceK79::target_msg_len ) != 0 )
	    {
	      std::cout << "WARNING >> Incorrect number of bytes: " << msg_len << std::endl;
	    }
	  else
	    {
	      ainstein_radar_msgs::RadarTarget target;
	      int offset;
	      for( int i = 0; i < ( msg_len / RadarInterfaceK79::target_msg_len ); ++i )
		{
		  offset = i * RadarInterfaceK79::target_msg_len;
		  target.target_id = i;
		  target.snr = 100.0; // K79 does not currently output SNR per target
		  target.azimuth = static_cast<int16_t>( ( buffer_[offset + 1] << 8 ) + buffer_[offset + 0] ) * -1.0 + 90.0; // 1 count = 1 deg, 90 deg offset
		  target.range = ( buffer_[offset + 2] ) * 0.1;   // 1 count = 0.1 m

		  // Speed is 0-127, with 0-64 negative (moving away) and 65-127 positive (moving towards).
		  // Note that 65 is the highest speed moving towards, hence the manipulation below.
		  if( buffer_[offset + 3] <= 64 ) // MOVING AWAY FROM RADAR
		    {
		      target.speed = -( buffer_[offset + 3] ) * 0.045; // 1 count = 0.045 m/s
		    }
		  else // MOVING TOWARDS RADAR
		    {
		      target.speed = -( buffer_[offset + 3] - 127 ) * 0.045; // 1 count = 0.045 m/s
		    }
	      
		  target.elevation = 0.0; // K79 does not output elevation angle

		  radar_data_msg_ptr_->raw_targets.push_back( target );
		}

	      // Publish the target data:
	      pub_radar_data_.publish( radar_data_msg_ptr_ );
	    }
	}

      // Check whether the data loop should still be running:
      mutex_.lock();
      running = is_running_;
      mutex_.unlock();  
     }
}
