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

namespace ainstein_radar_drivers
{

RadarInterfaceK79::RadarInterfaceK79( ros::NodeHandle node_handle,
				      ros::NodeHandle node_handle_private ) :
  nh_( node_handle ),
  nh_private_( node_handle_private ),
  radar_data_msg_ptr_raw_( new ainstein_radar_msgs::RadarTargetArray )
{
  // Get the host IP and port:
  std::string host_ip_addr;
  nh_private_.param( "host_ip", host_ip_addr, std::string( "10.0.0.75" ) );

  int host_port;
  nh_private_.param( "host_port", host_port, 1024 );

  // Get the radar IP and port:
  std::string radar_ip_addr;
  nh_private_.param( "radar_ip", radar_ip_addr, std::string( "10.0.0.10" ) );

  int radar_port;  
  nh_private_.param( "radar_port", radar_port, 7 );

  // Get the radar data frame ID:
  nh_private_.param( "frame_id", frame_id_, std::string( "map" ) );

  // Set the frame ID:
  radar_data_msg_ptr_raw_->header.frame_id = frame_id_;

  // Create the radar driver object:
  driver_.reset( new RadarDriverK79( host_ip_addr, host_port,
				     radar_ip_addr, radar_port ) );
  
  // Start the data collection thread:
  thread_ = std::unique_ptr<std::thread>( new std::thread( &RadarInterfaceK79::mainLoop, this ) );
  mutex_.lock();
  is_running_ = true;
  mutex_.unlock();

  // Advertise the K-79 data using the ROS node handle:
  pub_radar_data_raw_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "targets/raw", 10 );
}

RadarInterfaceK79::~RadarInterfaceK79(void)
{
  mutex_.lock();
  is_running_ = false;
  mutex_.unlock();
  thread_->join();
} 

void RadarInterfaceK79::mainLoop(void)
{
  // Connect to the radar:
  driver_->connect();
  
  // Enter the main data receiving loop:
  bool running = true;
  std::vector<ainstein_radar_drivers::RadarTarget> targets;
  while( running && !ros::isShuttingDown() )
    {
      // Call to block until data has been received:
      if( driver_->receiveTargets( targets ) == false )
	{
	  ROS_WARN_STREAM( "Failed to read data: " << std::strerror( errno ) << std::endl );
	}
      else
	{
	  // Fill in the RadarTargetArray message from the received targets:
	  radar_data_msg_ptr_raw_->header.stamp = ros::Time::now();
	  radar_data_msg_ptr_raw_->targets.clear();
	  for( const auto &t : targets )
	    {
	      radar_data_msg_ptr_raw_->targets.push_back( targetToROSMsg( t ) );
	    }
	  
	  // Publish the target data:
	  pub_radar_data_raw_.publish( radar_data_msg_ptr_raw_ );
	}

      // Check whether the data loop should still be running:
      mutex_.lock();
      running = is_running_;
      mutex_.unlock();  
    }
}

} // namespace ainstein_radar_drivers
