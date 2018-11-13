#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <iostream>
#include <cstdint>

#include "RadarNodeK79.h"

RadarNodeK79::RadarNodeK79(std::string ip_addr, std::string radar_name, std::string frame_id)
{
  // Store the client IP and port:
  ip_addr_ = ip_addr;
  port_ = 8;

  // Store the radar name and data frame ID:
  radar_name_ = radar_name;
  radar_data_msg_.header.frame_id = frame_id;

  /*// Connect and launch the data thread:
  if( !connect() )
  {
      std::cout << "Failed to connect to the K79 device " << radar_name << std::endl;
  }*/
}

RadarNodeK79::~RadarNodeK79(void)
{
  close( sockfd_ );
  
  mutex_.lock();
  is_running_ = false;
  mutex_.unlock();

  thread_->join();
}

bool RadarNodeK79::connect(void)
{
  // Create the client-side UDP socket to listen on:
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if( sockfd_ < 0 )
    {
      std::cout << "Failed to create socket." << std::endl;
      return false;
    }

  // Configure the client-side UDP socket using the client IP and port:
  memset( &sockaddr_, 0, sizeof( sockaddr_ ) );

  sockaddr_.sin_family = AF_INET;
  sockaddr_.sin_port = htons( port_ );
  sockaddr_.sin_addr.s_addr = inet_addr( ip_addr_.c_str() );

  int reuseaddr = 1;
  int res = setsockopt( sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr) );
  if( res < 0 )
    {
      std::cout << "Failed to set socket options, res: " << res << std::endl;
      return false;
    }
    
  // Explicitly bind the client-side UDP socket:
  res = bind( sockfd_, (struct sockaddr *)( &sockaddr_ ), sizeof( sockaddr_ ) );
  if( res < 0 )
    {
      std::cout << "Failed to bind socket, res: " << res << std::endl;
      return false;
    }

  thread_ = std::unique_ptr<std::thread>( new std::thread( &RadarNodeK79::mainLoop, this ) );
  mutex_.lock();
  is_running_ = true;
  mutex_.unlock();

  // Advertise the K-79 data using the ROS node handle:
  pub_radar_data_ = node_handle_.advertise<radar_sensor_msgs::RadarData>( radar_name_+"_data", 10 );
  
  return true;
}

void RadarNodeK79::mainLoop(void)
{
  // Buffer for the received messages:
  int msg_len;

  // Structures to store where the UDP message came from:
  struct sockaddr_storage src_addr;
  socklen_t src_addr_len = sizeof( src_addr );

  // Enter the main data receiving loop:
  bool running = true;
  while( running )
    {
      // Call to block until data has been received:
      msg_len = recvfrom( sockfd_, (char* )buffer_, MSG_LEN, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );

      // Prepare the radar targets message:
      radar_data_msg_.header.stamp = ros::Time::now();
      radar_data_msg_.raw_targets.clear();
      radar_data_msg_.tracked_targets.clear();
      radar_data_msg_.alarms.clear();

      // Extract the target ID and data from the message:
      if( ( msg_len % TARGET_MSG_LEN ) != 0 )
      {
	std::cout << "Incorrect number of bytes: " << msg_len << " Message is non-conforming, skipping" << std::endl;
      }
      else
      {
          radar_sensor_msgs::RadarTarget target;
	  int offset;
          for( int i = 0; i < ( msg_len / TARGET_MSG_LEN ); ++i )
          {
    	      offset = i * TARGET_MSG_LEN;
              target.target_id = i;
              target.snr = 100.0; // K79 does not currently output SNR per target
              target.azimuth = (int16_t)( ( buffer_[offset + 1] << 8 ) + buffer_[offset + 0] ) * 1.0 - 90.0; // 1 count = 1 deg, 90 deg offset
              target.range = ( buffer_[offset + 2] ) * 0.1;   // 1 count = 0.1 m
              target.speed = ( buffer_[offset + 3] ) * 0.045; // 1 count = 0.045 m/s
              target.elevation = 0.0; // K79 does not output elevation angle

	      radar_data_msg_.raw_targets.push_back( target );
	  }

          // Publish the target data:
          pub_radar_data_.publish( radar_data_msg_ );
      }

      // Check whether the data loop should still be running:
      mutex_.lock();
      running = is_running_;
      mutex_.unlock();  
  
    }

}
