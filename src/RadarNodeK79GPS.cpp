#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <iostream>
#include <cstdint>

#include "RadarNodeK79GPS.h"

RadarNodeK79GPS::RadarNodeK79GPS(std::string ip_addr, int port, std::string radar_name, std::string frame_id)
{
  // Store the client IP and port:
  ip_addr_ = ip_addr;
  port_ = port;

  // Store the radar name and data frame ID:
  radar_name_ = radar_name;
  radar_data_msg_.header.frame_id = frame_id;
  gps_data_msg_.header.frame_id = frame_id;
}

RadarNodeK79GPS::~RadarNodeK79GPS(void)
{
  mutex_.lock();
  is_running_ = false;
  mutex_.unlock();

  thread_->join();

  close( sockfd_ );
}

bool RadarNodeK79GPS::connect(void)
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

  struct timeval tv;
  tv.tv_sec = 10; // 10 second timeout
  tv.tv_usec = 0;
  res  = setsockopt( sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( tv ) );
  if( res < 0 )
    {
      std::cout << "Failed to set socket timeout, res: " << res << std::endl;
      return false;
    }
    
  // Explicitly bind the client-side UDP socket:
  res = bind( sockfd_, (struct sockaddr *)( &sockaddr_ ), sizeof( sockaddr_ ) );
  if( res < 0 )
    {
      std::cout << "Failed to bind socket, res: " << res << std::endl;
      return false;
    }

  thread_ = std::unique_ptr<std::thread>( new std::thread( &RadarNodeK79GPS::mainLoop, this ) );
  mutex_.lock();
  is_running_ = true;
  mutex_.unlock();

  // Advertise the K-79 data using the ROS node handle:
  pub_radar_data_ = node_handle_.advertise<radar_sensor_msgs::RadarData>( radar_name_+"_data", 10 );

  // Advertise the GPS data using the ROS node handle:
  pub_gps_data_ = node_handle_.advertise<radar_sensor_msgs::GPSData>( "gps_data", 10 );
  
  return true;
}

void RadarNodeK79GPS::mainLoop(void)
{
  // Buffer for the received messages:
  int msg_len;

  // Structures to store where the UDP message came from:
  struct sockaddr_storage src_addr;
  socklen_t src_addr_len = sizeof( src_addr );

  // Enter the main data receiving loop:
  bool running = true;
  while( running && !ros::isShuttingDown() )
    {
      // Call to block until data has been received:
      msg_len = recvfrom( sockfd_, (char* )buffer_, MSG_LEN, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );

      // Extract the sender's IP address:
      struct sockaddr_in* sin = (struct sockaddr_in* )&src_addr;
      unsigned char* src_ip = (unsigned char*)(&sin->sin_addr.s_addr);
      // printf("source IP: %d.%d.%d.%d\n", src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
      
      // Prepare the radar targets message:
      radar_data_msg_.header.stamp = ros::Time::now();
      radar_data_msg_.raw_targets.clear();
      radar_data_msg_.tracked_targets.clear();
      radar_data_msg_.alarms.clear();

      // Parse and fill the GPS data:
      uint32_t gps_time = static_cast<uint32_t>( buffer_[4] << 24 |
						 buffer_[5] << 16 |
						 buffer_[6] << 8 |
						 buffer_[7] ); 
      int32_t vel_n = static_cast<int32_t>( buffer_[0] << 24 |
					    buffer_[1] << 16 |
					    buffer_[2] << 8 |
					    buffer_[3] ); 
      int32_t vel_e = static_cast<int32_t>( buffer_[12] << 24 |
					    buffer_[13] << 16 |
					    buffer_[14] << 8 |
					    buffer_[15] ); 
      int32_t vel_d = static_cast<int32_t>( buffer_[8] << 24 |
					    buffer_[9] << 16 |
					    buffer_[10] << 8 |
					    buffer_[11] ); 

      gps_data_msg_.gps_time = 0.001 * static_cast<float>( gps_time );    // ms to s
      gps_data_msg_.velocity_ned.x = 0.001 * static_cast<float>( vel_n ); // mm/s to m/s
      gps_data_msg_.velocity_ned.y = 0.001 * static_cast<float>( vel_e ); // mm/s to m/s
      gps_data_msg_.velocity_ned.z = 0.001 * static_cast<float>( vel_d ); // mm/s to m/s

      // Publish the GPS data:
      pub_gps_data_.publish( gps_data_msg_ );	  

      // Extract the target ID and data from the message:
      if( ( msg_len % TARGET_MSG_LEN ) != 0 )
      {
	std::cout << "Incorrect number of bytes: " << msg_len << " Did recvfrom() time out?" << std::endl;
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
              target.azimuth = static_cast<int16_t>( ( buffer_[offset + 25] << 8 ) + buffer_[offset + 24] ) * -1.0 + 90.0; // 1 count = 1 deg, 90 deg offset
              target.range = ( buffer_[offset + 26] ) * 0.13;   // 1 count = 0.13 m
              target.speed = ( buffer_[offset + 27] ) * 0.045; // 1 count = 0.045 m/s
              target.elevation = 0.0; // K79 does not output elevation angle

	      if( target.range > 2.0 && target.range < 10.0 )
		{
		  radar_data_msg_.raw_targets.push_back( target );
		}
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
