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

#include "RadarNodeK79.h"

const std::string RadarNodeK79::connect_cmd_str = std::string( "connect" );
const unsigned int RadarNodeK79::connect_res_len = 18;

const std::string RadarNodeK79::run_cmd_str = std::string( "run" );

const unsigned int RadarNodeK79::radar_msg_len = 1000;
const unsigned int RadarNodeK79::target_msg_len = 8;

RadarNodeK79::RadarNodeK79( std::string host_ip_addr, int host_port, std::string radar_ip_addr, int radar_port, std::string radar_name, std::string frame_id )
{
  // Store the host IP and port:
  host_ip_addr_ = host_ip_addr;
  host_port_ = host_port;

  // Store the radar IP and port:
  radar_ip_addr_ = radar_ip_addr;
  radar_port_ = radar_port;

  // Store the radar name and data frame ID:
  radar_name_ = radar_name;
  radar_data_msg_.header.frame_id = frame_id;
}

RadarNodeK79::~RadarNodeK79(void)
{
  mutex_.lock();
  is_running_ = false;
  mutex_.unlock();

  thread_->join();

  close( sockfd_ );
}

bool RadarNodeK79::connect(void)
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
  res = recvfrom( sockfd_, (char* )buffer_, RadarNodeK79::radar_msg_len, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );
  if( res < 0 )
    {
      // If blocking recvfrom times out, errno is set to EAGAIN:
      if( errno == EAGAIN )
	{
	  // Send the connect command to the radar:
	  RadarNodeK79::connect_cmd_str.copy( buffer_, RadarNodeK79::connect_cmd_str.length() );
	  res = sendto( sockfd_, (char* )buffer_, RadarNodeK79::connect_cmd_str.length(), 0, ( struct sockaddr *)( &destaddr_ ), sizeof( destaddr_ ) );
	  if( res < 0 )
	    {
	      std::cout << "ERROR >> Failed to send connect command to radar: " << std::strerror( errno ) << std::endl;
	      return false;
	    }

	  // Wait for a response to the connect command:
	  res = recvfrom( sockfd_, (char* )buffer_, RadarNodeK79::connect_res_len, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );
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
	  RadarNodeK79::run_cmd_str.copy( buffer_, RadarNodeK79::run_cmd_str.length() );
	  res = sendto( sockfd_, (char* )buffer_, RadarNodeK79::run_cmd_str.length(), 0, ( struct sockaddr *)( &destaddr_ ), sizeof( destaddr_ ) );
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
      msg_len = recvfrom( sockfd_, (char* )buffer_, RadarNodeK79::radar_msg_len, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );

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
	  radar_data_msg_.header.stamp = ros::Time(0); // ros::Time::now();
	  radar_data_msg_.raw_targets.clear();
	  radar_data_msg_.tracked_targets.clear();
	  radar_data_msg_.alarms.clear();

	  // Extract the target ID and data from the message:
	  if( ( msg_len % RadarNodeK79::target_msg_len ) != 0 )
	    {
	      std::cout << "WARNING >> Incorrect number of bytes: " << msg_len << std::endl;
	    }
	  else
	    {
	      radar_sensor_msgs::RadarTarget target;
	      int offset;
	      for( int i = 0; i < ( msg_len / RadarNodeK79::target_msg_len ); ++i )
		{
		  offset = i * RadarNodeK79::target_msg_len;
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

		  radar_data_msg_.raw_targets.push_back( target );
		}

	      // Publish the target data:
	      pub_radar_data_.publish( radar_data_msg_ );
	    }
	}

      // Check whether the data loop should still be running:
      mutex_.lock();
      running = is_running_;
      mutex_.unlock();  
     }
}
