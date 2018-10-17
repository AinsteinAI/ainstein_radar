#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <iostream>
#include <cstdint>

#include <radar_to_rviz/TestPacket.h>

#include "ClientK79.h"

ClientK79::ClientK79(std::string client_ip, int client_port)
{
  // Store the client IP and port:
  ip_addr_ = client_ip;
  port_ = client_port;
}

ClientK79::~ClientK79(void)
{
  mutex_.lock();
  is_running_ = false;
  mutex_.unlock();

  thread_->join();

  std::cout << "Destructor finished." << std::endl;
}

bool ClientK79::connect(void)
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

  // Explicitly bind the client-side UDP socket:
  int res = bind( sockfd_, (struct sockaddr *)( &sockaddr_ ), sizeof( sockaddr_ ) );
  if( res < 0 )
    {
      std::cout << "Failed to bind socket, res: " << res << std::endl;
      return false;
    }

  thread_ = std::unique_ptr<std::thread>( new std::thread( &ClientK79::mainLoop, this ) );
  mutex_.lock();
  is_running_ = true;
  mutex_.unlock();

  // Advertise the K-79 data using the ROS node handle:
  pub_ = node_handle_.advertise<radar_to_rviz::TestPacket>( "k79_data", 10 );
  
  return true;
}

void ClientK79::mainLoop(void)
{
  // Buffer for the received messages:
  int msg_len;
  char buffer[MSG_LEN];

  // Structures to store where the UDP message came from:
  struct sockaddr_storage src_addr;
  socklen_t src_addr_len = sizeof( src_addr );

  // Enter the main data receiving loop:
  bool running = true;
  while( running )
    {
      msg_len = recvfrom( sockfd_, (char* )buffer, MSG_LEN, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );

      // Parse the test packet into 7 uint32 fields and publish to ROS topic:
      radar_to_rviz::TestPacket msg;
      msg.line1 = ClientK79::read_uint32( &( buffer[0] ) );
      msg.line2 = ClientK79::read_uint32( &( buffer[4] ) );
      msg.line3 = ClientK79::read_uint32( &( buffer[8] ) );
      msg.line4 = ClientK79::read_uint32( &( buffer[12] ) );
      msg.line5 = ClientK79::read_uint32( &( buffer[16] ) );
      msg.line6 = ClientK79::read_uint32( &( buffer[20] ) );
      msg.line7 = ClientK79::read_uint32( &( buffer[24] ) );
      
      pub_.publish( msg );
      
      // // Print the test packet data:
      // for( int i = 0; i < msg_len; ++i)
      // 	{
      // 	  std::cout << std::hex << ( int )( buffer[i] );
      // 	}
      // std::cout << std::endl;

      // Check whether the data loop should still be running:
      mutex_.lock();
      running = is_running_;
      mutex_.unlock();  
  
    }

}

uint32_t ClientK79::read_uint32(const char* p)
{
  return uint32_t( ( unsigned char )( p[0] ) << 24 |
		   ( unsigned char )( p[1] ) << 16 |
		   ( unsigned char )( p[2] ) << 8 |
		   ( unsigned char )( p[3] ) );
}
