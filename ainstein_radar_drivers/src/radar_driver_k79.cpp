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
#include <cstring>
#include <cerrno>

#include "ainstein_radar_drivers/radar_driver_k79.h"

namespace ainstein_radar_drivers
{

  const std::string RadarDriverK79::connect_cmd_str = std::string( "connect" );
  const unsigned int RadarDriverK79::connect_res_len = 18;
  
  const std::string RadarDriverK79::run_cmd_str = std::string( "run" );

  const unsigned int RadarDriverK79::radar_msg_len = 3000; // maximum length in bytes
  const unsigned int RadarDriverK79::target_msg_len = 8; // 8 bytes per target, first 4 are nonzero

  RadarDriverK79::RadarDriverK79( std::string host_ip_address, int host_port,
		  std::string radar_ip_address, int radar_port ) :
    host_ip_addr_( host_ip_address ),
    host_port_( host_port ),
    radar_ip_addr_( radar_ip_address ),
    radar_port_( radar_port )
  {
    buffer_ = static_cast<char*>( malloc( RadarDriverK79::radar_msg_len * sizeof( char ) ) );
  }

  RadarDriverK79::~RadarDriverK79(void)
  {
    close( sockfd_ );
    free( buffer_ );
  }

  bool RadarDriverK79::connect(void)
  {
    // Create the host UDP socket:
    sockfd_ = socket( AF_INET, SOCK_DGRAM, 0 );
    if( sockfd_ < 0 )
      {
	std::cout << "Failed to create socket." << std::endl;
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
    int res = setsockopt( sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof( reuseaddr ) );
    if( res < 0 )
      {
	std::cout << "Failed to set socket options: " << std::strerror( errno ) << std::endl;
	return false;
      }

    // Set socket timeout:
    struct timeval tv;
    tv.tv_sec = 3; // 3 second timeout
    tv.tv_usec = 0;
    res  = setsockopt( sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( tv ) );
    if( res < 0 )
      {
	std::cout << "Failed to set socket timeout: " << std::strerror( errno ) << std::endl;
	return false;
      }
    
    // Explicitly bind the host UDP socket:
    res = bind( sockfd_, ( struct sockaddr * )( &sockaddr_ ), sizeof( sockaddr_ ) );
    if( res < 0 )
      {
	std::cout << "Failed to bind socket: " << std::strerror( errno ) << std::endl;
	return false;
      }

    // Try to receive data until the timeout to see if the K79 is already running:
    struct sockaddr_storage src_addr;
    socklen_t src_addr_len = sizeof( src_addr );
    res = recvfrom( sockfd_, ( char* )buffer_, RadarDriverK79::radar_msg_len, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );
    if( res < 0 )
      {
	// If blocking recvfrom times out, errno is set to EAGAIN:
	if( errno == EAGAIN )
	  {
	    // Send the connect command to the radar:
	    RadarDriverK79::connect_cmd_str.copy( buffer_, RadarDriverK79::connect_cmd_str.length() );
	    res = sendto( sockfd_, ( char* )buffer_, RadarDriverK79::connect_cmd_str.length(), 0, ( struct sockaddr *)( &destaddr_ ), sizeof( destaddr_ ) );
	    if( res < 0 )
	      {
		std::cout << "Failed to send connect command to radar: " << std::strerror( errno ) << std::endl;
		return false;
	      }

	    // Wait for a response to the connect command:
	    res = recvfrom( sockfd_, ( char* )buffer_, RadarDriverK79::connect_res_len, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );
	    if( res < 0 )
	      {
		std::cout << "Failed to receive connect response from radar: " << std::strerror( errno ) << std::endl;
		return false;
	      }
	    else
	      {
		// TODO: print settings stored on radar from connect response message
	      }

	    // Send the run command to the radar:
	    RadarDriverK79::run_cmd_str.copy( buffer_, RadarDriverK79::run_cmd_str.length() );
	    res = sendto( sockfd_, ( char* )buffer_, RadarDriverK79::run_cmd_str.length(), 0, ( struct sockaddr *)( &destaddr_ ), sizeof( destaddr_ ) );
	    if( res < 0 )
	      {
		std::cout << "Failed to send run command to radar: " << std::strerror( errno ) << std::endl;
		return false;
	      }
	  }
	else // encountered some error other than timeout
	  {
	    std::cout << "Failed when attempting to detect whether radar is running: " << std::strerror( errno ) << std::endl;
	    return false;
	  }
      }
  
    return true;
  }

  bool RadarDriverK79::receiveTargets( std::vector<ainstein_radar_drivers::RadarTarget> &targets )
  {
    // Clear the targets array in preparation for message processing:
    targets.clear();
    
    // Received message length:
    int msg_len;

    // Structures to store where the UDP message came from:
    struct sockaddr_storage src_addr;
    socklen_t src_addr_len = sizeof( src_addr );

    // Call to block until data has been received:
    msg_len = recvfrom( sockfd_, ( char* )buffer_, RadarDriverK79::radar_msg_len, MSG_WAITALL, ( struct sockaddr *)( &src_addr ), &src_addr_len );

    if( msg_len < 0 )
      {
	std::cout << "Failed to read data: " << std::strerror( errno ) << std::endl;
	return false;
      }
    else
      {
	// Extract the sender's IP address:
	struct sockaddr_in* sin = ( struct sockaddr_in* )&src_addr;
	unsigned char* src_ip = ( unsigned char* )( &sin->sin_addr.s_addr );
	// printf("source IP: %d.%d.%d.%d\n", src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
      
	// Extract the target ID and data from the message:
	if( ( msg_len % RadarDriverK79::target_msg_len ) != 0 )
	  {
	    std::cout << "WARNING >> Incorrect number of bytes: " << msg_len << std::endl;
	    return false;
	  }
	else
	  {
	    int offset;
	    ainstein_radar_drivers::RadarTarget target;
	    for( int i = 0; i < ( msg_len / RadarDriverK79::target_msg_len ); ++i )
	      {
		offset = i * RadarDriverK79::target_msg_len;

		target.id = i;
		target.azimuth = static_cast<uint8_t>( buffer_[offset + 0] ) * -1.0 + 90.0; // 1 count = 1 deg, 90 deg offset
		target.range = static_cast<uint8_t>( buffer_[offset + 2] ) * 0.116;   // 1 count = 0.1 m

		// Speed is 0-127, with 0-64 negative (moving away) and 65-127 positive (moving towards).
		// Note that 65 is the highest speed moving towards, hence the manipulation below.
		if( static_cast<uint8_t>( buffer_[offset + 3] ) <= 64 ) // MOVING AWAY FROM RADAR
		  {
		    target.speed = static_cast<uint8_t>( buffer_[offset + 3] ) * 0.045; // 1 count = 0.045 m/s
		  }
		else // MOVING TOWARDS RADAR
		  {
		    target.speed = ( static_cast<uint8_t>( buffer_[offset + 3] ) - 127 ) * 0.045; // 1 count = 0.045 m/s
		  }
	
		target.elevation = 0.0; // K79 does not output elevation angle
		target.snr =  static_cast<double>( static_cast<uint16_t>( ( buffer_[offset + 7] & 0xff ) << 8 ) | static_cast<uint16_t>( buffer_[offset + 6] & 0xff ) );

		targets.push_back( target );
	      }
	  }
      }

    return true;
  }

} // namespace ainstein_radar_drivers
