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

#include "radar_ros_interface/radar_interface_k79.h"

int main( int argc, char** argv )
{
  // Initialize ROS and the default node name:
  ros::init( argc, argv, "k79_node" );
  
  // Radar node constructor arguments:
  std::string host_ip_address;
  int host_port = 1024;
  std::string radar_ip_address;
  int radar_port = 8;
  std::string radar_name = "k79";
  std::string frame_id = "map";
  
  // Parse the command line arguments for radar parameters:
  if( argc < 2 )
    {
      std::cerr << "Usage: rosrun radar_ros_interface k79_node --host-ip HOST_IP_ADDRESS --radar-ip RADAR_IP_ADDRESS [--radar-port RADAR_UDP_PORT] [--host-port HOST_UDP_PORT] [--name RADAR_NAME] [--frame RADAR_FRAME_ID]" << std::endl;
      return -1;
    }

  // Parse the command line arguments:
  for( int i = 0; i < argc; ++i )
    {
      // Check for the host IP address:
      if( std::string( argv[i] ) == std::string( "--host-ip" ) )
	{
	  host_ip_address = std::string( argv[++i] );
	}
      // Check for the host UDP port:
      else if( std::string( argv[i] ) == std::string( "--host-port" ) )
	{
	  host_port = atoi( argv[++i] );
	}
      // Check for the radar IP address:
      if( std::string( argv[i] ) == std::string( "--radar-ip" ) )
	{
	  radar_ip_address = std::string( argv[++i] );
	}
      // Check for the radar UDP port:
      else if( std::string( argv[i] ) == std::string( "--radar-port" ) )
	{
	  radar_port = atoi( argv[++i] );
	}
      // Check for the radar name:
      else if( std::string( argv[i] ) == std::string( "--name" ) )
	{
	  radar_name = std::string( argv[++i] );
	}
      // Check for the radar frame ID:
      else if( std::string( argv[i] ) == std::string( "--frame" ) )
	{
	  frame_id = std::string( argv[++i] );
	}
    }

  if( host_ip_address.empty() || host_ip_address.empty() )
    {
      std::cerr << "IP addresses must be set. Usage: rosrun radar_ros_interface k79_node --host-ip HOST_IP_ADDRESS --radar-ip RADAR_IP_ADDRESS [--radar-port RADAR_UDP_PORT] [--host-port HOST_UDP_PORT] [--name RADAR_NAME] [--frame RADAR_FRAME_ID]" << std::endl;
      return -1;
    }

  std::cout << "Running K79 node with host IP: " << host_ip_address << " host port: " << host_port << " radar IP: " << radar_ip_address << " radar port: " << radar_port << " name: " << radar_name << " frame: " << frame_id << std::endl;

  // Create the K79 interface and launch the data thread:
  RadarInterfaceK79 k79_intf( host_ip_address, host_port, radar_ip_address, radar_port, radar_name, frame_id );
  k79_intf.connect();
  
  ros::spin();

  return 0;
}
