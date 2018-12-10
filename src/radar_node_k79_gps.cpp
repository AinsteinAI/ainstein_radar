/*
Copyright <2018> <Ainstein, Inc.>

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

#include "RadarNodeK79GPS.h"
#include <signal.h>

int main( int argc, char** argv )
{
  // Initialize ROS and the default node name:
  ros::init( argc, argv, "radar_node_k79_gps" );
  
  // Radar node constructor arguments:
  std::string ip_address;
  int port = 8;
  std::string radar_name = "k79";
  std::string frame_id = "k79_link";
  
  // Parse the command line arguments for radar parameters:
  if( argc < 2 )
    {
      std::cerr << "Usage: rosrun radar_ros_interface radar_node_k79 --ip IP_ADDRESS [--port UDP_PORT] [--name RADAR_NAME] [--frame RADAR_FRAME_ID]" << std::endl;
      return -1;
    }

  // Parse the command line arguments:
  for( int i = 0; i < argc; ++i )
    {
      // Check for the radar IP address:
      if( std::string( argv[i] ) == std::string( "--ip" ) )
	{
	  ip_address = std::string( argv[++i] );
	}
      // Check for the radar UDP port:
      else if( std::string( argv[i] ) == std::string( "--port" ) )
	{
	  port = atoi( argv[++i] );
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

  if( ip_address.empty() )
    {
      std::cerr << "IP address must be set. Usage: rosrun radar_ros_interface radar_node_k79 --ip IP_ADDRESS [--name RADAR_NAME] [--frame RADAR_FRAME_ID]" << std::endl;
      return -1;
    }

  std::cout << "Running K79 GPS node with ip: " << ip_address << " port: " << port << " name: " << radar_name << " frame: " << frame_id << std::endl;

  // Create the K79 interface and launch the data thread:
  RadarNodeK79GPS k79_gps( ip_address, port, radar_name, frame_id );
  k79_gps.connect();
  
  ros::spin();

  return 0;
}
