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

int main( int argc, char** argv )
{
  // Initialize ROS and the default node name:
  ros::init( argc, argv, "t79_bsd_node" );
  
  // Radar node constructor arguments:
  ConfigT79BSD::RadarType radar_type = ConfigT79BSD::TIPI_79_FL;
  std::string radar_name = "t79_bsd";
  std::string frame_id = "map";
  
  // Parse the command line arguments for radar parameters:
  if( argc < 1 )
    {
      std::cerr << "Usage: rosrun radar_ros_interface t79_bsd_node [--type RADAR_TYPE] [--name RADAR_NAME] [--frame RADAR_FRAME_ID]" << std::endl;
      return -1;
    }

  // Parse the command line arguments:
  for( int i = 0; i < argc; ++i )
    {
      // Check for the radar type:
      if( std::string( argv[i] ) == std::string( "--type" ) )
	{
	  radar_type = static_cast<ConfigT79BSD::RadarType>( atoi( argv[++i] ) );
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

  std::cout << "Running T79 BSD node with radar type: " << ConfigT79BSD::radar_names.at( radar_type ) << " name: " << radar_name << " frame: " << frame_id << std::endl;

  // Create the K79 interface and launch the data thread:
  RadarInterfaceT79BSD t79_bsd_intf( radar_type, radar_name, frame_id );
  
  ros::spin();

  return 0;
}
