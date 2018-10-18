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

#include "RadarNodeT79BSD.h"
#include "RadarDataViz.h"

#include <can_msgs/Frame.h>

int main( int argc, char** argv )
{
    // Initialize ROS node:
    ros::init( argc, argv, "radar_ros_interface_test_node" );

    // Create the T-79 Nodes for receiving CAN messages and publishing radar data messages:
    RadarNodeT79BSD radar_node_kanza( ConfigT79BSD::KANZA, "kanza_front",
                                      "chassis" );
    RadarNodeT79BSD radar_node_FL( ConfigT79BSD::TIPI_79_FL, "tipi_79_bsd_front_left",
                                   "chassis" );
    RadarNodeT79BSD radar_node_FR( ConfigT79BSD::TIPI_79_FR, "tipi_79_bsd_front_right",
                                   "chassis" );
    RadarNodeT79BSD radar_node_RL( ConfigT79BSD::TIPI_79_RL, "tipi_79_bsd_rear_left",
                                   "chassis" );
    RadarNodeT79BSD radar_node_RR( ConfigT79BSD::TIPI_79_RR, "tipi_79_bsd_rear_right",
                                   "chassis" );

    // Create visualization nodes to publish target markers:
    RadarDataViz data_viz_node_kanza( "kanza_front_data", "KANZA_77_FL_TARGETS" );
    RadarDataViz data_viz_node_FL( "tipi_79_bsd_front_left_data", "TIPI_79_FL_TARGETS" );
    RadarDataViz data_viz_node_FR( "tipi_79_bsd_front_right_data", "TIPI_79_FR_TARGETS" );
    RadarDataViz data_viz_node_RL( "tipi_79_bsd_rear_left_data", "TIPI_79_RL_TARGETS" );
    RadarDataViz data_viz_node_RR( "tipi_79_bsd_rear_right_data", "TIPI_79_RR_TARGETS" );

    ros::spin();

    return 0;
}
