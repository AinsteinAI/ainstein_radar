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

#ifndef RADAR_NODE_H_
#define RADAR_NODE_H_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <radar_ros_interface/RadarData.h>

// Generic class from which all RADAR sensor ROS interface nodes should derive.
// Example:
//    RadarNode<can_msgs::Frame> radar_node( "tipi_79_bsd_front_left", "received_messages", "sent_messages" );
//
// The user must implement the startRadar, stopRadar and dataMsgCallback functions.
template<typename data_msg_type>
class RadarNode
{

public:

    RadarNode( std::string radar_name, std::string data_msg_topic, std::string radar_cmd_topic ) :
            name_( radar_name )
    {
        // Set up the subscriber to receive radar data:
        sub_data_msg_ = node_handle_.subscribe( data_msg_topic, 10,
                                                &RadarNode::dataMsgCallback,
                                                this );

        // Set up the publisher for sending commands to the radar:
        pub_radar_cmd_ = node_handle_.advertise<data_msg_type>( radar_cmd_topic,
                                                                10 );

        // Set up the publisher for sending out processed radar data:
        pub_radar_data_ = node_handle_.advertise<radar_ros_interface::RadarData>( radar_name + "_data",
                                                                            10 );

        // Sleep for a little to make sure messages are being advertised before we start sending:
        ros::Duration( 1.0 ).sleep();

    }
    virtual ~RadarNode( void )
    {
    }

    virtual void startRadar( void ) = 0;

    virtual void stopRadar( void ) = 0;

protected:

    std::string name_;

    virtual void dataMsgCallback( const data_msg_type& data_msg ) = 0;

    ros::NodeHandle node_handle_;
    ros::Publisher pub_radar_cmd_;
    ros::Publisher pub_radar_data_;

    ros::Subscriber sub_data_msg_;

    radar_ros_interface::RadarData radar_data_msg_;

};

#endif // RADAR_NODE_H_
