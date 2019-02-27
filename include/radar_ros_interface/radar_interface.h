#ifndef RADAR_INTERFACE_H_
#define RADAR_INTERFACE_H_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <radar_sensor_msgs/RadarData.h>

// Generic class from which all RADAR sensor ROS interface interfaces should derive.
// Example:
//    RadarInterface<can_msgs::Frame> radar_interface( "tipi_79_bsd_front_left", "received_messages", "sent_messages" );
//
// The user must implement the startRadar, stopRadar and dataMsgCallback functions.
template<typename data_msg_type>
class RadarInterface
{

public:

    RadarInterface( std::string radar_name, std::string data_msg_topic, std::string radar_cmd_topic ) :
            name_( radar_name )
    {
        // Set up the subscriber to receive radar data:
        sub_data_msg_ = node_handle_.subscribe( data_msg_topic, 10,
                                                &RadarInterface::dataMsgCallback,
                                                this );

        // Set up the publisher for sending commands to the radar:
        pub_radar_cmd_ = node_handle_.advertise<data_msg_type>( radar_cmd_topic,
                                                                10 );

        // Set up the publisher for sending out processed radar data:
        pub_radar_data_ = node_handle_.advertise<radar_sensor_msgs::RadarData>( radar_name + "_data",
                                                                            10 );

        // Sleep for a little to make sure messages are being advertised before we start sending:
        ros::Duration( 1.0 ).sleep();

    }
    virtual ~RadarInterface( void )
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

    radar_sensor_msgs::RadarData radar_data_msg_;

};

#endif // RADAR_INTERFACE_H_
