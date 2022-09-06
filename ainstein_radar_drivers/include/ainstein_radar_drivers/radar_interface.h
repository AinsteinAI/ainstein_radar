#ifndef RADAR_INTERFACE_H_
#define RADAR_INTERFACE_H_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_msgs/RadarTrackedObjectArray.h>
#include <ainstein_radar_msgs/RadarAlarmArray.h>
#include <ainstein_radar_msgs/BoundingBoxArray.h>
#include <ainstein_radar_msgs/RadarTargetCartesianArray.h>
#include <geometry_msgs/PoseArray.h>
#include <ainstein_radar_msgs/TwistArray.h>

namespace ainstein_radar_drivers
{

// Generic class from which all RADAR sensor ROS interface interfaces should derive.
// Example:
//    RadarInterface<can_msgs::Frame> radar_interface( "tipi_79_bsd_front_left", "received_messages", "sent_messages" );
//
// The user must implement the startRadar, stopRadar and dataMsgCallback functions.
template<typename data_msg_type>
class RadarInterface
{

public:

 RadarInterface( ros::NodeHandle node_handle,
		 ros::NodeHandle node_handle_private,
		 std::string radar_name,
		 std::string data_msg_topic,
		 std::string radar_cmd_topic ) :
  nh_( node_handle ),
    nh_private_( node_handle_private ),
    name_( radar_name ),
    radar_data_msg_ptr_raw_( new ainstein_radar_msgs::RadarTargetArray ),
    radar_data_msg_ptr_raw_cartesian_( new ainstein_radar_msgs::RadarTargetCartesianArray),
    radar_data_msg_ptr_filtered_pcl_( new ainstein_radar_msgs::RadarTargetArray ),
    radar_data_msg_ptr_ground_( new ainstein_radar_msgs::RadarTrackedObjectArray ),
    radar_data_msg_ptr_raw_clusters_( new ainstein_radar_msgs::BoundingBoxArray ),
    radar_data_msg_ptr_tracked_( new ainstein_radar_msgs::RadarTrackedObjectArray ),
    radar_data_msg_ptr_alarms_( new ainstein_radar_msgs::RadarAlarmArray )
    {
        // Set up the subscriber to receive radar data:
        sub_data_msg_ = nh_.subscribe( data_msg_topic, 50,
				       &RadarInterface::dataMsgCallback,
				       this );

        // Set up the publisher for sending commands to the radar:
        pub_radar_cmd_ = nh_.advertise<data_msg_type>( radar_cmd_topic,
						       10 );

        // Set up the publishers for sending out processed radar data:
        pub_radar_data_raw_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "targets/raw",
											10 );
        pub_radar_data_raw_cartesian_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetCartesianArray>( "targets/raw_cartesian",
											10 );
        pub_radar_data_filtered_pcl_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "targets/filtered",
											10 );
        pub_radar_data_raw_clusters_ = nh_private_.advertise<ainstein_radar_msgs::BoundingBoxArray>( "targets/raw_clusters",
            10 );
        pub_radar_data_tracked_ = nh_private_.advertise<ainstein_radar_msgs::RadarTrackedObjectArray>( "objects",
												10 );
        pub_radar_data_ground_ = nh_private_.advertise<ainstein_radar_msgs::RadarTrackedObjectArray>( "ground",
        10 );
        pub_radar_data_alarms_ = nh_private_.advertise<ainstein_radar_msgs::RadarAlarmArray>( "alarms",
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

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pub_radar_cmd_;
    ros::Publisher pub_radar_data_raw_;
    ros::Publisher pub_radar_data_raw_cartesian_;
    ros::Publisher pub_radar_data_filtered_pcl_;
    ros::Publisher pub_radar_data_raw_clusters_;
    ros::Publisher pub_radar_data_tracked_;
    ros::Publisher pub_radar_data_ground_;
    ros::Publisher pub_radar_data_alarms_;

    ros::Subscriber sub_data_msg_;

    boost::shared_ptr<ainstein_radar_msgs::RadarTargetArray> radar_data_msg_ptr_raw_;
    boost::shared_ptr<ainstein_radar_msgs::RadarTargetCartesianArray> radar_data_msg_ptr_raw_cartesian_;
    boost::shared_ptr<ainstein_radar_msgs::RadarTargetArray> radar_data_msg_ptr_filtered_pcl_;
    boost::shared_ptr<ainstein_radar_msgs::BoundingBoxArray> radar_data_msg_ptr_raw_clusters_;
    boost::shared_ptr<ainstein_radar_msgs::RadarTrackedObjectArray> radar_data_msg_ptr_tracked_;
    boost::shared_ptr<ainstein_radar_msgs::RadarTrackedObjectArray> radar_data_msg_ptr_ground_;
    boost::shared_ptr<ainstein_radar_msgs::RadarAlarmArray> radar_data_msg_ptr_alarms_;
};

} // namespace ainstein_radar_drivers

#endif // RADAR_INTERFACE_H_
