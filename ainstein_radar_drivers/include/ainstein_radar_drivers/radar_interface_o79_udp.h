#ifndef RADAR_INTERFACE_O79_UDP_H_
#define RADAR_INTERFACE_O79_UDP_H_

#include <netinet/in.h>
#include <string>
#include <mutex>
#include <memory>
#include <thread>

#include <ainstein_radar_msgs/RadarInfo.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_msgs/BoundingBoxArray.h>
#include <ainstein_radar_drivers/radar_driver_o79_udp.h>
#include <ainstein_radar_filters/data_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

namespace ainstein_radar_drivers
{

class RadarInterfaceO79UDP {

public:
  RadarInterfaceO79UDP( ros::NodeHandle node_Handle,
		     ros::NodeHandle node_Handle_private );
  ~RadarInterfaceO79UDP();

  void mainLoop( void );
  ainstein_radar_msgs::RadarTarget targetToROSMsg( const ainstein_radar_drivers::RadarTarget &t )
  {
    ainstein_radar_msgs::RadarTarget target;
    target.target_id = t.id;
    target.range = t.range;
    target.speed = t.speed;
    target.azimuth = t.azimuth;
    target.elevation = t.elevation;
    target.snr = t.snr;

    return target;
  }

  ainstein_radar_msgs::BoundingBox boundingBoxToROSMsg( const ainstein_radar_drivers::BoundingBox &b, std::string frame_id_str )
  {
    ainstein_radar_msgs::BoundingBox box;
    box.header.frame_id = frame_id_str;
    box.pose = tf2::toMsg( b.pose );
    box.dimensions.x = b.dimensions.x();
    box.dimensions.y = b.dimensions.y();
    box.dimensions.z = b.dimensions.z();

    return box;
  }
  
  // Radar specifications:
  static constexpr double UPDATE_RATE = 10.0;
  static constexpr int MAX_NUM_TARGETS = 1000;

  static constexpr double RANGE_MIN = 0.0;
  static constexpr double RANGE_MAX = 120.0;

  static constexpr double SPEED_MIN = 0.0;
  static constexpr double SPEED_MAX = 0.0;

  static constexpr double AZIMUTH_MIN = -40.0;
  static constexpr double AZIMUTH_MAX = 40.0;

  static constexpr double ELEVATION_MIN = -4.0;
  static constexpr double ELEVATION_MAX = 4.0;

  static constexpr double RANGE_RES = 0.1;
  static constexpr double RANGE_ACC = 0.1;

  static constexpr double SPEED_RES = 0.16;
  static constexpr double SPEED_ACC = 0.1;

  static constexpr double AZIMUTH_RES = 2.5;
  static constexpr double AZIMUTH_ACC = 1.0;

  static constexpr double ELEVATION_RES = 0.0;
  static constexpr double ELEVATION_ACC = 0.0;
  
private:

  void publishRadarInfo( void );
  
  std::string frame_id_;
  bool publish_raw_cloud_;
  bool publish_tracked_cloud_;
  
  std::unique_ptr<ainstein_radar_drivers::RadarDriverO79UDP> driver_;
  
  bool is_running_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_radar_data_raw_;
  ros::Publisher pub_radar_data_tracked_;
  ros::Publisher pub_cloud_raw_;
  ros::Publisher pub_cloud_tracked_;
  ros::Publisher pub_radar_info_;
  ros::Publisher pub_bounding_boxes_;
  ros::Publisher pub_tracked_targets_cart_;
  
  boost::shared_ptr<ainstein_radar_msgs::RadarTargetArray> radar_data_msg_ptr_raw_;      
  boost::shared_ptr<ainstein_radar_msgs::RadarTargetArray> radar_data_msg_ptr_tracked_;
  boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg_ptr_raw_;
  boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg_ptr_tracked_;
  boost::shared_ptr<ainstein_radar_msgs::RadarInfo> radar_info_msg_ptr_;
  boost::shared_ptr<ainstein_radar_msgs::BoundingBoxArray> msg_ptr_tracked_boxes_;
  boost::shared_ptr<geometry_msgs::PoseArray> msg_ptr_tracked_targets_cart_;
};

} // namespace ainstein_radar_drivers

#endif // RADAR_INTERFACE_O79_UDP_H_
