#ifndef RADAR_INTERFACE_K79_H_
#define RADAR_INTERFACE_K79_H_

#include <netinet/in.h>
#include <string>
#include <mutex>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_drivers/radar_driver_k79.h>

namespace ainstein_radar_drivers
{

class RadarInterfaceK79 {

public:
  RadarInterfaceK79( ros::NodeHandle node_Handle,
		     ros::NodeHandle node_Handle_private );
  ~RadarInterfaceK79();

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
  
private:
  std::string frame_id_;

  std::unique_ptr<ainstein_radar_drivers::RadarDriverK79> driver_;
  
  bool is_running_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_radar_data_raw_;

  boost::shared_ptr<ainstein_radar_msgs::RadarTargetArray> radar_data_msg_ptr_raw_;      
};

} // namespace ainstein_radar_drivers

#endif // RADAR_INTERFACE_K79_H_
