#ifndef RADAR_INTERFACE_K79_3D_H_
#define RADAR_INTERFACE_K79_3D_H_

#include <netinet/in.h>
#include <string>
#include <mutex>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_drivers
{

class RadarInterfaceK793D {

public:
  RadarInterfaceK793D( ros::NodeHandle node_Handle,
		       ros::NodeHandle node_Handle_private );
  ~RadarInterfaceK793D();

  bool connect( void );
  void mainLoop( void );
  
  static const std::string connect_cmd_str;
  static const unsigned int connect_res_len;

  static const std::string run_cmd_str;

  #define RADAR_MSG_LEN  3000    // maximum length in bytes
  #define TARGET_MSG_LEN 8       // 8 bytes per target, first 4 are nonzero

  static const unsigned int radar_msg_len;
  static const unsigned int target_msg_len;
  
private:
  std::string host_ip_addr_;
  int host_port_;

  std::string radar_name_;
  std::string radar_ip_addr_;
  int radar_port_;
  std::string frame_id_;

  int sockfd_; // socket file descriptor
  struct sockaddr_in sockaddr_;
  char buffer_[RADAR_MSG_LEN];

  struct sockaddr_in destaddr_;
  
  bool is_running_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_radar_data_raw_;

  boost::shared_ptr<ainstein_radar_msgs::RadarTargetArray> radar_data_msg_ptr_raw_;      
};

} // namespace ainstein_radar_drivers

#endif // RADAR_INTERFACE_K79_3D_H_
