#ifndef RADAR_INTERFACE_K79_H_
#define RADAR_INTERFACE_K79_H_

#include <netinet/in.h>
#include <string>
#include <mutex>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <radar_sensor_msgs/RadarData.h>

class RadarInterfaceK79 {

public:
  RadarInterfaceK79( ros::NodeHandle node_Handle,
		     ros::NodeHandle node_Handle_private );
  ~RadarInterfaceK79();

  bool connect( void );
  void mainLoop( void );
  
  static const std::string connect_cmd_str;
  static const unsigned int connect_res_len;

  static const std::string run_cmd_str;

  #define MSG_LEN        1000 // maximum length in bytes
  #define TARGET_MSG_LEN    8 // 8 bytes per target, first 4 are nonzero

  static const unsigned int radar_msg_len;
  static const unsigned int target_msg_len;
  
private:
  std::string host_ip_addr_;
  int host_port_;

  std::string radar_name_;
  std::string radar_ip_addr_;
  int radar_port_;

  int sockfd_; // socket file descriptor
  struct sockaddr_in sockaddr_;
  char buffer_[MSG_LEN];

  struct sockaddr_in destaddr_;
  
  bool is_running_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_radar_data_;

  radar_sensor_msgs::RadarData radar_data_msg_;
    
};

#endif
