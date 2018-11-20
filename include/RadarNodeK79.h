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

#ifndef RADAR_NODE_K79_H_
#define RADAR_NODE_K79_H_

#include <netinet/in.h>
#include <string>
#include <mutex>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <radar_sensor_msgs/RadarData.h>

#define MSG_LEN        1000 // maximum length in bytes
#define TARGET_MSG_LEN    8 // 8 bytes per target, first 4 are nonzero

class RadarNodeK79 {

public:
  RadarNodeK79( std::string ip_addr, int port, std::string radar_name, std::string frame_id );
  ~RadarNodeK79();

  bool connect( void );
  void mainLoop( void );

private:

  std::string radar_name_;
  std::string ip_addr_;
  int port_;

  int sockfd_; // socket file descriptor
  struct sockaddr_in sockaddr_;
  char buffer_[MSG_LEN];

  bool is_running_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_;

  ros::NodeHandle node_handle_;
  ros::Publisher pub_radar_data_;

  radar_sensor_msgs::RadarData radar_data_msg_;
    
};

#endif
