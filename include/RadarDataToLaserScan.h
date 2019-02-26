/*
  Copyright <2019> <Ainstein, Inc.>

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

#ifndef RADAR_DATA_TO_LASER_SCAN_H_
#define RADAR_DATA_TO_LASER_SCAN_H_

#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <radar_sensor_msgs/RadarData.h>

class RadarDataToLaserScan
{
public:
  RadarDataToLaserScan( std::string data_topic, std::string vel_topic, std::string laser_scan_topic );
  ~RadarDataToLaserScan(){}

  void radarVelCallback( const geometry_msgs::Twist &msg );     
  void radarDataCallback( const radar_sensor_msgs::RadarData &msg );

  bool useTarget( const radar_sensor_msgs::RadarTarget &t );
  
private:
  std::string data_topic_;
  std::string vel_topic_;
  std::string laser_scan_topic_;
  std::string frame_id_;

  int laser_scan_length_;
  sensor_msgs::LaserScan laser_scan_msg_;
    
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_radar_data_;
  ros::Publisher pub_laser_scan_;

  ros::Subscriber sub_radar_vel_;
  bool is_vel_available_;
  Eigen::Vector3d vel_world_;
  double rel_speed_thresh_;
  double min_dist_thresh_;
  double max_dist_thresh_;
  
  tf2_ros::TransformListener listen_tf_;
  tf2_ros::Buffer buffer_tf_;

};

#endif
