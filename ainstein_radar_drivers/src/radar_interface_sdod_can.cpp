/*
  Copyright <2022> <Ainstein, Inc.>

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

#include "ainstein_radar_drivers/radar_interface_sdod_can.h"
#include "ainstein_radar_drivers/radar_target_cartesian.h"
#include "ainstein_radar_drivers/types_sdod.h"
#include <tf2_eigen/tf2_eigen.h>
#include <iostream>

namespace ainstein_radar_drivers
{
  // const double RadarInterfaceSDODCAN::msg_range_res = 0.01;
  // const double RadarInterfaceSDODCAN::msg_speed_res = 0.005;
  // const double RadarInterfaceSDODCAN::msg_pos_res = 0.01;
  // const double RadarInterfaceSDODCAN::msg_vel_res = 0.005;

  // const double RadarInterfaceSDODCAN::msg_pos_x_res = 0.025;
  // const double RadarInterfaceSDODCAN::msg_pos_y_res = 0.025;
  // const double RadarInterfaceSDODCAN::msg_pos_z_res = 0.08;
  // const double RadarInterfaceSDODCAN::msg_vel_x_res = 0.025;
  // const double RadarInterfaceSDODCAN::msg_vel_y_res = 0.025;
  // const double RadarInterfaceSDODCAN::msg_vel_z_res = 0.125;

  const int16_t RadarInterfaceSDODCAN::q_type = 7U;

  RadarInterfaceSDODCAN::RadarInterfaceSDODCAN(ros::NodeHandle node_handle,
                                               ros::NodeHandle node_handle_private) : RadarInterface<can_msgs::Frame>(node_handle,
                                                                                                                      node_handle_private,
                                                                                                                      ros::this_node::getName(),
                                                                                                                      "received_messages",
                                                                                                                      "sent_messages"),
                                                                                      radar_info_msg_ptr_(new ainstein_radar_msgs::RadarInfo)
  {
    // Store the radar data frame ID:
    // Convert for RADAC ROS
    nh_private_.param("can_id", can_id_str_, std::string("0x1"));
    nh_private_.param("frame_id", frame_id_, std::string("map"));

    // Convert the CAN ID string to an int:
    can_id_ = std::stoul(can_id_str_, nullptr, 16);

    // Set the frame ID:
    radar_data_msg_ptr_raw_cartesian_->header.frame_id = frame_id_;
    radar_data_msg_ptr_raw_clusters_->header.frame_id = frame_id_;
    radar_data_msg_ptr_tracked_->header.frame_id = frame_id_;

    // Publish the RadarInfo message:
    publishRadarInfo();

    // Set up the CAN frame message:
    can_frame_msg_.header.frame_id = "0";
    can_frame_msg_.is_rtr = false;
    can_frame_msg_.is_extended = false;
    can_frame_msg_.is_error = false;
    can_frame_msg_.dlc = 8;
  }

  int targets_to_come = -1;
  unsigned int targets_received = 0;
  radar_message_type_t target_type = no_type;
  uint8_t object_id = 0;
  uint16_t radarFrameID = 0;

  void RadarInterfaceSDODCAN::dataMsgCallback(const can_msgs::Frame &msg)
  {

    // std::cout << "Message: ";
    // for(uint8_t cnt = 0; cnt < msg.dlc; cnt++)
    // {
    //   std::cout << std::hex << (int)msg.data[cnt];
    //   std::cout << " ";
    // }
    // std::cout << std::endl;

    if (msg.id == can_id_)
    {
      // Parse out start of frame messages:
      radar_message_type_t tmp_target_type = (radar_message_type_t)msg.data[4];

      // std::cout << "Message Received: " << msg << std::endl;

      if ((tmp_target_type == CAN_cartesian_2D_revA_T ||
           tmp_target_type == CAN_raw_cluster_2D_revA_T ||
           tmp_target_type == CAN_tracked_cluster_2D_revA_T) &&
          msg.data[5] == 0xFF && msg.data[6] == 0xFF && msg.data[7] == 0xFF)
      {
        if(targets_received > 0 )
        {
          std::cout << "New header was received prior to receving all objects. Received: " << targets_received << " Expected: " << targets_to_come << std::endl; 
          std::cout << "Message Type was: " << target_type << " Message Count Delta: " << targets_to_come - targets_received << std::endl; 
        }
        radarFrameID = (msg.data[1] << 1) | msg.data[0];
        target_type = tmp_target_type;
        targets_received = 0;
        targets_to_come = (msg.data[3] << 8) | msg.data[2];
        object_id = 0;

        // std::cout << "Message Type: " << target_type << " with NumObjs:" << targets_to_come << std::endl;

        /* Clear Radar Message Data Arrays */
        switch (target_type)
        {
        case CAN_cartesian_2D_revA_T:
        {
          radar_data_msg_ptr_raw_cartesian_->header.stamp = ros::Time::now();
          radar_data_msg_ptr_raw_cartesian_->targets.clear();
          break;
        }

        case CAN_raw_cluster_2D_revA_T:
        {
          radar_data_msg_ptr_raw_clusters_->header.stamp = ros::Time::now();
          radar_data_msg_ptr_raw_clusters_->boxes.clear();
          break;
        }

        case CAN_tracked_cluster_2D_revA_T:
        {
          radar_data_msg_ptr_tracked_->header.stamp = ros::Time::now();
          radar_data_msg_ptr_tracked_->objects.clear();
          break;
        }

        default:
        {
          ROS_DEBUG("Invalid target type received.");
          std::cout << "Invalid target type received: " << target_type << std::endl;
          break;
        }
        }
      }
      // Parse out raw target data messages:
      else if (targets_to_come > 0)
      {
        ROS_DEBUG("received target from radar");
        // std::cout << "Received target from radar: " << target_type << " Targets Received: " << targets_received << std::endl;

        switch (target_type)
        {
        case CAN_cartesian_2D_revA_T:
        {
          /* position message; add a new entry to the vector of targets */
          ainstein_radar_drivers::RadarTargetCartesian tc;
          ainstein_radar_msgs::RadarTargetCartesian tc_out;

          tc.id = object_id;
          tc.pos.x() = static_cast<double>(static_cast<int16_t>((msg.data[1] << 8) + msg.data[0])) / (1 << RadarInterfaceSDODCAN::q_type);
          tc.pos.y() = static_cast<double>(static_cast<int16_t>((msg.data[3] << 8) + msg.data[2])) / (1 << RadarInterfaceSDODCAN::q_type);
          tc.pos.z() = 0U;

          tc.speed = static_cast<double>(static_cast<int16_t>((msg.data[5] << 8) + msg.data[4])) / (1 << RadarInterfaceSDODCAN::q_type);
          tc.peakVal = static_cast<double>(static_cast<int16_t>((msg.data[7] << 8) + msg.data[6])) / (1 << RadarInterfaceSDODCAN::q_type);
          tc.vel.x() = tc.speed * tc.pos.x();
          tc.vel.y() = tc.speed * tc.pos.y();
          tc.vel.z() = tc.speed * tc.pos.z();

          // Fill in the pose information:
          tc_out.id = tc.id;
          tc_out.peakVal = tc.peakVal;
          tc_out.pose = ainstein_radar_filters::data_conversions::posVelToPose(tc.pos, tc.vel);

          // Fill in the velocity information:
          tc_out.velocity.linear.x = tc.vel.x();
          tc_out.velocity.linear.y = tc.vel.y();
          tc_out.velocity.linear.z = tc.vel.z();

          radar_data_msg_ptr_raw_cartesian_->targets.push_back(tc_out);
          object_id++;
          break;
        }

        case CAN_raw_cluster_2D_revA_T:
        {
          ainstein_radar_msgs::BoundingBox box;
          Eigen::Affine3d box_pose;

          // Compute box pose (identity orientation, geometric center is position):
          box_pose.linear() = Eigen::Matrix3d::Identity();
          box_pose.translation().x() = static_cast<double>(static_cast<int16_t>((msg.data[1] << 8) + msg.data[0])) / (1 << RadarInterfaceSDODCAN::q_type);
          box_pose.translation().y() = static_cast<double>(static_cast<int16_t>((msg.data[3] << 8) + msg.data[2])) / (1 << RadarInterfaceSDODCAN::q_type);
          box_pose.translation().z() = (0U);

          // Form the box message:
          box.pose = tf2::toMsg(box_pose);

          box.dimensions.x = static_cast<double>(static_cast<int16_t>((msg.data[5] << 8) + msg.data[4])) / (1 << RadarInterfaceSDODCAN::q_type);
          box.dimensions.y = static_cast<double>(static_cast<int16_t>((msg.data[7] << 8) + msg.data[6])) / (1 << RadarInterfaceSDODCAN::q_type);
          box.dimensions.z = (0U);

          radar_data_msg_ptr_raw_clusters_->boxes.push_back(box);
          break;
        }

        case CAN_tracked_cluster_2D_revA_T:
        {
          ainstein_radar_msgs::RadarTrackedObject object;
          Eigen::Vector3d pos, vel;
          Eigen::Affine3d box_pose;
          
          object.id = object_id;

          pos.x() = static_cast<double>(static_cast<int16_t>((msg.data[1] << 8) + msg.data[0])) / (1 << RadarInterfaceSDODCAN::q_type);
          pos.y() = static_cast<double>(static_cast<int16_t>((msg.data[3] << 8) + msg.data[2])) / (1 << RadarInterfaceSDODCAN::q_type);
          pos.z() = (0U);

          vel.x() = static_cast<double>(static_cast<int16_t>((msg.data[5] << 8) + msg.data[4])) / (1 << RadarInterfaceSDODCAN::q_type);
          vel.y() = static_cast<double>(static_cast<int16_t>((msg.data[7] << 8) + msg.data[6])) / (1 << RadarInterfaceSDODCAN::q_type);
          vel.z() = (0U);

          object.pose = ainstein_radar_filters::data_conversions::posVelToPose(pos, vel);

          object.box.pose = object.pose;
          object.box.dimensions.x = static_cast<double>(static_cast<int16_t>((msg.data[9] << 8) + msg.data[8])) / (1 << RadarInterfaceSDODCAN::q_type);
          object.box.dimensions.y = static_cast<double>(static_cast<int16_t>((msg.data[11] << 8) + msg.data[10])) / (1 << RadarInterfaceSDODCAN::q_type);
          object.box.dimensions.z = (0U);

          object.velocity.linear.x = vel.x();
          object.velocity.linear.y = vel.y();
          object.velocity.linear.z = vel.z();

          radar_data_msg_ptr_tracked_->objects.push_back(object);
          object_id++;
          break;
        }

        default:
        {
          ROS_DEBUG("Invalid target type received.");
          std::cout << "Invalid target type received: " << target_type << std::endl;
          break;
        }
        }
        targets_received++;

        if (targets_received == targets_to_come)
        {
          switch (target_type)
          {
          case CAN_cartesian_2D_revA_T:
          {
            pub_radar_data_raw_cartesian_.publish(radar_data_msg_ptr_raw_cartesian_);
            // std::cout << "Published Raw Cartesians" << std::endl;
            break;
          }

          case CAN_raw_cluster_2D_revA_T:
          {
            pub_radar_data_raw_clusters_.publish(radar_data_msg_ptr_raw_clusters_);
            // std::cout << "Published Raw Clusters" << std::endl;
            break;
          }

          case CAN_tracked_cluster_2D_revA_T:
          {
            pub_radar_data_tracked_.publish(radar_data_msg_ptr_tracked_);
            // std::cout << "Published Tracked Clusters" << std::endl;
            break;
          }

          default:
          {
            ROS_DEBUG("Invalid target type received.");
            std::cout << "Invalid target type received: " << target_type << std::endl;
            break;
          }
          }
          targets_received = 0;
          targets_to_come = -1;
          target_type = no_type;
        }
      }
      else
      {
        std::cout << "Message Received but not detected as object" << std::endl;
      }
    }
  }

  void RadarInterfaceSDODCAN::publishRadarInfo(void)
  {
    // Advertise the SDOD sensor info (LATCHED):
#if 1
    pub_radar_info_ = nh_private_.advertise<ainstein_radar_msgs::RadarInfo>("radar_info", 10, true);

    // Form the RadarInfo message which is fixed for a given sensor:
    radar_info_msg_ptr_->header.stamp = ros::Time::now();
    radar_info_msg_ptr_->header.frame_id = frame_id_;

    radar_info_msg_ptr_->update_rate = UPDATE_RATE;
    radar_info_msg_ptr_->max_num_targets = MAX_NUM_TARGETS;

    radar_info_msg_ptr_->range_min = RANGE_MIN;
    radar_info_msg_ptr_->range_max = RANGE_MAX;

    radar_info_msg_ptr_->speed_min = SPEED_MIN;
    radar_info_msg_ptr_->speed_max = SPEED_MAX;

    radar_info_msg_ptr_->azimuth_min = AZIMUTH_MIN;
    radar_info_msg_ptr_->azimuth_max = AZIMUTH_MAX;

    radar_info_msg_ptr_->elevation_min = ELEVATION_MIN;
    radar_info_msg_ptr_->elevation_max = ELEVATION_MAX;

    radar_info_msg_ptr_->range_resolution = RANGE_RES;
    radar_info_msg_ptr_->range_accuracy = RANGE_ACC;

    radar_info_msg_ptr_->speed_resolution = SPEED_RES;
    radar_info_msg_ptr_->speed_accuracy = SPEED_ACC;

    radar_info_msg_ptr_->azimuth_resolution = AZIMUTH_RES;
    radar_info_msg_ptr_->azimuth_accuracy = AZIMUTH_ACC;

    radar_info_msg_ptr_->elevation_resolution = ELEVATION_RES;
    radar_info_msg_ptr_->elevation_accuracy = ELEVATION_ACC;

    // Publish the RadarInfo message once since it's latched:
    pub_radar_info_.publish(radar_info_msg_ptr_);
#endif
  }

} // namespace ainstein_drivers
