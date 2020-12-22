/*
  Copyright <2020> <Ainstein, Inc.>

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

#include "ainstein_radar_drivers/radar_interface_o79_can.h"
#include  "ainstein_radar_drivers/radar_target_cartesian.h"
#include <tf2_eigen/tf2_eigen.h>

namespace ainstein_radar_drivers
{

  const double RadarInterfaceO79CAN::msg_range_res = 0.01;
  const double RadarInterfaceO79CAN::msg_speed_res = 0.005;
  const double RadarInterfaceO79CAN::msg_pos_res = 0.01;
  const double RadarInterfaceO79CAN::msg_vel_res = 0.005;

  RadarInterfaceO79CAN::RadarInterfaceO79CAN( ros::NodeHandle node_handle,
        ros::NodeHandle node_handle_private ) :
    RadarInterface<can_msgs::Frame>( node_handle,
       node_handle_private,
       ros::this_node::getName(),
       "received_messages",
       "sent_messages" ),
    radar_info_msg_ptr_( new ainstein_radar_msgs::RadarInfo )
  {
    // Store the radar data frame ID:
    nh_private_.param( "can_id", can_id_str_, std::string( "0x18FFB24D" ) );
    nh_private_.param( "frame_id", frame_id_, std::string( "map" ) );

    // Convert the CAN ID string to an int:
    can_id_ = std::stoul( can_id_str_, nullptr, 16 );

    // Set the frame ID:
    radar_data_msg_ptr_raw_->header.frame_id = frame_id_;
    radar_data_msg_ptr_tracked_->header.frame_id = frame_id_;
    msg_ptr_tracked_targets_cart_pose_->header.frame_id = frame_id_;
    msg_ptr_tracked_targets_cart_vel_->header.frame_id = frame_id_;

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
  int target_type = -1;
  const ros::Duration t_raw_timeout = ros::Duration(1.0);
  /* declare the Cartesian targets as global because each one spans two messages */
  std::vector<ainstein_radar_drivers::RadarTargetCartesian> gtarget_cart;
  void RadarInterfaceO79CAN::dataMsgCallback( const can_msgs::Frame &msg )
  {
    if( msg.id == can_id_ )
      {
        // Parse out start of frame messages:
        if( (msg.data[4]==0x00 || msg.data[4]==0x01 || msg.data[4]==0x04) \
            && msg.data[5]==0xFF && msg.data[6]==0xFF && msg.data[7]==0xFF )
          {
            ROS_DEBUG( "received start frame from radar" );
            target_type = msg.data[4];
            targets_received = 0;
            targets_to_come = (msg.data[2] << 8) | msg.data[3];

            // clear radar data message arrays here
            radar_data_msg_ptr_raw_->header.stamp = ros::Time::now();
            radar_data_msg_ptr_tracked_->header.stamp = ros::Time::now();

            if( target_type == 0 || target_type == 1 )
              {
                // Targets in spherical coordinates

                radar_data_msg_ptr_raw_->targets.clear();
                radar_data_msg_ptr_tracked_->targets.clear();
              }
            else if( target_type == 4 )
              {
                // Targets in Cartesian Coordinates
                msg_ptr_tracked_targets_cart_pose_->header.stamp = ros::Time::now();
                msg_ptr_tracked_targets_cart_pose_->poses.clear();
                msg_ptr_tracked_targets_cart_vel_->header.stamp = ros::Time::now();
                msg_ptr_tracked_targets_cart_vel_->velocities.clear();
                gtarget_cart.clear();

              }
            ROS_DEBUG( "receiving %d type %d targets from radar", targets_to_come, target_type );
          }
        // Parse out raw target data messages:
        else if( targets_to_come > 0 )
          {
            ROS_DEBUG( "received target from radar" );

            if( target_type == 0 || target_type == 1 )
              {
                // Extract the target ID and data from the message:
                ainstein_radar_msgs::RadarTarget target;
                target.target_id = static_cast<uint8_t>( msg.data[0] );
                target.snr = static_cast<uint8_t>( msg.data[1] );

                // Range scaling is 0.01m per count:
                target.range = RadarInterfaceO79CAN::msg_range_res * static_cast<double>( static_cast<uint16_t>( ( msg.data[2] << 8 ) + msg.data[3] ) );

                // Speed scaling is 0.005m/s per count, +ve AWAY from radar, -ve TOWARDS:
                target.speed = RadarInterfaceO79CAN::msg_speed_res * static_cast<double>( static_cast<int16_t>( ( msg.data[4] << 8 ) + msg.data[5] ) );

                // Azimuth angle scaling is 1 deg per count:
                target.azimuth = static_cast<double>( static_cast<int8_t>( msg.data[6] ) );

                // Elevation angle scaling is 1 deg per count:
                target.elevation = static_cast<double>( static_cast<int8_t>( msg.data[7] ) );

                if ( target_type == 0 )
                  {
                    radar_data_msg_ptr_raw_->targets.push_back( target );
                  }
                else if( target_type == 1 )
                  {
                    radar_data_msg_ptr_tracked_->targets.push_back( target );
                  }
                targets_received++;
              }
            else if( target_type == 4 )
              {
                if( msg.data[1] == 0 )
                {
                  /* position message; add a new entry to the vector of targets */
                  ainstein_radar_drivers::RadarTargetCartesian tc;
                  tc.id = static_cast<uint8_t>( msg.data[0] );
                  tc.pos.x() = RadarInterfaceO79CAN::msg_pos_res * \
                               static_cast<double>( static_cast<int16_t>( ( msg.data[2] << 8 ) + msg.data[3] ) );
                  tc.pos.y() = RadarInterfaceO79CAN::msg_pos_res * \
                               static_cast<double>( static_cast<int16_t>( ( msg.data[4] << 8 ) + msg.data[5] ) );
                  tc.pos.z() = RadarInterfaceO79CAN::msg_pos_res * \
                               static_cast<double>( static_cast<int16_t>( ( msg.data[6] << 8 ) + msg.data[7] ) );
                  gtarget_cart.push_back(tc);
                }
                else if( msg.data[1] == 1 )
                  {
                    targets_received++;
                    if( gtarget_cart.size() > 0 )
                      {
                      if( gtarget_cart.back().id == msg.data[0] )
                        {
                          gtarget_cart.back().vel.x() = RadarInterfaceO79CAN::msg_vel_res * \
                                                        static_cast<double>( static_cast<int16_t>( ( msg.data[2] << 8 ) + msg.data[3] ) );

                          gtarget_cart.back().vel.y() = RadarInterfaceO79CAN::msg_vel_res * \
                                                        static_cast<double>( static_cast<int16_t>( ( msg.data[4] << 8 ) + msg.data[5] ) );

                          gtarget_cart.back().vel.z() = RadarInterfaceO79CAN::msg_vel_res * \
                                                        static_cast<double>( static_cast<int16_t>( ( msg.data[6] << 8 ) + msg.data[7] ) );
                        }
                      else
                        {
                          std::cout << "WARNING >> no matching position frame found " << std::endl;
                        }
                      }
                    else
                      {
                        std::cout << "WARNING >> no position frames in memory " << std::endl;
                      }
                  }
              }

          }
        else
          {
            ROS_DEBUG( "received message with unknown id: %02x", msg.id );
          }

        if( targets_received == targets_to_come )
          {
            if( ( target_type == 0 ) || ( ( ros::Time::now() - radar_data_msg_ptr_raw_->header.stamp ) > t_raw_timeout ) )
              {
                // if we get a tracked frame, and it's been a while since we got a
                // raw frame, publish an empty raw frame to clear the display
                pub_radar_data_raw_.publish( radar_data_msg_ptr_raw_ );
              }
            else if( target_type == 1 )
              {
                pub_radar_data_tracked_.publish( radar_data_msg_ptr_tracked_ );
              }
            else if( target_type == 4 )
              {
                for( const auto &t : gtarget_cart )
                  {
                    // Fill the velocity message:
                    geometry_msgs::Twist twist_msg;
                    twist_msg.linear.x = t.vel.x();
                    twist_msg.linear.y = t.vel.y();
                    twist_msg.linear.z = t.vel.z();
                    msg_ptr_tracked_targets_cart_vel_->velocities.push_back( twist_msg );

                    // Fill the pose message:
                    Eigen::Affine3d pose_eigen;
                    pose_eigen.translation() = t.pos;

                    // Compute the pose assuming the +x direction is the current
                    // estimated Cartesian velocity direction
                    Eigen::Matrix3d rot_mat;
                    if( t.vel.norm() < 1e-3 ) // handle degenerate case of zero velocity
                      {
                        rot_mat = Eigen::Matrix3d::Identity();
                      }
                    else
                      {
                        rot_mat.col( 0 ) = t.vel / t.vel.norm();
                        rot_mat.col( 1 ) = Eigen::Vector3d::UnitZ().cross( rot_mat.col( 0 ) );
                        rot_mat.col( 2 ) = rot_mat.col( 0 ).cross( rot_mat.col( 1 ) );
                      }

                    pose_eigen.linear() = rot_mat;

                    geometry_msgs::Pose pose_msg;
                    pose_msg = tf2::toMsg( pose_eigen );

                    msg_ptr_tracked_targets_cart_pose_->poses.push_back( pose_msg );
                  }
                // Publish the tracked target data:
                pub_tracked_targets_cart_pose_.publish( msg_ptr_tracked_targets_cart_pose_ );
                pub_tracked_targets_cart_vel_.publish( msg_ptr_tracked_targets_cart_vel_ );

              }
            targets_received = 0;
            targets_to_come = -1;
            target_type = -1;
          }
      }
  }

  void RadarInterfaceO79CAN::publishRadarInfo( void )
  {
    // Advertise the O79 sensor info (LATCHED):
    pub_radar_info_ = nh_private_.advertise<ainstein_radar_msgs::RadarInfo>( "radar_info", 10, true );

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
    pub_radar_info_.publish( radar_info_msg_ptr_ );
  }

} // namespace ainstein_drivers
