/*
  Copyright <2018-2020> <Ainstein, Inc.>

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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <iostream>
#include <cstdint>
#include <cerrno>

#include "ainstein_radar_drivers/radar_interface_o79_udp.h"

namespace ainstein_radar_drivers
{

	const ros::Duration t_raw_timeout = ros::Duration(0.4);

RadarInterfaceO79UDP::RadarInterfaceO79UDP( ros::NodeHandle node_handle,
					    ros::NodeHandle node_handle_private ) :
  nh_( node_handle ),
  nh_private_( node_handle_private ),
  radar_data_msg_ptr_raw_( new ainstein_radar_msgs::RadarTargetArray ),
  radar_data_msg_ptr_tracked_( new ainstein_radar_msgs::RadarTargetArray ),
  cloud_msg_ptr_raw_( new sensor_msgs::PointCloud2 ),
  cloud_msg_ptr_tracked_( new sensor_msgs::PointCloud2 ),
  msg_ptr_tracked_boxes_( new ainstein_radar_msgs::BoundingBoxArray ),
  msg_ptr_tracked_targets_cart_pose_( new geometry_msgs::PoseArray ),
  msg_ptr_tracked_targets_cart_vel_( new ainstein_radar_msgs::TwistArray ),
  radar_info_msg_ptr_( new ainstein_radar_msgs::RadarInfo )
{
  // Get the host IP and port:
  std::string host_ip_addr;
  nh_private_.param( "host_ip", host_ip_addr, std::string( "10.0.0.75" ) );

  int host_port;
  nh_private_.param( "host_port", host_port, 1024 );

  // Get the radar IP and port:
  std::string radar_ip_addr;
  nh_private_.param( "radar_ip", radar_ip_addr, std::string( "10.0.0.10" ) );

  int radar_port;
  nh_private_.param( "radar_port", radar_port, 7 );

  // Get the radar data frame ID:
  nh_private_.param( "frame_id", frame_id_, std::string( "map" ) );

  // Get whether to publish ROS point cloud messages:
  nh_private_.param( "publish_raw_cloud", publish_raw_cloud_, false );
  nh_private_.param( "publish_tracked_cloud", publish_tracked_cloud_, false );

  // Set the frame ID:
  radar_data_msg_ptr_raw_->header.frame_id = frame_id_;
  radar_data_msg_ptr_tracked_->header.frame_id = frame_id_;
  msg_ptr_tracked_boxes_->header.frame_id = frame_id_;
  msg_ptr_tracked_targets_cart_pose_->header.frame_id = frame_id_;
  msg_ptr_tracked_targets_cart_vel_->header.frame_id = frame_id_;

  // Publish the RadarInfo message:
  publishRadarInfo();

  // Create the radar driver object:
  driver_.reset( new RadarDriverO79UDP( host_ip_addr, host_port,
					radar_ip_addr, radar_port ) );

  // Advertise the O79 raw targets data:
  pub_radar_data_raw_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "targets/raw", 10 );

  // Advertise the O79 tracked targets data:
  pub_radar_data_tracked_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "targets/tracked", 10 );

  // Advertise the O79 raw point cloud:
  pub_cloud_raw_ = nh_private_.advertise<sensor_msgs::PointCloud2>( "cloud/raw", 10 );

  // Advertise the O79 tracked point cloud:
  pub_cloud_tracked_ = nh_private_.advertise<sensor_msgs::PointCloud2>( "cloud/tracked", 10 );

  // Advertise the O79 tracked targets data:
  pub_radar_data_tracked_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "targets/tracked", 10 );

  // Advertise the O79 tracked object bounding boxes:
  pub_bounding_boxes_ = nh_private_.advertise<ainstein_radar_msgs::BoundingBoxArray>( "boxes", 10 );

  // Advertise the O79 tracked object poses:
  pub_tracked_targets_cart_pose_ = nh_private_.advertise<geometry_msgs::PoseArray>( "poses", 10 );

  // Advertise the O79 tracked object velocities:
  pub_tracked_targets_cart_vel_ = nh_private_.advertise<ainstein_radar_msgs::TwistArray>( "velocities", 10 );

  // Start the data collection thread:
  thread_ = std::unique_ptr<std::thread>( new std::thread( &RadarInterfaceO79UDP::mainLoop, this ) );
  mutex_.lock();
  is_running_ = true;
  mutex_.unlock();
}

RadarInterfaceO79UDP::~RadarInterfaceO79UDP(void)
{
  mutex_.lock();
  is_running_ = false;
  mutex_.unlock();
  thread_->join();
}

void RadarInterfaceO79UDP::mainLoop(void)
{
  // Connect to the radar:
  driver_->connect();

  // Enter the main data receiving loop:
  bool running = true;
  std::vector<ainstein_radar_drivers::RadarTarget> targets_raw;
  std::vector<ainstein_radar_drivers::RadarTarget> targets_tracked;
  std::vector<ainstein_radar_drivers::BoundingBox> bounding_boxes;
  std::vector<ainstein_radar_drivers::RadarTargetCartesian> targets_tracked_cart;

  while( running && !ros::isShuttingDown() )
    {
      // Call to block until data has been received:
      if( driver_->receiveTargets( targets_raw, targets_tracked, bounding_boxes, targets_tracked_cart ) == false )
	{
	  ROS_WARN_STREAM( "Failed to read data: " << std::strerror( errno ) << std::endl );
	}
      else
	{
	  if( targets_raw.size() > 0 )
	    {
	      // Fill in the raw RadarTargetArray message from the received targets:
	      radar_data_msg_ptr_raw_->header.stamp = ros::Time::now();
	      radar_data_msg_ptr_raw_->targets.clear();
	      for( const auto &t : targets_raw )
		{
			if (t.id >= 0)
			{
		  	radar_data_msg_ptr_raw_->targets.push_back( targetToROSMsg( t ) );
			}
		}

	      // Publish the raw target data:
	      pub_radar_data_raw_.publish( radar_data_msg_ptr_raw_ );

	      // Optionally publish raw detections as ROS point cloud:
	      if( publish_raw_cloud_ )
		{
		  ainstein_radar_filters::data_conversions::radarTargetArrayToROSCloud( *radar_data_msg_ptr_raw_, *cloud_msg_ptr_raw_ );
		  pub_cloud_raw_.publish( cloud_msg_ptr_raw_ );
		}

	    }

	  if( targets_tracked.size() > 0 )
	    {
	      // Fill in the tracked RadarTargetArray message from the received targets:
	      radar_data_msg_ptr_tracked_->header.stamp = ros::Time::now();
	      radar_data_msg_ptr_tracked_->targets.clear();
	      for( const auto &t : targets_tracked )
		{
			if (t.id >= 0)
			{
				radar_data_msg_ptr_tracked_->targets.push_back( targetToROSMsg( t ) );
			}
		}

	      // Publish the tracked target data:
	      pub_radar_data_tracked_.publish( radar_data_msg_ptr_tracked_ );

	      // Optionally publish tracked detections as ROS point cloud:
	      if( publish_tracked_cloud_ )
		{
		  ainstein_radar_filters::data_conversions::radarTargetArrayToROSCloud( *radar_data_msg_ptr_tracked_, *cloud_msg_ptr_tracked_ );
		  pub_cloud_tracked_.publish( cloud_msg_ptr_tracked_ );
		}

		// Publish an empty raw frame if sufficient time has passed since a real one was received
		// This clears the rviz display if no points are detected and the radar is running
		if ( (ros::Time::now() - radar_data_msg_ptr_raw_->header.stamp ) > t_raw_timeout )
		{
			radar_data_msg_ptr_raw_->header.stamp = ros::Time::now();
			radar_data_msg_ptr_raw_->targets.clear();
			// Publish the raw target data:
			pub_radar_data_raw_.publish( radar_data_msg_ptr_raw_ );

			// Optionally publish raw detections as ROS point cloud:
			if( publish_raw_cloud_ )
			{
				ainstein_radar_filters::data_conversions::radarTargetArrayToROSCloud( *radar_data_msg_ptr_raw_, *cloud_msg_ptr_raw_ );
				pub_cloud_raw_.publish( cloud_msg_ptr_raw_ );
			}

		}

	    }

	  if( bounding_boxes.size() > 0 )
	    {
	      // Fill in the BoundingBox message from the received boxes:
	      msg_ptr_tracked_boxes_->header.stamp = ros::Time::now();
	      msg_ptr_tracked_boxes_->boxes.clear();

	      for( const auto &b : bounding_boxes )
		{
		  msg_ptr_tracked_boxes_->boxes.push_back( boundingBoxToROSMsg( b, frame_id_ ) );
		}

	      // Publish the tracked target data:
	      pub_bounding_boxes_.publish( msg_ptr_tracked_boxes_ );
	    }

	  if( targets_tracked_cart.size() > 0 )
	    {
	      // Fill in the tracked PoseArray and TwistArray messages from the received targets:
	      msg_ptr_tracked_targets_cart_pose_->header.stamp = ros::Time::now();
	      msg_ptr_tracked_targets_cart_pose_->poses.clear();
	      msg_ptr_tracked_targets_cart_vel_->header.stamp = ros::Time::now();
	      msg_ptr_tracked_targets_cart_vel_->velocities.clear();
	      for( const auto &t : targets_tracked_cart )
		{
		  // Fill the velocity message:
		  geometry_msgs::Twist twist_msg;
		  twist_msg.linear.x = t.vel.x();
		  twist_msg.linear.y = t.vel.y();
		  twist_msg.linear.z = t.vel.z();
		  msg_ptr_tracked_targets_cart_vel_->velocities.push_back( twist_msg );

		  // Publish the tracked Cartesian velocities:
		  pub_tracked_targets_cart_vel_.publish( msg_ptr_tracked_targets_cart_vel_ );

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

	      // Publish the tracked Cartesian poses:
	      pub_tracked_targets_cart_pose_.publish( msg_ptr_tracked_targets_cart_pose_ );
	    }

	}

      // Check whether the data loop should still be running:
      mutex_.lock();
      running = is_running_;
      mutex_.unlock();
    }
}

  void RadarInterfaceO79UDP::publishRadarInfo( void )
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

} // namespace ainstein_radar_drivers
