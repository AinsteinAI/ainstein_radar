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
#include <rviz_visual_tools/rviz_visual_tools.h>
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

namespace rvt = rviz_visual_tools;

namespace ainstein_radar_drivers
{

	const ros::Duration t_gnd_timeout = ros::Duration(0.4);

RadarInterfaceO79UDP::RadarInterfaceO79UDP( ros::NodeHandle node_handle,
					    ros::NodeHandle node_handle_private ) :
  nh_( node_handle ),
  nh_private_( node_handle_private ),
  radar_data_msg_ptr_raw_( new ainstein_radar_msgs::RadarTargetArray ),
  radar_data_msg_ptr_filtered_pcl_( new ainstein_radar_msgs::RadarTargetArray ),
  radar_data_msg_ptr_tracked_( new ainstein_radar_msgs::RadarTrackedObjectArray ),
  radar_data_msg_ptr_ground_( new ainstein_radar_msgs::RadarTrackedObjectArray ),
  msg_ptr_tracked_boxes_( new ainstein_radar_msgs::BoundingBoxArray ),
  cloud_msg_ptr_raw_( new sensor_msgs::PointCloud2 ),
  cloud_msg_ptr_filtered_( new sensor_msgs::PointCloud2 ),
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

  // Get string message parameters
  std::string name;
  nh_private_.param("name", name, std::string("o79_udp"));
  nh_private_.param("msg_offset_x", msg_offset_x_, -2.0);
  nh_private_.param("msg_offset_y", msg_offset_y_, 0.0);
  nh_private_.param("msg_offset_z", msg_offset_z_, 0.0);

  // Create pose for string messages
  msg_pose_ = Eigen::Isometry3d::Identity();
  msg_pose_.translation().x() = msg_offset_x_;
  msg_pose_.translation().y() = msg_offset_y_;
  msg_pose_.translation().z() = msg_offset_z_;

  // Set the frame ID:
  radar_data_msg_ptr_raw_->header.frame_id = frame_id_;
  radar_data_msg_ptr_filtered_pcl_->header.frame_id = frame_id_;
  radar_data_msg_ptr_tracked_->header.frame_id = frame_id_;
  radar_data_msg_ptr_ground_->header.frame_id = frame_id_;
  msg_ptr_tracked_boxes_->header.frame_id = frame_id_;

  // Publish the RadarInfo message:
  publishRadarInfo();

  // Create the radar driver object:
  driver_.reset( new RadarDriverO79UDP( host_ip_addr, host_port,
					radar_ip_addr, radar_port ) );

  // Advertise the O79 raw targets data:
  pub_radar_data_raw_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "targets/raw", 10 );

  // Advertise the O79 filter 1 raw targets data:
  pub_radar_data_filtered_pcl_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "targets/filtered", 10 );

  // Advertise the O79 tracked targets data:
  pub_radar_data_tracked_ = nh_private_.advertise<ainstein_radar_msgs::RadarTrackedObjectArray>( "objects", 10 );

    // Advertise the O79 ground data:
  pub_radar_data_ground_ = nh_private_.advertise<ainstein_radar_msgs::RadarTrackedObjectArray>( "ground", 10 );
  
  // Advertise the O79 tracked object bounding boxes:
  pub_bounding_boxes_ = nh_private_.advertise<ainstein_radar_msgs::BoundingBoxArray>( "boxes", 10 );

  // Advertise the O79 raw point cloud:
  pub_cloud_raw_ = nh_private_.advertise<sensor_msgs::PointCloud2>( "cloud/raw", 10 );

  // Advertise the O79 raw point cloud (filter 1):
  pub_cloud_filtered_ = nh_private_.advertise<sensor_msgs::PointCloud2>( "cloud/filtered", 10 );

  str_msg_ptr_.reset(new rvt::RvizVisualTools(frame_id_, name + "/messages"));
  str_msg_ptr_->loadMarkerPub();  // create publisher before waiting
  // Clear messages
  str_msg_ptr_->deleteAllMarkers();
  //   str_msg_ptr_->enableBatchPublishing();

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
  std::vector<ainstein_radar_drivers::RadarTarget> raw_pcl;
  std::vector<ainstein_radar_drivers::RadarTarget> filtered_pcl;
  std::vector<ainstein_radar_drivers::BoundingBox> bounding_boxes;
  std::vector<ainstein_radar_drivers::RadarTargetCartesian> targets_tracked_cart;
  std::vector<ainstein_radar_drivers::RadarTargetCartesian> targets_ground_cart;
  std::vector<ainstein_radar_drivers::RadarDeviceAlarms> alarms;

  while( running && !ros::isShuttingDown() )
    {
      // Call to block until data has been received:
      if( driver_->receiveTargets( raw_pcl, 
								   bounding_boxes, 
								   targets_tracked_cart,
								   targets_ground_cart,
								   filtered_pcl,
								   alarms) == false )
	{
	  ROS_WARN_STREAM( "Failed to read data: " << std::strerror( errno ) << std::endl );
	}
      else
	{
	  if( raw_pcl.size() > 0 )
	    {
	      // Fill in the raw RadarTargetArray message from the received targets:
	      radar_data_msg_ptr_raw_->header.stamp = ros::Time::now();
	      radar_data_msg_ptr_raw_->targets.clear();
	      for( const auto &t : raw_pcl )
		{
		  if (t.id >= 0)
		    {
		      radar_data_msg_ptr_raw_->targets.push_back( utilities::targetToROSMsg( t ) );
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
	if(filtered_pcl.size() > 0)
	   {
		// Fill in the raw RadarTargetArray message from the received targets:
		radar_data_msg_ptr_filtered_pcl_->header.stamp = ros::Time::now();
		radar_data_msg_ptr_filtered_pcl_->targets.clear();
		for( const auto &t : filtered_pcl )
		{
			if (t.id >= 0)
			{
				radar_data_msg_ptr_filtered_pcl_->targets.push_back( utilities::targetToROSMsg( t ) );
			}
		}

		// Publish the raw target data:
		pub_radar_data_filtered_pcl_.publish( radar_data_msg_ptr_filtered_pcl_ );

		// Optionally publish raw detections as ROS point cloud:
		if( publish_raw_cloud_ )
		{
		  ainstein_radar_filters::data_conversions::radarTargetArrayToROSCloud( *radar_data_msg_ptr_filtered_pcl_, *cloud_msg_ptr_filtered_ );
		  pub_cloud_filtered_.publish( cloud_msg_ptr_filtered_ );
		}
	    }

	  if( bounding_boxes.size() > 0 )
	    {
	      // Fill in the BoundingBox message from the received boxes:
	      msg_ptr_tracked_boxes_->header.stamp = ros::Time::now();
	      msg_ptr_tracked_boxes_->boxes.clear();

	      for( const auto &b : bounding_boxes )
		{
		  msg_ptr_tracked_boxes_->boxes.push_back( utilities::boundingBoxToROSMsg( b ) );
		}

	      // Publish the tracked target data:
	      pub_bounding_boxes_.publish( msg_ptr_tracked_boxes_ );
	    }

	  if( targets_tracked_cart.size() > 0 )
	    {
	      // Fill in the RadarTrackedObjectArray message from the received Cartesian targets
	      // using the same object message as for spherical data (only one type of tracked
	      // object data will be active at once):
	      radar_data_msg_ptr_tracked_->header.stamp = ros::Time::now();
	      radar_data_msg_ptr_tracked_->objects.clear();

	      ainstein_radar_msgs::RadarTrackedObject obj_msg;
	      for( const auto &t : targets_tracked_cart )
			{
				if( t.id >= 0)
					{
					// Only populate the values if there's a valid tid, otherwise the intent is to publish an empty frame to clear the display

					// Pass the target ID through to the object ID:
					obj_msg.id = t.id;
				
					// Fill in the pose information:
					obj_msg.pose = ainstein_radar_filters::data_conversions::posVelToPose( t.pos, t.vel );

					// Fill in the velocity information:
					obj_msg.velocity.linear.x = t.vel.x();
					obj_msg.velocity.linear.y = t.vel.y();
					obj_msg.velocity.linear.z = t.vel.z();

					// Fill in dummy bounding box information:
					obj_msg.box.pose = obj_msg.pose;
					obj_msg.box.dimensions.x = 0.01;
					obj_msg.box.dimensions.y = 0.01;
					obj_msg.box.dimensions.z = 0.01;

					radar_data_msg_ptr_tracked_->objects.push_back( obj_msg );
					}		      
			}

		  // Publish an empty ground frame if sufficient time has passed since a real one was received
	      // This clears the rviz display if no ground targets are detected and the radar is running
	      if ( (ros::Time::now() - radar_data_msg_ptr_ground_->header.stamp ) > t_gnd_timeout )
			{
				radar_data_msg_ptr_ground_->header.stamp = ros::Time::now();
				radar_data_msg_ptr_ground_->objects.clear();

				// Publish the ground target data:
				pub_radar_data_ground_.publish( radar_data_msg_ptr_ground_ );	  
			}

	      // Publish the tracked target data:
	      pub_radar_data_tracked_.publish( radar_data_msg_ptr_tracked_ );
	    }

	  if( targets_ground_cart.size() > 0 )
	    {
	      // Fill in the RadarTrackedObjectArray message from the received ground targets
	      radar_data_msg_ptr_ground_->header.stamp = ros::Time::now();
	      radar_data_msg_ptr_ground_->objects.clear();

	      ainstein_radar_msgs::RadarTrackedObject obj_msg;
	      for( const auto &t : targets_ground_cart )
		    {
				if( t.id >= 0)
				  {
					// Only populate the values if there's a valid tid, otherwise the intent is to publish an empty frame to clear the display

					// Pass the target ID through to the object ID:
					obj_msg.id = t.id;
				
					// Fill in the pose information:
					obj_msg.pose = ainstein_radar_filters::data_conversions::posVelToPose( t.pos, t.vel );

					// Fill in the velocity information:
					obj_msg.velocity.linear.x = t.vel.x();
					obj_msg.velocity.linear.y = t.vel.y();
					obj_msg.velocity.linear.z = t.vel.z();

					// Fill in dummy bounding box information:
					obj_msg.box.pose = obj_msg.pose;
					obj_msg.box.dimensions.x = 0.01;
					obj_msg.box.dimensions.y = 0.01;
					obj_msg.box.dimensions.z = 0.01;

					radar_data_msg_ptr_ground_->objects.push_back( obj_msg );
				  }		      
		    }
	      // Publish the ground target data:
	      pub_radar_data_ground_.publish( radar_data_msg_ptr_ground_ );
	    }
		if(alarms.size() > 0)
		{
			RadarDeviceAlarms l_alarm;
			for( const auto &a : alarms )
		    {
				l_alarm.FA_bits |= a.FA_bits;
				l_alarm.TFTKO_bits |= a.TFTKO_bits;
			}

			std::string disp_str = "FA: ";
			disp_str += l_alarm.getStatusStr(l_alarm.FA_bits);
			disp_str += "\nTFTKO: ";
			disp_str += l_alarm.getStatusStr(l_alarm.TFTKO_bits);

			rvt::colors disp_color = rvt::GREEN;
			if(l_alarm.FA_bits > 0)
			{
				disp_color = rvt::RED;
			}
			str_msg_ptr_->deleteAllMarkers();
			str_msg_ptr_->publishText(msg_pose_, disp_str, disp_color, rvt::XXXXLARGE, false);
			str_msg_ptr_->trigger();
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
