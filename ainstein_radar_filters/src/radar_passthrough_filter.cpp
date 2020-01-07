/*
  Copyright <2018-2019> <Ainstein, Inc.>

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

#include "ainstein_radar_filters/radar_passthrough_filter.h"

namespace ainstein_radar_filters
{
  RadarPassthroughFilter::RadarPassthroughFilter( const ros::NodeHandle& node_handle,
						  const ros::NodeHandle& node_handle_private ) :
    nh_( node_handle ),
    nh_private_( node_handle_private ),
    listen_tf_( buffer_tf_ )
  {
    pub_radar_data_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "radar_out", 10 );
    sub_radar_data_ = nh_.subscribe( "radar_in", 10,
				     &RadarPassthroughFilter::radarDataCallback,
				     this );

    // Get the non-dynamic parameters:
    nh_private_.param( "input_frame", input_frame_, std::string( "" ) );
    nh_private_.param( "output_frame", output_frame_, std::string( "" ) );
    
    // Set up dynamic reconfigure:
    dynamic_reconfigure::Server<ainstein_radar_filters::PassthroughFilterConfig>::CallbackType f;
    f = boost::bind( &RadarPassthroughFilter::dynConfigCallback, this, _1, _2 );
    dyn_config_server_.setCallback( f );
  }

  void RadarPassthroughFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg )
  {
    // Convert from radar message to PCL point cloud type
    sensor_msgs::PointCloud2 ros_cloud;
    data_conversions::radarTargetArrayToROSCloud( *msg, ros_cloud );

    // Transform the ROS cloud message to the specified input frame, using the
    // message's original frame_id if not set in configuration
    sensor_msgs::PointCloud2 ros_cloud_input_frame;
    geometry_msgs::TransformStamped tf_msg_to_input;
    if( !input_frame_.empty() )
      {
	tf_msg_to_input = buffer_tf_.lookupTransform( input_frame_,
						      msg->header.frame_id,
						      ros::Time( 0 ) );
	tf2::doTransform( ros_cloud, ros_cloud_input_frame, tf_msg_to_input );
      }
    else
      {
	ros_cloud_input_frame = ros_cloud;
      }

    // Convert ROS cloud in input frame to a PCL cloud
    pcl::PointCloud<PointRadarTarget> pcl_cloud_input_frame;
    pcl::fromROSMsg( ros_cloud_input_frame, pcl_cloud_input_frame );

    // Filter the PCL point cloud using the PCL passthrough class
    pcl::PointCloud<PointRadarTarget> pcl_cloud_filt;
    passthrough_filt_.setInputCloud( pcl_cloud_input_frame.makeShared() );
    passthrough_filt_.filter( pcl_cloud_filt );

    // Convert back to PointCloud2
    sensor_msgs::PointCloud2 ros_cloud_filt;
    pcl::toROSMsg( pcl_cloud_filt, ros_cloud_filt );

    // Transform to the specified output frame, using the original frame if
    // no output frame was specified
    sensor_msgs::PointCloud2 ros_cloud_output_frame;
    geometry_msgs::TransformStamped tf_input_to_output;
    if( !output_frame_.empty() )
      {
	tf_input_to_output = buffer_tf_.lookupTransform( output_frame_,
							 ros_cloud_filt.header.frame_id,
							 ros::Time( 0 ) );
	tf2::doTransform( ros_cloud_filt, ros_cloud_output_frame, tf_input_to_output );
      }
    else
      {
	// If the input frame in which filtering was performed is not empty, then we need to
	// transform back to the original output frame before publishing the result
	if( !input_frame_.empty() )
	  {
	    tf_input_to_output = buffer_tf_.lookupTransform( msg->header.frame_id,
							     ros_cloud_filt.header.frame_id,
							     ros::Time( 0 ) );
	    tf2::doTransform( ros_cloud_filt, ros_cloud_output_frame, tf_input_to_output );
	  }
	else
	  {
	    // if both input and output frames were not specified, nothing to do but
	    // copy the filtered cloud
	    ros_cloud_output_frame = ros_cloud_filt;
	  }
      }
    
    // Convert back to radar message type
    ainstein_radar_msgs::RadarTargetArray msg_filt;
    data_conversions::rosCloudToRadarTargetArray( ros_cloud_output_frame, msg_filt );

    // Copy metadata from original message/output frame param and publish
    msg_filt.header.stamp = msg->header.stamp;
    if( !output_frame_.empty() )
      {
	msg_filt.header.frame_id = output_frame_;
      }
    else
      {
	msg_filt.header.frame_id = msg->header.frame_id;
      }
    
    pub_radar_data_.publish( msg_filt );
  }

} // namespace ainstein_radar_filters
