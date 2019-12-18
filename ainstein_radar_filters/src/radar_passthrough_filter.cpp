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
    nh_private_( node_handle_private )
  {
    pub_radar_data_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "radar_out", 10 );
    sub_radar_data_ = nh_.subscribe( "radar_in", 10,
				     &RadarPassthroughFilter::radarDataCallback,
				     this );
    
    // Set up dynamic reconfigure:
    dynamic_reconfigure::Server<ainstein_radar_filters::PassthroughFilterConfig>::CallbackType f;
    f = boost::bind(&RadarPassthroughFilter::dynConfigCallback, this, _1, _2);
    dyn_config_server_.setCallback( f );
  }

  void RadarPassthroughFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray& msg )
  {
    // Convert from radar message to PCL point cloud type
    pcl::PointCloud<PointRadarTarget> pcl_cloud;
    RadarTargetArrayToPointCloud::radarTargetArrayToPclCloud( msg, pcl_cloud );

    // Filter the PCL point cloud using the PCL passthrough class
    pcl::PointCloud<PointRadarTarget> pcl_cloud_filt;
    passthrough_filt_.setInputCloud( pcl_cloud.makeShared() );
    passthrough_filt_.filter( pcl_cloud_filt );

    // Convert back to radar message type
    ainstein_radar_msgs::RadarTargetArray msg_filt;
    RadarTargetArrayToPointCloud::pclCloudToRadarTargetArray( pcl_cloud_filt, msg_filt );
    
    // Copy metadata from input data and publish
    msg_filt.header = msg.header;
    pub_radar_data_.publish( msg_filt );
  }

} // namespace ainstein_radar_filters
