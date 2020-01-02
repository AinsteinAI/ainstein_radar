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

#include "ainstein_radar_filters/radar_combine_filter.h"

namespace ainstein_radar_filters
{
  RadarCombineFilter::RadarCombineFilter( const ros::NodeHandle& node_handle,
					  const ros::NodeHandle& node_handle_private ) :
    nh_( node_handle ),
    nh_private_( node_handle_private ),
    sub_radar_data_A_( nh_, "radar_in_A", 1 ),
    sub_radar_data_B_( nh_, "radar_in_B", 1 ),
    listen_tf_( buffer_tf_ )
  {
    // Set up the publisher for the combined radar messages
    pub_radar_data_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "radar_out", 10 );

    // Set the desired common output frame for the data
    nh_private_.param( "output_frame_id", output_frame_id_, std::string( "map" ) );
    
    // Approximate time synchronizer for two RadarTargetArray topics
    sync_policy_.reset( new RadarSyncPolicy( 10 ) );
    sync_.reset( new message_filters::Synchronizer<RadarSyncPolicy>( static_cast<const RadarSyncPolicy&>( *sync_policy_ ),
								     sub_radar_data_A_,
								     sub_radar_data_B_ ) );
    sync_->registerCallback( boost::bind( &RadarCombineFilter::radarDataCallback, this, _1, _2 ) );
    
    // Set up dynamic reconfigure:
    dynamic_reconfigure::Server<ainstein_radar_filters::CombineFilterConfig>::CallbackType f;
    f = boost::bind( &RadarCombineFilter::dynConfigCallback, this, _1, _2 );
    dyn_config_server_.setCallback( f );
  }

  void RadarCombineFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg_A,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg_B )
  {
    // Convert from radar message to PCL point cloud type
    pcl::PointCloud<PointRadarTarget> pcl_cloud_A, pcl_cloud_B;
    data_conversions::radarTargetArrayToPclCloud( *msg_A, pcl_cloud_A );
    data_conversions::radarTargetArrayToPclCloud( *msg_B, pcl_cloud_B );

    // Transform radar point clouds to common output frame
    // Eigen::Affine3d tf_radar_to_camera;
    // if( buffer_tf_.canTransform( "camera_color_optical_frame", "radar_frame", ros::Time( 0 ) ) )
    //   {
    //     tf_radar_to_camera =
    //       tf2::transformToEigen( buffer_tf_.lookupTransform( "camera_color_optical_frame", "radar_frame", ros::Time( 0 ) ) );
    //   }
    // else
    //   {
    //     ROS_WARN_STREAM( "Timeout while waiting for transform." );
    //   }
    
    // Combine the radar point clouds
    pcl::PointCloud<PointRadarTarget> pcl_cloud_combined;
    pcl_cloud_combined = pcl_cloud_A + pcl_cloud_B;
    
    // Convert back to radar message type
    ainstein_radar_msgs::RadarTargetArray msg_combined;
    data_conversions::pclCloudToRadarTargetArray( pcl_cloud_combined, msg_combined );
    
    // Copy metadata from input data and publish
    msg_combined.header.frame_id = output_frame_id_;
    msg_combined.header.stamp = msg_A->header.stamp; // use msg_A's stamp for now    
    pub_radar_data_.publish( msg_combined );
  }

} // namespace ainstein_radar_filters
