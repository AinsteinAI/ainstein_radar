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
    listen_tf_( buffer_tf_ )
  {
    // Set up the publisher for the combined radar messages
    pub_radar_data_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "radar_out", 10 );

    // Set the desired common output frame for the data
    nh_private_.param( "output_frame_id", output_frame_id_, std::string( "map" ) );

    // Set up radar subscribers
    std::vector<std::string> topic_names;
    nh_private_.getParam( "topic_names", topic_names );
    for( const auto& name : topic_names )
      {
	sub_radar_data_.push_back( std::unique_ptr<Subscriber<ainstein_radar_msgs::RadarTargetArray>>( new Subscriber<ainstein_radar_msgs::RadarTargetArray>( nh_, name, 1 ) ) );
      }

    // Store the number of input topics to sync
    n_topics_ = sub_radar_data_.size();
    
    // Register subscribers based on number of specified topics
    switch( n_topics_ )
      {
      case 1:
	ROS_ERROR_STREAM( "Combine filter should only be used for 2+ topics." );
	break;
	
      case 2:
      	registerSubscribers( *sub_radar_data_.at( 0 ),
      			     *sub_radar_data_.at( 1 ) );
      	break;

      case 3:
      	registerSubscribers( *sub_radar_data_.at( 0 ),
      			     *sub_radar_data_.at( 1 ),
      			     *sub_radar_data_.at( 2 ) );
      	break;
	
      case 4:
      	registerSubscribers( *sub_radar_data_.at( 0 ),
      			     *sub_radar_data_.at( 1 ),
      			     *sub_radar_data_.at( 2 ),
      			     *sub_radar_data_.at( 3 ) );
      	break;
	
      case 5:
      	registerSubscribers( *sub_radar_data_.at( 0 ),
      			     *sub_radar_data_.at( 1 ),
      			     *sub_radar_data_.at( 2 ),
      			     *sub_radar_data_.at( 3 ),
      			     *sub_radar_data_.at( 4 ) );
      	break;
	
      case 6:
      	registerSubscribers( *sub_radar_data_.at( 0 ),
      			     *sub_radar_data_.at( 1 ),
      			     *sub_radar_data_.at( 2 ),
      			     *sub_radar_data_.at( 3 ),
      			     *sub_radar_data_.at( 4 ),
      			     *sub_radar_data_.at( 5 ) );
      	break;
	
      case 7:
      	registerSubscribers( *sub_radar_data_.at( 0 ),
      			     *sub_radar_data_.at( 1 ),
      			     *sub_radar_data_.at( 2 ),
      			     *sub_radar_data_.at( 3 ),
      			     *sub_radar_data_.at( 4 ),
      			     *sub_radar_data_.at( 5 ),
      			     *sub_radar_data_.at( 6 ) );
      	break;
	
      case 8:
      	registerSubscribers( *sub_radar_data_.at( 0 ),
      			     *sub_radar_data_.at( 1 ),
      			     *sub_radar_data_.at( 2 ),
      			     *sub_radar_data_.at( 3 ),
      			     *sub_radar_data_.at( 4 ),
      			     *sub_radar_data_.at( 5 ),
      			     *sub_radar_data_.at( 6 ),
      			     *sub_radar_data_.at( 7 ) );
      	break;

      default:
	ROS_ERROR_STREAM( "Unsupported number of topics (" << sub_radar_data_.size() << ") specified." );
      }
    
    // Set up dynamic reconfigure
    dynamic_reconfigure::Server<ainstein_radar_filters::CombineFilterConfig>::CallbackType f;
    f = boost::bind( &RadarCombineFilter::dynConfigCallback, this, _1, _2 );
    dyn_config_server_.setCallback( f );
  }

  void RadarCombineFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2 )
  {
    std::vector<ainstein_radar_msgs::RadarTargetArray> msg_arr( 2 );
    msg_arr.at( 0 ) = *msg1;
    msg_arr.at( 1 ) = *msg2;
    
    ainstein_radar_msgs::RadarTargetArray msg_combined;
    combineMsgs( msg_arr, msg_combined );
       
    // Copy metadata from input data and publish
    msg_combined.header.frame_id = output_frame_id_;
    msg_combined.header.stamp = msg1->header.stamp; // use msg1's stamp for now    
    pub_radar_data_.publish( msg_combined ); 
  }

  void RadarCombineFilter::combineMsgs( const std::vector<ainstein_radar_msgs::RadarTargetArray>& msg_arr,
					ainstein_radar_msgs::RadarTargetArray& msg_combined )
  {
    pcl::PointCloud<PointRadarTarget> pcl_cloud_combined;
    pcl_cloud_combined.clear();
    for( const auto& msg : msg_arr)
      {
	// Convert from radar message to ROS point cloud type
	sensor_msgs::PointCloud2 cloud_in;
	data_conversions::radarTargetArrayToROSCloud( msg, cloud_in );

	// Transform radar point clouds to common output frame. There is no version of the function
	// transformPointCloud for tf2, so we instead convert to ROS cloud and use tf2::doTransform.
	// Note that we should really implement doTransform for radar types natively eg following:
	// http://library.isr.ist.utl.pt/docs/roswiki/tf2(2f)Tutorials(2f)Transforming(20)your(20)own(20)datatypes.html
	sensor_msgs::PointCloud2 cloud_out;
	if( buffer_tf_.canTransform( output_frame_id_, msg.header.frame_id, ros::Time( 0 ) ) )
      {
	tf2::doTransform( cloud_in, cloud_out,
			  buffer_tf_.lookupTransform( output_frame_id_,
						      msg.header.frame_id, ros::Time( 0 ) ) );
      }
    else
      {
	ROS_WARN_STREAM( "Timeout while waiting for transform for cloud A." );
      }

	// Combine the radar point clouds by comverting to PCL and using PCL cloud addition
	pcl::PointCloud<PointRadarTarget> pcl_cloud_out;
	pcl::fromROSMsg( cloud_out, pcl_cloud_out );
    
	pcl_cloud_combined += pcl_cloud_out;
      }
    
    // Convert back to radar message type
    data_conversions::pclCloudToRadarTargetArray( pcl_cloud_combined, msg_combined );
  }

  void RadarCombineFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3 )
  {
    std::vector<ainstein_radar_msgs::RadarTargetArray> msg_arr( 3 );
    msg_arr.at( 0 ) = *msg1;
    msg_arr.at( 1 ) = *msg2;
    msg_arr.at( 2 ) = *msg3;
    
    ainstein_radar_msgs::RadarTargetArray msg_combined;
    combineMsgs( msg_arr, msg_combined );
       
    // Copy metadata from input data and publish
    msg_combined.header.frame_id = output_frame_id_;
    msg_combined.header.stamp = msg1->header.stamp; // use msg1's stamp for now    
    pub_radar_data_.publish( msg_combined ); 
  }

  void RadarCombineFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4 )
  {
    std::vector<ainstein_radar_msgs::RadarTargetArray> msg_arr( 4 );
    msg_arr.at( 0 ) = *msg1;
    msg_arr.at( 1 ) = *msg2;
    msg_arr.at( 2 ) = *msg3;
    msg_arr.at( 3 ) = *msg4;
    
    ainstein_radar_msgs::RadarTargetArray msg_combined;
    combineMsgs( msg_arr, msg_combined );
       
    // Copy metadata from input data and publish
    msg_combined.header.frame_id = output_frame_id_;
    msg_combined.header.stamp = msg1->header.stamp; // use msg1's stamp for now    
    pub_radar_data_.publish( msg_combined ); 
  }

  void RadarCombineFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg5 )
  {
    std::vector<ainstein_radar_msgs::RadarTargetArray> msg_arr( 5 );
    msg_arr.at( 0 ) = *msg1;
    msg_arr.at( 1 ) = *msg2;
    msg_arr.at( 2 ) = *msg3;
    msg_arr.at( 3 ) = *msg4;
    msg_arr.at( 4 ) = *msg5;
    
    ainstein_radar_msgs::RadarTargetArray msg_combined;
    combineMsgs( msg_arr, msg_combined );
       
    // Copy metadata from input data and publish
    msg_combined.header.frame_id = output_frame_id_;
    msg_combined.header.stamp = msg1->header.stamp; // use msg1's stamp for now    
    pub_radar_data_.publish( msg_combined ); 
  }

  void RadarCombineFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg5,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg6 )
  {
    std::vector<ainstein_radar_msgs::RadarTargetArray> msg_arr( 6 );
    msg_arr.at( 0 ) = *msg1;
    msg_arr.at( 1 ) = *msg2;
    msg_arr.at( 2 ) = *msg3;
    msg_arr.at( 3 ) = *msg4;
    msg_arr.at( 4 ) = *msg5;
    msg_arr.at( 5 ) = *msg6;
    
    ainstein_radar_msgs::RadarTargetArray msg_combined;
    combineMsgs( msg_arr, msg_combined );
       
    // Copy metadata from input data and publish
    msg_combined.header.frame_id = output_frame_id_;
    msg_combined.header.stamp = msg1->header.stamp; // use msg1's stamp for now    
    pub_radar_data_.publish( msg_combined ); 
  }
  
  void RadarCombineFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg5,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg6,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg7 )
  {
    std::vector<ainstein_radar_msgs::RadarTargetArray> msg_arr( 7 );
    msg_arr.at( 0 ) = *msg1;
    msg_arr.at( 1 ) = *msg2;
    msg_arr.at( 2 ) = *msg3;
    msg_arr.at( 3 ) = *msg4;
    msg_arr.at( 4 ) = *msg5;
    msg_arr.at( 5 ) = *msg6;
    msg_arr.at( 6 ) = *msg7;
    
    ainstein_radar_msgs::RadarTargetArray msg_combined;
    combineMsgs( msg_arr, msg_combined );
       
    // Copy metadata from input data and publish
    msg_combined.header.frame_id = output_frame_id_;
    msg_combined.header.stamp = msg1->header.stamp; // use msg1's stamp for now    
    pub_radar_data_.publish( msg_combined ); 
  }
  
  void RadarCombineFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg1,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg2,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg3,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg4,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg5,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg6,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg7,
					      const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg8 )
  {
    std::vector<ainstein_radar_msgs::RadarTargetArray> msg_arr( 8 );
    msg_arr.at( 0 ) = *msg1;
    msg_arr.at( 1 ) = *msg2;
    msg_arr.at( 2 ) = *msg3;
    msg_arr.at( 3 ) = *msg4;
    msg_arr.at( 4 ) = *msg5;
    msg_arr.at( 5 ) = *msg6;
    msg_arr.at( 6 ) = *msg7;
    msg_arr.at( 7 ) = *msg8;
    
    ainstein_radar_msgs::RadarTargetArray msg_combined;
    combineMsgs( msg_arr, msg_combined );
       
    // Copy metadata from input data and publish
    msg_combined.header.frame_id = output_frame_id_;
    msg_combined.header.stamp = msg1->header.stamp; // use msg1's stamp for now    
    pub_radar_data_.publish( msg_combined ); 
  }
  
} // namespace ainstein_radar_filters
