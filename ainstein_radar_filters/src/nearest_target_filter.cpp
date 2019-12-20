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

#include "ainstein_radar_filters/nearest_target_filter.h"

namespace ainstein_radar_filters
{
  NearestTargetFilter::NearestTargetFilter( ros::NodeHandle node_handle,
						      ros::NodeHandle node_handle_private ) :
    nh_( node_handle ),
    nh_private_( node_handle_private )
  {
    sub_radar_data_ = nh_.subscribe( "radar_in", 10,
				     &NearestTargetFilter::radarTargetArrayCallback,
				     this );

    pub_nearest_target_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetStamped>( "nearest_target", 10 );

    pub_nearest_target_data_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "nearest_target_array", 10 );

    // Get parameters:  
    nh_private_.param( "min_dist_thresh", min_dist_thresh_, 0.0 );
    nh_private_.param( "max_dist_thresh", max_dist_thresh_, 100.0 );

    filter_data_ = false;
    if( nh_private_.hasParam( "data_lpf_alpha" ) )
      {
	nh_private_.param( "data_lpf_alpha", data_lpf_alpha_, 0.1 );
	filter_data_ = true;
	is_filter_init_ = false;
      }
  
    nh_private_.param( "data_lpf_timeout", data_lpf_timeout_, 3.0 );
  }

  void NearestTargetFilter::radarTargetArrayCallback( const ainstein_radar_msgs::RadarTargetArray::Ptr &msg )
  {
    nearest_target_array_msg_.targets.clear();
    double nearest_range = 1000.0;
    for( auto target : msg->targets )
      {
	// Filter first based on specified range limits:
	if( ( target.range >= min_dist_thresh_ ) &&
	    ( target.range <= max_dist_thresh_ ) )
	  {
	    // Determine whether to re-initialize tracked target:
	    if( ( ros::Time::now() - time_prev_ ).toSec() > data_lpf_timeout_ )
	      {
		is_filter_init_ = false;
	      }

	    // Check if nearest so far:
	    if( nearest_range > target.range )
	      {
		// Low-pass filter data:
		if( filter_data_ )
		  {
		    if( is_filter_init_ )
		      {
			nearest_target_msg_.target.range = data_lpf_alpha_ * target.range
			  + ( 1.0 - data_lpf_alpha_ ) * nearest_target_msg_.target.range;
			  
			nearest_target_msg_.target.speed = data_lpf_alpha_ * target.speed
			  + ( 1.0 - data_lpf_alpha_ ) * nearest_target_msg_.target.speed;

			nearest_target_msg_.target.azimuth = data_lpf_alpha_ * target.azimuth
			  + ( 1.0 - data_lpf_alpha_ ) * nearest_target_msg_.target.azimuth;

			nearest_target_msg_.target.elevation = data_lpf_alpha_ * target.elevation
			  + ( 1.0 - data_lpf_alpha_ ) * nearest_target_msg_.target.elevation;
		      }
		    else
		      {
			nearest_target_msg_.target = target;
			is_filter_init_ = true;
		      }
		  }
		else
		  {
		    nearest_target_msg_.target = target;
		  }
	      
		nearest_range = target.range;
	      }
	  }
      }
  
    if( nearest_target_msg_.target.range < 1000.0 )
      {
	nearest_target_msg_.header = msg->header;
	pub_nearest_target_.publish( nearest_target_msg_ );

	nearest_target_array_msg_.header = msg->header;
	nearest_target_array_msg_.targets.push_back( nearest_target_msg_.target );
	pub_nearest_target_data_.publish( nearest_target_array_msg_ );
      }

    time_prev_ = ros::Time::now();
  }

} // namespace ainstein_radar_filters
