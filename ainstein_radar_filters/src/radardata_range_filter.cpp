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

#include "ainstein_radar_filters/radardata_range_filter.h"

namespace ainstein_radar_filters
{
  RadarDataRangeFilter::RadarDataRangeFilter( const ros::NodeHandle& node_handle,
					      const ros::NodeHandle& node_handle_private ) :
    nh_( node_handle ),
    nh_private_( node_handle_private )
  {
    pub_radar_data_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "radar_out", 10 );
    sub_radar_data_ = nh_.subscribe( "radar_in", 10,
				     &RadarDataRangeFilter::radarDataCallback,
				     this );
    
    // Set up dynamic reconfigure:
    dynamic_reconfigure::Server<ainstein_radar_filters::RangeFilterConfig>::CallbackType f;
    f = boost::bind(&RadarDataRangeFilter::dynConfigCallback, this, _1, _2);
    dyn_config_server_.setCallback(f);
  }

  void RadarDataRangeFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray& msg )
  {
    // Iterate through targets and add them to the point cloud:
    ainstein_radar_msgs::RadarTarget target_filtered;
    int target_id = 0; // keep track of new target IDs

    msg_filtered_.targets.clear();
    for( const auto& target : msg.targets )
      {
	// Filter based on specified range limits:
	if( target.range >= config_.min_range &&
	    target.range <= config_.max_range)
	  {
	    target_filtered = target;
	    target_filtered.target_id = target_id;
	    msg_filtered_.targets.push_back( target );
	    ++target_id;
	  }
      }

    // Copy metadata from input data and publish:
    msg_filtered_.header = msg.header;
    pub_radar_data_.publish( msg_filtered_ );
  }

} // namespace ainstein_radar_filters
