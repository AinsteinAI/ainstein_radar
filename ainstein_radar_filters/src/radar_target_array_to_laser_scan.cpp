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

#include "ainstein_radar_filters/radar_target_array_to_laser_scan.h"

namespace ainstein_radar_filters
{
  
  RadarTargetArrayToLaserScan::RadarTargetArrayToLaserScan( ros::NodeHandle node_handle,
							    ros::NodeHandle node_handle_private ) :
    nh_( node_handle ),
    nh_private_( node_handle_private )
  {
    // Get parameters:
    nh_private_.param( "angle_min", laser_scan_msg_.angle_min, static_cast<float>( -0.5 * M_PI ) );
    nh_private_.param( "angle_max", laser_scan_msg_.angle_max, static_cast<float>( 0.5 * M_PI ) );
    nh_private_.param( "angle_increment", laser_scan_msg_.angle_increment, static_cast<float>( 1.0 * ( M_PI / 180.0 ) ) );
  
    nh_private_.param( "time_increment", laser_scan_msg_.time_increment, static_cast<float>( 0.0 ) );
    nh_private_.param( "scan_time", laser_scan_msg_.scan_time, static_cast<float>( 0.1 ) );

    nh_private_.param( "range_min", laser_scan_msg_.range_min, static_cast<float>( 0.0 ) );
    nh_private_.param( "range_max", laser_scan_msg_.range_max, static_cast<float>( 100.0 ) );
  
    // Set the laser scan message array lengths:
    laser_scan_length_ = static_cast<int>( std::floor( ( laser_scan_msg_.angle_max -
							 laser_scan_msg_.angle_min ) /
						       laser_scan_msg_.angle_increment ) ) + 1;
    laser_scan_msg_.ranges.resize( laser_scan_length_, std::numeric_limits<float>::infinity() );
    laser_scan_msg_.intensities.resize( laser_scan_length_, 0.0 );

    // Subscribe to radar data and radar velocity topics:
    sub_radar_data_ = nh_.subscribe( "radar_in", 10,
				     &RadarTargetArrayToLaserScan::radarDataCallback,
				     this );

    // Create the LaserScan publisher:
    pub_laser_scan_ = nh_private_.advertise<sensor_msgs::LaserScan>( "scan_out", 10 );
  }

  void RadarTargetArrayToLaserScan::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray &msg )
  {
    // Clear the point laser_scan point vector:
    std::fill( laser_scan_msg_.ranges.begin(), laser_scan_msg_.ranges.end(), std::numeric_limits<float>::infinity() );
    std::fill( laser_scan_msg_.intensities.begin(), laser_scan_msg_.intensities.end(), 0.0 );

    // Iterate through targets and add them to the point laser_scan:
    for( auto t : msg.targets )
      {
	if( useTarget( t ) )
	  {
	    // Compute the laser scan beam index from target azimuth angle:
	    unsigned int beam_ind = static_cast<int>( std::floor( ( ( M_PI / 180.0 ) * t.azimuth - laser_scan_msg_.angle_min ) / laser_scan_msg_.angle_increment ) );

	    // Update the range at this index iff it's smaller than the current range:
	    if( t.range < laser_scan_msg_.ranges.at( beam_ind ) )
	      {
		laser_scan_msg_.ranges.at( beam_ind ) = static_cast<float>( t.range );
		laser_scan_msg_.intensities.at( beam_ind ) = static_cast<float>( t.snr );
	      }
	  }
      }

    // Set the message header and publish:
    laser_scan_msg_.header.frame_id = msg.header.frame_id;
    laser_scan_msg_.header.stamp = msg.header.stamp;

    pub_laser_scan_.publish( laser_scan_msg_ );
  } 

  bool RadarTargetArrayToLaserScan::useTarget( const ainstein_radar_msgs::RadarTarget &t )
  {
    // Check that target range and azimuth are within bounds:
    if( t.range <= laser_scan_msg_.range_min ||
	t.range >= laser_scan_msg_.range_max ||
	( ( M_PI / 180.0 ) * t.azimuth ) <= laser_scan_msg_.angle_min ||
	( ( M_PI / 180.0 ) * t.azimuth ) >= laser_scan_msg_.angle_max )
      {
	return false;
      }

    return true;
  }

} // namespace ainstein_radar_filters
