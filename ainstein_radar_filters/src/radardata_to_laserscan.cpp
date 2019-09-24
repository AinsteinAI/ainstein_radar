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

#include "ainstein_radar_filters/radardata_to_laserscan.h"

namespace ainstein_radar_filters
{

RadarDataToLaserScan::RadarDataToLaserScan( void ) :
  nh_private_( "~" ),
  listen_tf_( buffer_tf_ )
{
  // Get parameters:
  nh_private_.param( "angle_min", laser_scan_msg_.angle_min, static_cast<float>( -0.5 * M_PI ) );
  nh_private_.param( "angle_max", laser_scan_msg_.angle_max, static_cast<float>( 0.5 * M_PI ) );
  nh_private_.param( "angle_increment", laser_scan_msg_.angle_increment, static_cast<float>( 1.0 * ( M_PI / 180.0 ) ) );
  
  nh_private_.param( "time_increment", laser_scan_msg_.time_increment, static_cast<float>( 0.0 ) );
  nh_private_.param( "scan_time", laser_scan_msg_.scan_time, static_cast<float>( 0.1 ) );

  nh_private_.param( "range_min", laser_scan_msg_.range_min, static_cast<float>( 1.0 ) );
  nh_private_.param( "range_max", laser_scan_msg_.range_max, static_cast<float>( 20.0 ) );
  
  nh_private_.param( "rel_speed_thresh", rel_speed_thresh_, 2.0 );

  // Set the laser scan message array lengths:
  laser_scan_length_ = static_cast<int>( std::floor( ( laser_scan_msg_.angle_max -
							laser_scan_msg_.angle_min ) /
						     laser_scan_msg_.angle_increment ) ) + 1;
  laser_scan_msg_.ranges.resize( laser_scan_length_, std::numeric_limits<float>::infinity() );
  laser_scan_msg_.intensities.clear(); // do not fill intensities
  
  // Assume radar velocity is not available until a message is received:
  is_vel_available_ = false;

  // Create the LaserScan publisher:
  pub_laser_scan_ = nh_private_.advertise<sensor_msgs::LaserScan>( "scan", 10 );

  // Subscribe to radar data and radar velocity topics:
  sub_radar_data_ = nh_.subscribe( "radardata_in", 10,
				   &RadarDataToLaserScan::radarDataCallback,
				   this );
  sub_radar_vel_ = nh_.subscribe( "radar_vel", 10,
				  &RadarDataToLaserScan::radarVelCallback,
				  this );
}

void RadarDataToLaserScan::radarVelCallback( const geometry_msgs::Twist &msg )
{
  // Get the radar linear velocity in world frame:
  tf2::fromMsg( msg.linear, vel_world_ );
  is_vel_available_ = true;
}

void RadarDataToLaserScan::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray &msg )
{
  // Get the data frame ID and look up the corresponding tf transform:
  Eigen::Affine3d tf_sensor_to_world =
    tf2::transformToEigen(buffer_tf_.lookupTransform( "map", msg.header.frame_id, ros::Time( 0 ) ) );
  
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

	  // If the radar world frame velocity is available from another source, use it for further processing:
	  if( is_vel_available_ )
	    {
	      // Compute the velocity of the radar rotated into instantaneous radar frame:
	      Eigen::Vector3d vel_radar = -tf_sensor_to_world.linear().inverse() * vel_world_;
	  
	      // Compute the unit vector along the axis between sensor and target:
	      // n = [cos(azi)*cos(elev), sin(azi)*cos(elev), sin(elev)]
	      Eigen::Vector3d meas_dir = Eigen::Vector3d( cos( ( M_PI / 180.0 ) * t.azimuth ) * cos( ( M_PI / 180.0 ) * t.elevation ),
							  sin( ( M_PI / 180.0 ) * t.azimuth ) * cos( ( M_PI / 180.0 ) * t.elevation ),
							  sin( ( M_PI / 180.0 ) * t.elevation ) );

	      // Radar measures relative speed s of target along meas_dir.:
	      // s = n^{T} * R^{T} * ( v_{T} - v_{car} )
	      // We wish to filter out targets with nonzero world frame
	      // velocity v_{T}, but can only compute the projection of v_{T}:
	      // v_{T,proj} = n^{T} * R^{T} * v_{T} = s + n^{T} * R^{T} * v_{car}
	      double proj_speed = t.speed - meas_dir.dot( tf_sensor_to_world.linear().inverse() * vel_world_ );
	  
	      // Filter out targets based on project target absolute speed limit:
	      if( std::abs( proj_speed ) < rel_speed_thresh_ )
		{
		  if( t.range < laser_scan_msg_.ranges.at( beam_ind ) )
		    {
		      laser_scan_msg_.ranges.at( beam_ind ) = static_cast<float>( t.range );
		    }
		}
	    }
	  else // No velocity information available
	    {
	      if( t.range < laser_scan_msg_.ranges.at( beam_ind ) )
		{
		  laser_scan_msg_.ranges.at( beam_ind ) = static_cast<float>( t.range );
		}
	    }
	}
    }

  // Set the message header and publish:
  laser_scan_msg_.header.frame_id = msg.header.frame_id;
  laser_scan_msg_.header.stamp = msg.header.stamp;

  pub_laser_scan_.publish( laser_scan_msg_ );
} 

bool RadarDataToLaserScan::useTarget( const ainstein_radar_msgs::RadarTarget &t )
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
