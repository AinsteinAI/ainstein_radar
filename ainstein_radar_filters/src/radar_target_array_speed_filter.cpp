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

#include "ainstein_radar_filters/radar_target_array_speed_filter.h"

namespace ainstein_radar_filters
{
  
RadarTargetArraySpeedFilter::RadarTargetArraySpeedFilter( ros::NodeHandle node_handle,
					      ros::NodeHandle node_handle_private ) :
  nh_( node_handle ),
  nh_private_( node_handle_private ),
  listen_tf_( buffer_tf_ )
{
  sub_radar_data_ = nh_.subscribe( "radardata_in", 10,
				   &RadarTargetArraySpeedFilter::radarDataCallback,
				   this );
  sub_radar_vel_ = nh_.subscribe( "radar_vel", 10,
				  &RadarTargetArraySpeedFilter::radarVelCallback,
				  this );
  
  pub_radar_data_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "radardata_out", 10 );

  // Get parameters:
  if( nh_private_.hasParam( "min_speed_thresh" ) )
    {
      nh_private_.param( "min_speed_thresh", min_speed_thresh_, 1.0 );
      filter_stationary_ = true;
    }
  else
    {
      filter_stationary_ = false;
    }
  if( nh_private_.hasParam( "max_speed_thresh" ) )
    {
      nh_private_.param( "max_speed_thresh", max_speed_thresh_, 1.0 );
      filter_moving_ = true;
    }
  else
    {
      filter_moving_ = false;
    }
  
  nh_private_.param( "compute_3d", compute_3d_, false );
  nh_private_.param( "is_rotated", is_rotated_, false );
  
  // Assume radar velocity is not available until a message is received:
  is_vel_available_ = false;
}

void RadarTargetArraySpeedFilter::radarVelCallback( const geometry_msgs::Twist &msg )
{
  // Get the radar linear velocity in world frame:
  tf2::fromMsg( msg.linear, vel_world_ );
  is_vel_available_ = true;
}

void RadarTargetArraySpeedFilter::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray &msg )
{
  // Get the data frame ID and look up the corresponding tf transform:
  Eigen::Affine3d tf_sensor_to_world;
  if( buffer_tf_.canTransform( "map", msg.header.frame_id, ros::Time( 0 ) ) )
    {
      tf_sensor_to_world =
	tf2::transformToEigen(buffer_tf_.lookupTransform( "map", msg.header.frame_id, ros::Time( 0 ) ) );
    }
  else
    {
      std::cout << "Timeout while waiting for transform to frame " << msg.header.frame_id << std::endl;
      return;
    }
  
  // Clear the output radar data message:
  msg_filtered_.targets.clear();

  ainstein_radar_msgs::RadarTarget target_filtered;
  int target_id = 0;
  for( auto target : msg.targets )
    {
      // If the radar world frame velocity is available from another source, use it for further processing:
      if( is_vel_available_ )
	{
	  // Copy the original radar target for further processing:
	  ainstein_radar_msgs::RadarTarget t = target;

	  // Copy azimuth to elevation if the radar has been rotated:
	  // In radar frame, +ve azimuth is LEFT, +ve elevation is DOWN.
	  // When rotated -90* about x (clockwise from front), swap azimuth and elevation:
	  if( is_rotated_ )
	    {
	      t.elevation = t.azimuth; // +ve azimith maps to +ve elevation when rotated
	      t.azimuth = 0.0;
	    }
	  
	  // Compute the velocity of the radar rotated into instantaneous radar frame:
	  Eigen::Vector3d vel_radar = -tf_sensor_to_world.linear().inverse() * vel_world_;

	  if( compute_3d_ )
	    {
	      // Use the radar velocity information and target relative speed to compute
	      // either election or azimuth (if rotated):
	      //
	      // s = n^{T} * R_{W}^{R} * ( v_{T}^{W} - v_{R}^{W} )
	      // s = -n^{T} * R_{W}^{R} * v_{R}^{W} assuming v_{T}^{W} = 0 (static target)
	      // -s = [cos(a) * cos(e), sin(a) * cos(e), sin(e)] * v_{R}^{R}
	      // -s = v_{T,x}^{R} * cos(a) * cos(e) +
	      //      v_{T,y}^{R} * sin(a) * cos(e) + 
	      //   	  v_{T,z}^{R} * sin(e)
	      //
	      // In either case, we solve an equation of the form x*cos(th)+y*sin(th)=z
	      // which has infinite solutions of the form:
	      //
	      // th = 2*arctan((1/(x+z)) * (y+/-sqrt(y^2+x^2-z^2))) + 2*pi*n for all n
	      //
	      // We take n=0 and choose the minimum (abs) of the two solutions
	      //
	      // th_m = 2*arctan((1/(x+z)) * (y-sqrt(y^2+x^2-z^2)))
	      // th_p = 2*arctan((1/(x+z)) * (y+sqrt(y^2+x^2-z^2)))

	      double x, y, z;
	      if( is_rotated_ )
		{
		  // Assuming elevation is known, solve for azimuth:
		  //
		  // v_{T,x}^{R} * cos(e) * cos(a) + v_{T,y}^{R} * cos(e) * sin(a) = -s - v_{T,z}^{R} * sin(e)
		  //
		  // Put in form x * cos(e) + y * sin(e) = z:
		  // x = v_{T,x}^{R} * cos(e)
		  // y = v_{T,y}^{R} * cos(e)
		  // z = -s - v_{T,z}^{R} * sin(e)
		  
		  x = vel_radar( 0 ) * cos( ( M_PI / 180.0 ) * t.elevation );
		  y = vel_radar( 1 ) * cos( ( M_PI / 180.0 ) * t.elevation );
		  z = t.speed - vel_radar( 2 ) * sin( ( M_PI / 180.0 ) * t.elevation ); // should be -t.speed with correct speed sign, fix
		  
		  t.azimuth = solveForAngle( x, y, z );
		}
	      else
		{
		  // Assuming azimuth is known, solve for elevation:
		  //
		  // ( v_{T,x}^{R} * cos(a) + v_{T,y}^{R} * sin(a) ) * cos(e) + v_{T,z}^{R} * sin(e) = -s
		  //
		  // Put in form x * cos(e) + y * sin(e) = z:
		  // x = v_{T,x}^{R} * cos(a) + v_{T,y}^{R} * sin(a)
		  // y = v_{T,z}^{R}
		  // z = -s
		  
		  x = vel_radar( 0 ) * cos( ( M_PI / 180.0 ) * t.azimuth ) +
		    vel_radar( 1 ) * sin( ( M_PI / 180.0 ) * t.azimuth );
		  y = vel_radar( 2 ); // should be zero since we ignore vz for now
		  z = t.speed; // should be -t.speed with correct speed sign, fix
		  
		  t.elevation = solveForAngle( x, y, z );
		}
	    }
	  
	  // Compute the unit vector along the axis between sensor and target:
	  // n = [cos(azi)*cos(elev), sin(azi)*cos(elev), sin(elev)]
	  Eigen::Vector3d meas_dir = Eigen::Vector3d( cos( ( M_PI / 180.0 ) * t.azimuth ) * cos( ( M_PI / 180.0 ) * t.elevation ),
						      sin( ( M_PI / 180.0 ) * t.azimuth ) * cos( ( M_PI / 180.0 ) * t.elevation ),
						      sin( ( M_PI / 180.0 ) * t.elevation ) );
	  
	  // Radar measures relative speed s of target w/r/t radar along meas_dir:
	  //
	  // s = n^{T} * R^{T} * ( v_{T} - v_{radar} ) where n = radar axis unit vec
	  //
	  // We wish to filter out targets with nonzero world frame velocity v_{T},
	  // but can only compute the projection of v_{T}:
	  //
	  // v_{T,proj} = n^{T} * R^{T} * v_{T} = s + n^{T} * R^{T} * v_{radar}
	  //
	  // Assuming +ve speed is AWAY from radar and -ve speed is TOWARDS radar:
	  double proj_speed = t.speed + meas_dir.dot( tf_sensor_to_world.linear().inverse() * vel_world_ );
	  
	  // Filter out targets based on projected target absolute speed:
	  if( filter_moving_ && std::abs( proj_speed ) < max_speed_thresh_ )
	    {
	      target_filtered = target;
	      target_filtered.target_id = target_id;
	      msg_filtered_.targets.push_back( target_filtered );
	      ++target_id;
	    }
	  else if( filter_stationary_ && std::abs( proj_speed ) > min_speed_thresh_ )
	    {
	      target_filtered = target;
	      target_filtered.target_id = target_id;
	      msg_filtered_.targets.push_back( target_filtered );
	      ++target_id;
	    }
	}
      else // No velocity information available
	{
	  msg_filtered_.targets.push_back( target );
	}
    }
  
  msg_filtered_.header.frame_id = msg.header.frame_id;
  msg_filtered_.header.stamp = msg.header.stamp;
  pub_radar_data_.publish( msg_filtered_ );
}

double RadarTargetArraySpeedFilter::solveForAngle( double x, double y, double z )
{
  // Solve x*cos(th)+y*sin(th)=z for th:
  //
  // th = 2*arctan((1/(x+z)) * (y+/-sqrt(y^2+x^2-z^2))) + 2*pi*n for all n
  //
  // We take n=0 and choose the minimum (abs) of the two solutions
  //
  // th_m = 2*arctan((1/(x+z)) * (y-sqrt(y^2+x^2-z^2)))
  // th_p = 2*arctan((1/(x+z)) * (y+sqrt(y^2+x^2-z^2)))
  //
  if( ( y * y + x * x ) < ( z * z ) ) // sqrt returns -nan
    {
      return 0.0;
    }
  else
    {    
      double th_p = 2.0 * atan( ( y + sqrt( y * y + x * x - z * z ) ) / ( x + z ) );
      double th_m = 2.0 * atan( ( y - sqrt( y * y + x * x - z * z ) ) / ( x + z ) );
  
      return -( 180.0 / M_PI ) * ( std::abs( th_p ) < std::abs( th_m ) ? th_p : th_m ); // negative here from comparing to Yan's, not sure why needed yet...
    }
}

} // namespace ainstein_radar_filters
