#include "RadarDataToLaserScan.h"

RadarDataToLaserScan::RadarDataToLaserScan( std::string data_topic, std::string vel_topic, std::string laser_scan_topic )
  : data_topic_( data_topic ),
    vel_topic_( vel_topic ),
    laser_scan_topic_( laser_scan_topic ),
    listen_tf_( buffer_tf_ )
    
{
  pub_laser_scan_ = node_handle_.advertise<sensor_msgs::LaserScan>( laser_scan_topic_, 10 );
  sub_radar_data_ = node_handle_.subscribe( data_topic_, 10,
					    &RadarDataToLaserScan::radarDataCallback,
					    this );
  sub_radar_vel_ = node_handle_.subscribe( vel_topic_, 10,
					   &RadarDataToLaserScan::radarVelCallback,
					   this );

  // Get parameters:
  node_handle_.param( "angle_min", laser_scan_msg_.angle_min, static_cast<float>( -0.5 * M_PI ) );
  node_handle_.param( "angle_max", laser_scan_msg_.angle_max, static_cast<float>( 0.5 * M_PI ) );
  node_handle_.param( "angle_increment", laser_scan_msg_.angle_increment, static_cast<float>( 5.0 * ( M_PI / 180.0 ) ) );
  
  node_handle_.param( "time_increment", laser_scan_msg_.time_increment, static_cast<float>( 0.0 ) );
  node_handle_.param( "scan_time", laser_scan_msg_.scan_time, static_cast<float>( 0.1 ) );

  node_handle_.param( "range_min", laser_scan_msg_.range_min, static_cast<float>( 1.0 ) );
  node_handle_.param( "range_max", laser_scan_msg_.range_max, static_cast<float>( 20.0 ) );
  
  node_handle_.param( "max_speed_thresh", max_speed_thresh_, 1.0 );
  node_handle_.param( "min_dist_thresh", min_dist_thresh_, 1.0 );
  node_handle_.param( "max_dist_thresh", max_dist_thresh_, 20.0 );

  // Set the laser scan message array lengths:
  laser_scan_length_ = static_cast<int>( floor( ( laser_scan_msg_.angle_max -
						  laser_scan_msg_.angle_max ) /
						laser_scan_msg_.angle_increment ) );
  laser_scan_msg_.ranges.resize( laser_scan_length_, 0.0 );
  laser_scan_msg_.intensities.clear(); // do not fill intensities
  
  // Assume radar velocity is not available until a message is received:
  is_vel_available_ = false;
}

void RadarDataToLaserScan::radarVelCallback( const geometry_msgs::Twist &msg )
{
  // Get the radar linear velocity in world frame:
  tf2::fromMsg( msg.linear, vel_world_ );
  is_vel_available_ = true;
}

void RadarDataToLaserScan::radarDataCallback( const radar_sensor_msgs::RadarData &msg )
{
  // Get the data frame ID and look up the corresponding tf transform:
  Eigen::Affine3d tf_sensor_to_world =
    tf2::transformToEigen(buffer_tf_.lookupTransform( "base_link", msg.header.frame_id, ros::Time( 0 ) ) );
  
  // Clear the point laser_scan point vector:
  std::fill( laser_scan_msg_.ranges.begin(), laser_scan_msg_.ranges.begin(), 0.0 );
  std::fill( laser_scan_msg_.intensities.begin(), laser_scan_msg_.intensities.begin(), 0.0 );

  // Iterate through raw targets and add them to the point laser_scan:
  for( auto t : msg.raw_targets )
    {
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
	  
	  // Filter out targets based on project target absolute speed
	  // and range limits:
	  if( std::abs( proj_speed ) < max_speed_thresh_ &&
	      t.range >= min_dist_thresh_ &&
	      t.range <= max_dist_thresh_)
	    {
	      laser_scan_msg_.ranges.push_back( static_cast<float>( t.range ) );
	    }
	}
      else // No velocity information available
	{
	  laser_scan_msg_.ranges.push_back( static_cast<float>( t.range ) );
	}
    }

  laser_scan_msg_.header.frame_id = msg.header.frame_id;
  laser_scan_msg_.header.stamp = msg.header.stamp;

  pub_laser_scan_.publish( laser_scan_msg_ );
} 
