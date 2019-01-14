#include "RadarDataVizPointCloud.h"

RadarDataVizPointCloud::RadarDataVizPointCloud( std::string data_topic, std::string vel_topic, std::string pcl_topic )
  : data_topic_( data_topic ),
    vel_topic_( vel_topic ),
    pcl_topic_( pcl_topic ),
    listen_tf_( buffer_tf_ )
    
{
  pub_pcl_ = node_handle_.advertise<sensor_msgs::PointCloud2>( pcl_topic_, 10 );
  sub_radar_data_ = node_handle_.subscribe( data_topic_, 10,
					    &RadarDataVizPointCloud::radarDataCallback,
					    this );
  sub_radar_vel_ = node_handle_.subscribe( vel_topic_, 10,
					   &RadarDataVizPointCloud::radarVelCallback,
					   this );

  // Get parameters:
  node_handle_.param( "max_speed_thresh", max_speed_thresh_, 1.0 );
  node_handle_.param( "min_dist_thresh", min_dist_thresh_, 1.0 );
  node_handle_.param( "max_dist_thresh", max_dist_thresh_, 20.0 );
    
  // Assume radar velocity is not available until a message is received:
  is_vel_available_ = false;
}

void RadarDataVizPointCloud::radarVelCallback( const geometry_msgs::Twist &msg )
{
  // Get the radar linear velocity in world frame:
  tf2::fromMsg( msg.linear, vel_world_ );
  is_vel_available_ = true;
}

void RadarDataVizPointCloud::radarDataCallback( const radar_sensor_msgs::RadarData &msg )
{
  // Get the data frame ID and look up the corresponding tf transform:
  Eigen::Affine3d tf_sensor_to_world =
    tf2::transformToEigen(buffer_tf_.lookupTransform( "base_link", msg.header.frame_id, ros::Time( 0 ) ) );
  
  // Clear the point cloud point vector:
  pcl_.clear();

  // Iterate through raw targets and add them to the point cloud:
  for( auto it = msg.raw_targets.begin(); it != msg.raw_targets.end(); ++it )
    {
      // If the radar world frame velocity is available from another source, use it for further processing:
      if( is_vel_available_ )
	{
	  // Copy the original radar target for further processing:
	  radar_sensor_msgs::RadarTarget t = *it;
	  
	  // Compute the velocity of the radar rotated into instantaneous radar frame:
	  Eigen::Vector3d vel_radar = tf_sensor_to_world.linear().inverse() * vel_world_;
	  
	  // Use the radar velocity information and target relative speed to compute elevation:
	  // s = n^{T} * R_{W}^{R} * ( v_{T}^{W} - v_{R}^{W} )
	  // s = -n^{T} * R_{W}^{R} * v_{R}^{W} assuming v_{T}^{W} = 0 (static target)
	  // s = [-cos(a) * cos(e), -sin(a) * cos(e), -sin(e)] * v_{R}^{R}
	  // s = -v_{T,x}^{R} * cos(a) * cos(e)
	  //     - v_{T,y}^{R} * sin(a) * cos(e)
	  //	 - v_{T,z}^{R} * sin(e)
	  // 
	  t.elevation = ( 180.0 / M_PI ) * acos( t.speed / ( -vel_radar( 0 ) * cos( ( M_PI / 180.0 ) * t.azimuth )
							     - vel_radar( 1 ) * sin( ( M_PI / 180.0 ) * t.azimuth ) ) ) - 90.0;
	  
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
	      pcl_.points.push_back( radarDataToPclPoint( t ) );
	    }
	}
      else // No velocity information available
	{
	  pcl_.points.push_back( radarDataToPclPoint( *it ) );
	}
    }

  pcl_.width = pcl_.points.size();
  pcl_.height = 1;
	
  pcl::toROSMsg( pcl_, cloud_msg_ );
  cloud_msg_.header.frame_id = msg.header.frame_id;
  cloud_msg_.header.stamp = msg.header.stamp;

  pub_pcl_.publish( cloud_msg_ );
} 

pcl::PointXYZ RadarDataVizPointCloud::radarDataToPclPoint( const radar_sensor_msgs::RadarTarget &target )
{
  pcl::PointXYZ p;
  p.x = cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
    * target.range;
  p.y = sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
    * target.range;
  p.z = sin( ( M_PI / 180.0 ) * target.elevation ) * target.range;

  return p;
}
