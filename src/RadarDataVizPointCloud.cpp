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
  node_handle_.param( "rel_speed_thresh", rel_speed_thresh_, 1.0 );
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
  Eigen::Affine3d tf_sensor_to_world = tf2::transformToEigen(
							     buffer_tf_.lookupTransform( "base_link", msg.header.frame_id, ros::Time( 0 ) ) );
  
  // First delete, then populate the raw pcls:
  pcl_.clear();
  for( auto it = msg.raw_targets.begin(); it != msg.raw_targets.end(); ++it )
    {
      // Compute the relative speed of target to radar in radar sensor frame:
      if( is_vel_available_ )
	{
	  Eigen::Vector3d meas_dir = Eigen::Vector3d( cos( ( M_PI / 180.0 ) * it->azimuth ) * cos( ( M_PI / 180.0 ) * it->elevation ),
						      sin( ( M_PI / 180.0 ) * it->azimuth ) * cos( ( M_PI / 180.0 ) * it->elevation ),
						      sin( ( M_PI / 180.0 ) * it->elevation ) );

	  double rel_speed = it->speed - meas_dir.dot( tf_sensor_to_world.linear().inverse() * vel_world_ );
	  // Filter out targets based on relative speed:
	  if( std::abs( rel_speed ) < rel_speed_thresh_ &&
	      it->range >= min_dist_thresh_ &&
	      it->range <= max_dist_thresh_) // hack to filter close and far targets
	    {
	      pcl_.points.push_back( radarDataToPclPoint( *it ) );
	    }
	}
      else // do not filter
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
