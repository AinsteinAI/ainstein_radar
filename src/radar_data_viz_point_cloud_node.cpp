/*
  Copyright <2018> <Ainstein, Inc.>

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

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <radar_sensor_msgs/RadarData.h>

class RadarDataVizPointCloud
{
public:
  RadarDataVizPointCloud( std::string data_topic, std::string vel_topic, std::string pcl_topic )
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
    node_handle_.param( "rel_speed_thresh", rel_speed_thresh_, 0.1 );
    
    // Assume radar velocity is not available until a message is received:
    is_vel_available_ = false;
  }

  ~RadarDataVizPointCloud()
  {
  }

  pcl::PointXYZ radarDataToPclPoint( const radar_sensor_msgs::RadarTarget &target )
  {
    pcl::PointXYZ p;
    p.x = cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
      * target.range;
    p.y = sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
      * target.range;
    p.z = sin( ( M_PI / 180.0 ) * target.elevation ) * target.range;

    return p;
  }

  void radarVelCallback( const geometry_msgs::Twist &msg )
  {
    // Get the radar linear velocity in world frame:
    tf2::fromMsg( msg.linear, vel_world_ );
    is_vel_available_ = true;
  }
    
  void radarDataCallback( const radar_sensor_msgs::RadarData &msg )
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
	    double rel_speed = ( tf_sensor_to_world.linear().inverse() * vel_world_ )( 0 ) + it->speed;
	    // Filter out targets based on relative speed:
	    if( std::abs( rel_speed ) < rel_speed_thresh_ )
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

private:
  std::string data_topic_;
  std::string vel_topic_;
  std::string pcl_topic_;
  std::string frame_id_;

  pcl::PointCloud<pcl::PointXYZ> pcl_;
  sensor_msgs::PointCloud2 cloud_msg_;
    
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_radar_data_;
  ros::Publisher pub_pcl_;

  ros::Subscriber sub_radar_vel_;
  bool is_vel_available_;
  Eigen::Vector3d vel_world_;
  double rel_speed_thresh_;

  tf2_ros::TransformListener listen_tf_;
  tf2_ros::Buffer buffer_tf_;

};

int main( int argc, char** argv )
{
  // Initialize ROS node:
  ros::init( argc, argv, "radar_data_viz_point_cloud_node" );

  // Data viz constructor arguments:
  std::string data_topic;
  std::string vel_topic;
  std::string pcl_topic;
    
  // Parse the command line arguments for radar parameters:
  if( argc < 2 )
    {
      std::cerr << "Usage: rosrun radar_ros_interface radar_data_viz_point_cloud_node --topic TOPIC" << std::endl;
      return -1;
    }

  // Parse the command line arguments:
  for( int i = 0; i < argc; ++i )
    {
      // Check for the data topic name:
      if( std::string( argv[i] ) == std::string( "--topic" ) )
	{
	  data_topic = std::string( argv[++i] );
	}
    }

  if( data_topic.empty() )
    {
      std::cerr << "Data topic name must be set. Usage: rosrun radar_ros_interface radar_data_viz_point_cloud_node --topic TOPIC" << std::endl;
      return -1;
    }

  vel_topic = "car_vel";
  pcl_topic = data_topic + "_pcl";
  
  std::cout << "Running radar data viz point cloud node with data topic: " << data_topic << " radar velocity topic: " << vel_topic << " point cloud topic: " << pcl_topic << std::endl;
    
  // Create visualization node to publish target point cloud:
  RadarDataVizPointCloud data_viz( data_topic, vel_topic, pcl_topic );

  ros::spin();

  return 0;
}
