#ifndef RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H
#define RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_filters/pcl_point_radar_target.h>

namespace ainstein_radar_filters
{

class RadarTargetArrayToPointCloud
{
public:
  RadarTargetArrayToPointCloud() {}
  ~RadarTargetArrayToPointCloud() {}

  // Made these functions static to expose radar data to 3d point conversion, however these should
  // probably be moved to a utilities class with other such simple conversions for data types.
  static void radarTargetToPclPoint( const ainstein_radar_msgs::RadarTarget& target,
				     PointRadarTarget& pcl_point )
  {
    pcl_point.x = cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
      * target.range;
    pcl_point.y = sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
      * target.range;
    pcl_point.z = sin( ( M_PI / 180.0 ) * target.elevation ) * target.range;

    pcl_point.snr = target.snr;
    pcl_point.range = target.range;
    pcl_point.speed = target.speed;
    pcl_point.azimuth = target.azimuth;
    pcl_point.elevation = target.elevation;
  }
  
  static void radarTargetArrayToPclCloud( const ainstein_radar_msgs::RadarTargetArray& target_array,
					  pcl::PointCloud<PointRadarTarget>& pcl_cloud )
  {
    // Clear the PCL point cloud
    pcl_cloud.clear();
    
    // Iterate through targets and add them to the point cloud
    PointRadarTarget pcl_point;
    for( auto target : target_array.targets )
      {
	radarTargetToPclPoint( target, pcl_point );
	pcl_cloud.points.push_back( pcl_point );
      }

    pcl_cloud.width = pcl_cloud.points.size();
    pcl_cloud.height = 1;
  } 

  static void radarTargetArrayToROSCloud( const ainstein_radar_msgs::RadarTargetArray& target_array,
					  sensor_msgs::PointCloud2& ros_cloud )
  {
    pcl::PointCloud<PointRadarTarget> pcl_cloud;
    radarTargetArrayToPclCloud( target_array, pcl_cloud );

    pcl::toROSMsg( pcl_cloud, ros_cloud );
    ros_cloud.header.frame_id = target_array.header.frame_id;
    ros_cloud.header.stamp = target_array.header.stamp;
  }
  
};
  
} // namespace ainstein_radar_filters

#endif // RADAR_TARGET_ARRAY_TO_POINT_CLOUD_H
