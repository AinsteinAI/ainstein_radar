#ifndef RADAR_DATA_CONVERSIONS_H_
#define RADAR_DATA_CONVERSIONS_H_

#include <cmath>

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_filters/pcl_point_radar_target.h>
#include <ainstein_radar_drivers/utilities.h>

namespace ainstein_radar_filters
{
  namespace data_conversions
  {    
    static void radarTargetToPclPoint( const ainstein_radar_msgs::RadarTarget& target,
				       PointRadarTarget& pcl_point )
    {
      Eigen::Vector3d p;
      ainstein_radar_drivers::utilities::sphericalToCartesian( target.range,
			    ( M_PI / 180.0 ) * target.azimuth,
			    ( M_PI / 180.0 ) * target.elevation,
			    p );
      pcl_point.x = p.x();
      pcl_point.y = p.y();
      pcl_point.z = p.z();

      // Copy the spherical coordinate data
      pcl_point.snr = target.snr;
      pcl_point.range = target.range;
      pcl_point.speed = target.speed;
      pcl_point.azimuth = target.azimuth;
      pcl_point.elevation = target.elevation;
    }
  
    static void pclPointToRadarTarget( const PointRadarTarget& pcl_point,
				       ainstein_radar_msgs::RadarTarget& target )
    {
      target.snr = pcl_point.snr;
      target.speed = pcl_point.speed;

      double range, azimuth, elevation;
      ainstein_radar_drivers::utilities::cartesianToSpherical( Eigen::Vector3d( pcl_point.x, pcl_point.y, pcl_point.z ),
			    range, azimuth, elevation );
      target.range = range;
      target.azimuth = ( 180.0 / M_PI ) * azimuth;
      target.elevation = ( 180.0 / M_PI ) * elevation;
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

    static void pclCloudToRadarTargetArray( const pcl::PointCloud<PointRadarTarget>& pcl_cloud ,
					    ainstein_radar_msgs::RadarTargetArray& target_array )
    {
      // Clear the targets array
      target_array.targets.clear();

      // Copy the header info
      pcl_conversions::fromPCL( pcl_cloud.header, target_array.header );
      
      // Iterate through point cloud point targets and add them to the target array
      ainstein_radar_msgs::RadarTarget target;
      target.target_id = 0;
      for( auto pcl_point : pcl_cloud.points )
	{
	  pclPointToRadarTarget( pcl_point, target );
	  ++target.target_id;
	
	  target_array.targets.push_back( target );
	}
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

    
    static void rosCloudToRadarTargetArray( const sensor_msgs::PointCloud2& ros_cloud,
					    ainstein_radar_msgs::RadarTargetArray& target_array )
    {
      // Convert to PCL cloud and use the appropriate conversion function above
      pcl::PointCloud<PointRadarTarget> pcl_cloud;
      pcl::fromROSMsg( ros_cloud, pcl_cloud );

      pclCloudToRadarTargetArray( pcl_cloud, target_array );
    }

    /* static void transformRadarTargetArray( const std::string& target_frame, */
    /* 					   const ainstein_radar_msgs::RadarTargetArray& radar_in, */
    /* 					   ainstein_radar_msgs::RadarTargetArray& radar_out, */
    /* 					   const tf2_ros::Buffer& buffer ) */
    /* { */
    /*   pcl::PointCloud<PointRadarTarget> cloud_in; */
    /*   radarTargetArrayToPclCloud( radar_in, cloud_in ); */

    /*   pcl::PointCloud<PointRadarTarget> cloud_out; */
    /*   pcl_ros::transformPointCloud( target_frame, cloud_in, cloud_out, buffer ); */

    /*   pclCloudToRadarTargetArray( cloud_out, radar_out ); */
    /* } */

  } // namespace radar_data_conversions
  
} // namespace ainstein_radar_filters

#endif
