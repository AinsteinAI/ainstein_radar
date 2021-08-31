#ifndef RADAR_DRIVER_UTILITIES_H_
#define RADAR_DRIVER_UTILITIES_H_

#include "ainstein_radar_drivers/radar_target.h"
#include "ainstein_radar_drivers/bounding_box.h"
#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_msgs/BoundingBoxArray.h>
#include <ainstein_radar_msgs/RadarTarget.h>
#include <ainstein_radar_msgs/RadarTrackedObject.h>
#include <ainstein_radar_msgs/BoundingBox.h>
#include <tf2_eigen/tf2_eigen.h>

namespace ainstein_radar_drivers
{
  namespace utilities
  {
    static void sphericalToCartesian( double range, double azimuth, double elevation,
				      Eigen::Vector3d& p )
    {
      // Convert spherical coordinates to Cartesian coordinates. Range and point xyz are in
      // meters, angles are in radians.
      p.x() = range * cos( azimuth ) * cos( elevation );
      p.y() = range * sin( azimuth ) * cos( elevation );
      p.z() = range * sin( elevation );
    }

    static void cartesianToSpherical( const Eigen::Vector3d& p,
				      double& range, double& azimuth, double& elevation )
    {
      // Convert Cartesian coordinates to spherical coordinates. Range and point xyz are in
      // meters, angles are in radians.
      range = std::sqrt( std::pow( p.x(), 2.0 ) +
			 std::pow( p.y(), 2.0 ) +
			 std::pow( p.z(), 2.0 ) );
      azimuth = std::atan2( p.y(), p.x() );
      elevation = std::asin( p.z() / range );
    }

    static Eigen::Matrix3d velocityToRotation( const Eigen::Vector3d& vel )
    {
      // Compute the pose assuming the +x direction is the current estimated Cartesian
      // velocity direction:                         
      Eigen::Matrix3d rot_mat;
      if( vel.norm() < 1e-3 ) // handle degenerate case of near-zero velocity               
	{
	  rot_mat = Eigen::Matrix3d::Identity();
	}
      else
	{
	  rot_mat.col( 0 ) = vel / vel.norm();
	  rot_mat.col( 1 ) = Eigen::Vector3d::UnitZ().cross( rot_mat.col( 0 ) );
	  rot_mat.col( 2 ) = rot_mat.col( 0 ).cross( rot_mat.col( 1 ) );
	}

      return rot_mat;
    }

    static geometry_msgs::Pose posVelToPose( const Eigen::Vector3d& pos, const Eigen::Vector3d& vel )
    {
      Eigen::Affine3d pose_eigen;
      pose_eigen.translation() = pos;
      pose_eigen.linear() = velocityToRotation( vel );
      return tf2::toMsg( pose_eigen );
    }

    static ainstein_radar_msgs::RadarTarget targetToROSMsg( const ainstein_radar_drivers::RadarTarget &t )
      {
	ainstein_radar_msgs::RadarTarget target;
	target.target_id = t.id;
	target.range = t.range;
	target.speed = t.speed;
	target.azimuth = t.azimuth;
	target.elevation = t.elevation;
	target.snr = t.snr;

	return target;
      }
  
    static ainstein_radar_msgs::RadarTrackedObject targetToObjectROSMsg( const ainstein_radar_drivers::RadarTarget &t )
      {
	ainstein_radar_msgs::RadarTrackedObject obj_msg;
    
	// Pass through the target ID:
	obj_msg.id = t.id;

	// Convert spherical coordinates to 3d pos and vel and then pose msg:
	Eigen::Vector3d pos, vel;
	utilities::sphericalToCartesian( t.range, ( M_PI / 180.0 ) * t.azimuth, ( M_PI / 180.0 ) * t.elevation, pos );
	utilities::sphericalToCartesian( t.speed, ( M_PI / 180.0 ) * t.azimuth, ( M_PI / 180.0 ) * t.elevation, vel );
	obj_msg.pose = utilities::posVelToPose( pos, vel );

	// Fill in the velocity information:
	obj_msg.velocity.linear.x = vel.x();
	obj_msg.velocity.linear.y = vel.y();
	obj_msg.velocity.linear.z = vel.z();

	// Fill in dummy bounding box information:
	obj_msg.box.pose = obj_msg.pose;
	obj_msg.box.dimensions.x = 0.01;
	obj_msg.box.dimensions.y = 0.01;
	obj_msg.box.dimensions.z = 0.01;
    
	return obj_msg;
      }

    static ainstein_radar_msgs::BoundingBox boundingBoxToROSMsg( const ainstein_radar_drivers::BoundingBox &b )
      {
	ainstein_radar_msgs::BoundingBox box;
	box.pose = tf2::toMsg( b.pose );
	box.dimensions.x = b.dimensions.x();
	box.dimensions.y = b.dimensions.y();
	box.dimensions.z = b.dimensions.z();

	return box;
      }

    static void getTargetsBoundingBox(const ainstein_radar_msgs::RadarTargetArray& targets, ainstein_radar_msgs::BoundingBox& box)
    {
      // Find the bounding box dimensions:
      Eigen::Vector3d min_point =
	Eigen::Vector3d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
			std::numeric_limits<double>::infinity());
      Eigen::Vector3d max_point =
	Eigen::Vector3d(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
			-std::numeric_limits<double>::infinity());

      if (targets.targets.size() > 0)
	{
	  for (const auto& t : targets.targets)
	    {
	      Eigen::Vector3d target_point;
	      ainstein_radar_drivers::utilities::sphericalToCartesian(t.range, (M_PI / 180.0) * t.azimuth, (M_PI / 180.0) * t.elevation,
								      target_point);
	      min_point = min_point.cwiseMin(target_point);
	      max_point = max_point.cwiseMax(target_point);
	    }
	}

      // Check for the case in which the box is degenerative:
      if ((max_point - min_point).norm() < 1e-6)
	{
	  min_point -= 0.1 * Eigen::Vector3d::Ones();
	  max_point += 0.1 * Eigen::Vector3d::Ones();
	}

      // Compute box pose (identity orientation, geometric center is position):
      Eigen::Affine3d box_pose;
      box_pose.linear() = Eigen::Matrix3d::Identity();
      box_pose.translation() = min_point + (0.5 * (max_point - min_point));

      // Form the box message:
      box.pose = tf2::toMsg(box_pose);

      box.dimensions.x = max_point.x() - min_point.x();
      box.dimensions.y = max_point.y() - min_point.y();
      box.dimensions.z = 0.1;
    }

  } // namespace utilities
  
} // namespace ainstein_radar_drivers

#endif // #define RADAR_DRIVER_UTILITIES_H_
