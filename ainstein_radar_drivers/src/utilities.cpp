#include "ainstein_radar_drivers/utilities.h"

namespace ainstein_radar_drivers
{
  namespace utilities
  {
    ainstein_radar_msgs::RadarTarget targetToROSMsg( const ainstein_radar_drivers::RadarTarget &t )
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
  
    ainstein_radar_msgs::RadarTrackedObject targetToObjectROSMsg( const ainstein_radar_drivers::RadarTarget &t )
      {
	ainstein_radar_msgs::RadarTrackedObject obj_msg;
    
	// Pass through the target ID:
	obj_msg.id = t.id;

	// Convert spherical coordinates to 3d pos and vel and then pose msg:
	Eigen::Vector3d pos, vel;
	ainstein_radar_filters::data_conversions::sphericalToCartesian( t.range, ( M_PI / 180.0 ) * t.azimuth, ( M_PI / 180.0 ) * t.elevation, pos );
	ainstein_radar_filters::data_conversions::sphericalToCartesian( t.speed, ( M_PI / 180.0 ) * t.azimuth, ( M_PI / 180.0 ) * t.elevation, vel );
	obj_msg.pose = ainstein_radar_filters::data_conversions::posVelToPose( pos, vel );

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

    ainstein_radar_msgs::BoundingBox boundingBoxToROSMsg( const ainstein_radar_drivers::BoundingBox &b )
      {
	ainstein_radar_msgs::BoundingBox box;
	box.pose = tf2::toMsg( b.pose );
	box.dimensions.x = b.dimensions.x();
	box.dimensions.y = b.dimensions.y();
	box.dimensions.z = b.dimensions.z();

	return box;
      }
    
  } // namespace utilities
  
} // namespace ainstein_radar_drivers
