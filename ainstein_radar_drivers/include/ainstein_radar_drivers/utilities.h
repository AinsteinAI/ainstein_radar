#ifndef RADAR_DRIVER_UTILITIES_H_
#define RADAR_DRIVER_UTILITIES_H_

#include "ainstein_radar_drivers/radar_target.h"
#include "ainstein_radar_drivers/bounding_box.h"
#include <ainstein_radar_msgs/RadarTarget.h>
#include <ainstein_radar_msgs/RadarTrackedObject.h>
#include <ainstein_radar_msgs/BoundingBox.h>
#include <ainstein_radar_filters/data_conversions.h>

namespace ainstein_radar_drivers
{
  namespace utilities
  {
    ainstein_radar_msgs::RadarTarget targetToROSMsg( const ainstein_radar_drivers::RadarTarget &t );
  
    ainstein_radar_msgs::RadarTrackedObject targetToObjectROSMsg( const ainstein_radar_drivers::RadarTarget &t );

    ainstein_radar_msgs::BoundingBox boundingBoxToROSMsg( const ainstein_radar_drivers::BoundingBox &b );
    
  } // namespace utilities
  
} // namespace ainstein_radar_drivers

#endif // #define RADAR_DRIVER_UTILITIES_H_
