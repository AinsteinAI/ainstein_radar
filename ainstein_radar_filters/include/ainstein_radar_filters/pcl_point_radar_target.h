#ifndef PCL_POINT_RADAR_TARGET_H
#define PCL_POINT_RADAR_TARGET_H

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

/* namespace ainstein_radar_filters */ // we cannot define a custom type within a namespace! see https://github.com/PointCloudLibrary/pcl/issues/1152
/* { */
  struct PointRadarTarget
  {
    PCL_ADD_POINT4D;
    float snr;          // Signal-to-Noise Ratio
    float range;        // Range (meters)
    float speed;        // Speed (meters per second)
    float azimuth;      // Azimuth angle (degrees)
    float elevation;    // Elevation angle (degrees)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // make sure our new allocators are aligned
  } EIGEN_ALIGN16;    // enforce SSE padding for correct memory alignment

  POINT_CLOUD_REGISTER_POINT_STRUCT( PointRadarTarget,        
				     ( float, x, x )
				     ( float, y, y )
				     ( float, z, z )
				     ( float, snr, snr )
				     ( float, range, range )
				     ( float, speed, speed )
				     ( float, azimuth, azimuth )
				     ( float, elevation, elevation )
				     )
/* } // namespace ainstein_radar_filters */

#endif // PCL_POINT_RADAR_TARGET_H
