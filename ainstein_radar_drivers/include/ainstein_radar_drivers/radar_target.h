#ifndef RADAR_TARGET_H_
#define RADAR_TARGET_H_

namespace ainstein_radar_drivers
{
  // Simple ROS-independent radar target class
  class RadarTarget
  {
  public:

    RadarTarget( int id,
		 double range,
		 double speed,
		 double azimuth,
		 double elevation,
		 double snr ) :
      id( id ),
      range( range ),
      speed( speed),
      azimuth( azimuth ),
      elevation( elevation ),
      snr( snr)
    {
    }
    RadarTarget( void )
    {
      id = 0;
      range = 0.0;
      speed = 0.0;
      azimuth = 0.0;
      elevation = 0.0;
      snr = 0.0;
      }
    ~RadarTarget( void )
      {
      }

    int id;
    double range;
    double speed;
    double azimuth;
    double elevation;
    double snr;
  };

} // namespace ainstein_radar_drivers

#endif // RADAR_TARGET_H_
