#ifndef RADAR_INTERFACE_T79_H_
#define RADAR_INTERFACE_T79_H_

#include <can_msgs/Frame.h>
#include <dynamic_reconfigure/server.h>

#include "ainstein_radar_drivers/radar_interface.h"
#include "ainstein_radar_drivers/ZoneOfInterestT79Config.h"

namespace ainstein_radar_drivers
{
  class RadarInterfaceT79: public RadarInterface<can_msgs::Frame>
  {
  public:
    RadarInterfaceT79( ros::NodeHandle node_Handle,
		       ros::NodeHandle node_Handle_private );
    ~RadarInterfaceT79()
    {
      // Stop the radar (doesn't seem to get called):
      stopRadar();
    }

    void startRadar( void );
    void stopRadar( void );
  
    void dynConfigCallback( const ainstein_radar_drivers::ZoneOfInterestT79Config& config, uint32_t level );
  
    void updateZOI( double range_min, double range_max,
		    double azimuth_min, double azimuth_max );
        
    // Radar command message IDs
    static const uint16_t RADAR_COMMAND = 0x100;
 
    static const uint16_t RADAR_START = 0x01;
    static const uint16_t RADAR_STOP = 0x02;
    static const uint16_t RADAR_SET_PARAMS = 0x03;
    static const uint16_t RADAR_SET_ZOI = 0x04;
    static const uint16_t RADAR_SEND_TRACKED = 0x01;
    static const uint16_t RADAR_SEND_RAW = 0x02;
    
    // Radar output message IDs
    static const uint16_t RADAR_COMMAND_RET = 0x101;
    static const uint16_t RADAR_START_FRAME = 0x420;
    static const uint16_t RADAR_STOP_FRAME = 0x480;
    static const uint16_t RADAR_RAW_TARGET = 0x4A0;
    static const uint16_t RADAR_TRACKED_TARGET = 0x490;

    // Misc. message IDs
    static const uint16_t RESERVED = 0xff; 

  private:
    void dataMsgCallback( const can_msgs::Frame &msg );
  
    can_msgs::Frame can_frame_msg_;
  
    int can_id_;
    std::string frame_id_;
  
    dynamic_reconfigure::Server<ainstein_radar_drivers::ZoneOfInterestT79Config> dyn_config_server_;
    ainstein_radar_drivers::ZoneOfInterestT79Config config_;
  };
  
} // namespace ainstein_drivers

#endif // RADAR_INTERFACE_T79_H_
