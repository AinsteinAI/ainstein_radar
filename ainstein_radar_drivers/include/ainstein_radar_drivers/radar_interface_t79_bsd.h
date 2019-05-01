#ifndef RADAR_INTERFACE_T79_BSD_H_
#define RADAR_INTERFACE_T79_BSD_H_

#include <can_msgs/Frame.h>

#include "ainstein_radar_drivers/radar_interface.h"
#include "ainstein_radar_drivers/config_t79_bsd.h"

namespace ainstein_radar_drivers
{

class RadarInterfaceT79BSD: public RadarInterface<can_msgs::Frame>
{
 public:
  RadarInterfaceT79BSD( ros::NodeHandle node_Handle,
			ros::NodeHandle node_Handle_private );
  ~RadarInterfaceT79BSD()
    {
      // Stop the radar (doesn't seem to get called):
      stopRadar();
    }

    void startRadar( void );
    void stopRadar( void );

private:
    void dataMsgCallback( const can_msgs::Frame &msg );

    ConfigT79BSD::RadarType type_;
    std::string frame_id_;
    std::string name_;
};

} // namespace ainstein_drivers


#endif
