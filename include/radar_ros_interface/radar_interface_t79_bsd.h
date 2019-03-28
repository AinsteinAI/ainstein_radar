#ifndef RADAR_INTERFACE_T79_BSD_H_
#define RADAR_INTERFACE_T79_BSD_H_

#include <can_msgs/Frame.h>

#include "radar_ros_interface/radar_interface.h"
#include "radar_ros_interface/config_t79_bsd.h"

class RadarInterfaceT79BSD: public RadarInterface<can_msgs::Frame>
{
public:
  RadarInterfaceT79BSD( ros::NodeHandle node_Handle,
			ros::NodeHandle node_Handle_private );
  ~RadarInterfaceT79BSD()
    {
      stopRadar();
    }

    void startRadar( void );
    void stopRadar( void );

private:
    void dataMsgCallback( const can_msgs::Frame &msg );

    ConfigT79BSD::RadarType type_;
    std::string name_;
};


#endif
