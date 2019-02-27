#ifndef RADAR_INTERFACE_T79_BSD_H_
#define RADAR_INTERFACE_T79_BSD_H_

#include <can_msgs/Frame.h>

#include "radar_ros_interface/radar_interface.h"
#include "radar_ros_interface/config_t79_bsd.h"

class RadarInterfaceT79BSD: public RadarInterface<can_msgs::Frame>
{
public:
    RadarInterfaceT79BSD( ConfigT79BSD::RadarType radar_type, std::string radar_name,
                     std::string frame_id ) :
            type_( radar_type ),
            frame_id_( frame_id ),
            RadarInterface<can_msgs::Frame>( radar_name, "received_messages",
                                                             "sent_messages" )
    {
        name_ = ConfigT79BSD::radar_names.at( type_ );
        startRadar();
    }
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
    std::string frame_id_;
};


#endif
