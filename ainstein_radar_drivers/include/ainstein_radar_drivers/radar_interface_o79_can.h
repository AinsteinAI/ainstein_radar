#ifndef RADAR_INTERFACE_O79_CAN_H_
#define RADAR_INTERFACE_O79_CAN_H_

#include <can_msgs/Frame.h>
#include <dynamic_reconfigure/server.h>

#include "ainstein_radar_drivers/radar_interface.h"
#include "ainstein_radar_drivers/ZoneOfInterestT79Config.h"
#include <ainstein_radar_msgs/RadarInfo.h>
#include "ainstein_radar_drivers/utilities.h"

namespace ainstein_radar_drivers
{
  class RadarInterfaceO79CAN: public RadarInterface<can_msgs::Frame>
  {
  public:
    RadarInterfaceO79CAN( ros::NodeHandle node_Handle,
			  ros::NodeHandle node_Handle_private );
    ~RadarInterfaceO79CAN()
    {
    }

    // Start and stop not required, currently autostarts
    void startRadar( void ) { return; };
    void stopRadar( void ) { return; };

    // Radar output message IDs
    static const uint16_t RADAR_START_FRAME = 0x420;
    static const uint16_t RADAR_STOP_FRAME = 0x480;
    static const uint16_t RADAR_RAW_TARGET = 0x4A0;
    static const uint16_t RADAR_TRACKED_TARGET = 0x490;

    // Misc. message IDs
    static const uint16_t RESERVED = 0xff;

    // Radar specifications (copied from K79, placeholders):
    static constexpr double UPDATE_RATE = 10.0;
    static constexpr int MAX_NUM_TARGETS = 1000;

    static constexpr double RANGE_MIN = 0.0;
    static constexpr double RANGE_MAX = 40.0;

    static constexpr double SPEED_MIN = 0.0;
    static constexpr double SPEED_MAX = 0.0;

    static constexpr double AZIMUTH_MIN = -40.0;
    static constexpr double AZIMUTH_MAX = 40.0;

    static constexpr double ELEVATION_MIN = -4.0;
    static constexpr double ELEVATION_MAX = 4.0;

    static constexpr double RANGE_RES = 0.0;
    static constexpr double RANGE_ACC = 0.1;

    static constexpr double SPEED_RES = 0.0;
    static constexpr double SPEED_ACC = 0.1;

    static constexpr double AZIMUTH_RES = 0.0;
    static constexpr double AZIMUTH_ACC = 1.0;

    static constexpr double ELEVATION_RES = 0.0;
    static constexpr double ELEVATION_ACC = 0.0;

    static const double msg_range_res;
    static const double msg_speed_res;
    static const double msg_pos_res;
    static const double msg_vel_res;

    static const double msg_pos_x_res;
    static const double msg_pos_y_res;
    static const double msg_pos_z_res;
    static const double msg_vel_x_res;
    static const double msg_vel_y_res;
    static const double msg_vel_z_res;

  private:
    void publishRadarInfo( void );

    void dataMsgCallback( const can_msgs::Frame &msg );

    can_msgs::Frame can_frame_msg_;

    unsigned int can_id_;
    std::string frame_id_;
    std::string can_id_str_;

    ros::Publisher pub_radar_info_;
    boost::shared_ptr<ainstein_radar_msgs::RadarInfo> radar_info_msg_ptr_;
};

} // namespace ainstein_drivers

#endif // RADAR_INTERFACE_O79_CAN_H_
