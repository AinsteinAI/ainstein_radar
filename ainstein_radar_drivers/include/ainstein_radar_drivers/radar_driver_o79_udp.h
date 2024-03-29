#ifndef RADAR_DRIVER_O79_UDP_H_
#define RADAR_DRIVER_O79_UDP_H_

#include <netinet/in.h>
#include <string>
#include <memory>
#include <vector>

#include "radar_target.h"
#include "bounding_box.h"
#include "radar_target_cartesian.h"
#include "radar_device_alarms.h"

namespace ainstein_radar_drivers
{

  class RadarDriverO79UDP {

  public:
    RadarDriverO79UDP( std::string host_ip_address, int host_port,
		    std::string radar_ip_address, int radar_port );
    ~RadarDriverO79UDP( void );

    bool connect( void );
    bool receiveTargets( std::vector<ainstein_radar_drivers::RadarTarget> &targets,
                         std::vector<ainstein_radar_drivers::BoundingBox> &bounding_boxes,
                         std::vector<ainstein_radar_drivers::RadarTargetCartesian> &targets_tracked_cart,
                         std::vector<ainstein_radar_drivers::RadarTargetCartesian> &targets_ground_cart,
                         std::vector<ainstein_radar_drivers::RadarTarget> &targets_filtered,
                         std::vector<ainstein_radar_drivers::RadarDeviceAlarms> &alarms);

    static const std::string connect_cmd_str;
    static const unsigned int connect_res_len;

    static const std::string run_cmd_str;

    static const unsigned int max_msg_len;
    static const unsigned int msg_len_raw_targets;
    static const unsigned int msg_len_tracked_targets;
    static const unsigned int msg_len_bounding_boxes;
    static const unsigned int msg_len_tracked_targets_cart;
    static const unsigned int msg_len_alarms;

    static const unsigned int msg_header_len;
    static const unsigned int msg_type_byte;
    static const unsigned int msg_id_raw_targets;
    static const unsigned int msg_id_tracked_targets;
    static const unsigned int msg_id_bounding_boxes;
    static const unsigned int msg_id_tracked_targets_cart;
    static const unsigned int msg_id_ground_targets_cart;
    static const unsigned int msg_id_raw_targets_16bit_pwr;
    static const unsigned int msg_id_filtered_pcl;
    static const unsigned int msg_id_alarms;

    static const double msg_range_res;
    static const double msg_speed_res;
    static const double msg_cart_pos_res;
    static const double msg_cart_vel_res;
    static const double msg_bbox_pos_res;
    static const double msg_bbox_dim_res;

  private:
    std::string host_ip_addr_;
    int host_port_;
    std::string radar_ip_addr_;
    int radar_port_;

    int sockfd_; // socket file descriptor
    struct sockaddr_in sockaddr_;
    char* buffer_;

    struct sockaddr_in destaddr_;
  };

} // namespace ainstein_radar_drivers

#endif // RADAR_DRIVER_O79_H_
