#ifndef RADAR_DRIVER_K79_H_
#define RADAR_DRIVER_K79_H_

#include <netinet/in.h>
#include <string>
#include <memory>
#include <vector>

#include "radar_target.h"

namespace ainstein_radar_drivers
{  
  class RadarDriverK79 {

  public:
    RadarDriverK79( std::string host_ip_address, int host_port,
		    std::string radar_ip_address, int radar_port );         
    ~RadarDriverK79( void );

    bool connect( void );
    bool receiveTargets( std::vector<ainstein_radar_drivers::RadarTarget> &targets,
			 std::vector<ainstein_radar_drivers::RadarTarget> &targets_tracked );
  
    static const std::string connect_cmd_str;
    static const unsigned int connect_res_len;

    static const std::string run_cmd_str;

    static const unsigned int radar_msg_len;
    static const unsigned int target_msg_len;
  
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

#endif // RADAR_DRIVER_K79_H_
