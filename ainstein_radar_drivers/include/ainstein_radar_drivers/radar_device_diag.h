#ifndef RADAR_DEVICE_DIAG_H_
#define RADAR_DEVICE_DIAG_H_
#include <string.h>
#include <cstdint>
#include <map>

namespace ainstein_radar_drivers
{
    class RadarDeviceDiag
    {
        public:
        RadarDeviceDiag(uint16_t const mask, std::string const desc) : bitmask(mask), description(desc) {}
        
        ~RadarDeviceDiag() {}
        const uint16_t bitmask;
        const std::string description;
    };

}
#endif // RADAR_DEVICE_DIAG_H_