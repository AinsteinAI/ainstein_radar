#ifndef RADAR_DEVICE_ALARMS_H_
#define RADAR_DEVICE_ALARMS_H_
#include <stdlib.h>
#include <vector>
#include "radar_device_diag.h"
namespace ainstein_radar_drivers
{  
    class RadarDeviceAlarms
    {
    public:
        RadarDeviceAlarms(void)
        {
            FA_bits = 0;
            TFTKO_bits = 0;
        }
        ~RadarDeviceAlarms(void) {}

        std::string getStatusStr(uint16_t flags);

        uint16_t FA_bits;
        uint16_t TFTKO_bits;

    private:
        /* The bitmask value here must match the corresponding definitions on the radar*/
        const std::vector<RadarDeviceDiag> diags_{RadarDeviceDiag((uint16_t)0, "None"),
                                                  RadarDeviceDiag((uint16_t)(1<<0), "Blocked Radar"),
                                                  RadarDeviceDiag((uint16_t)(1<<1), "Frame Time"),
                                                  RadarDeviceDiag((uint16_t)(1<<2), "Main Processor Temperature"),
                                                  RadarDeviceDiag((uint16_t)(1<<3), "RF Processor Temperature"),
                                                  RadarDeviceDiag((uint16_t)(1<<4), "RF Processor Performance"),
                                                  RadarDeviceDiag((uint16_t)(1<<5), "Main Processor Performance")};
    };
}
#endif