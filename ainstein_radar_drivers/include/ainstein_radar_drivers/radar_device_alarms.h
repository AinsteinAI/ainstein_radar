#ifndef RADAR_DEVICE_ALARMS_H_
#define RADAR_DEVICE_ALARMS_H_

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

        enum diag_status_bits: uint16_t
        {
            none            = 0,
            blocked_radar   = 1<<0,
            frame_time      = 1<<1,
            main_proc_temp  = 1<<2,
            mmwave_temp     = 1<<3,
            mmwave_proc     = 1<<4,
            main_proc       = 1<<5
        };

        uint16_t FA_bits;
        uint16_t TFTKO_bits;
    };
}
#endif