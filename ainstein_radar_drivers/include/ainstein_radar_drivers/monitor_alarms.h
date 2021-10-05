#ifndef MONITOR_ALARMS_H_
#define MONITOR_ALARMS_H_

namespace ainstein_radar_drivers
{
  // Simple ROS-independent monitor alarms class
  class MonitorAlarms
  {
  public:

    enum class Temperature
    {
      Disabled,
	High,
	HighWarn,
	Normal,
	LowWarn,
	Low
	};

    enum class FrameTime
    {
      Disabled,
	Normal,
	FailedHigh
	};

    enum class BlockedRadar
    {
      Disabled,
	Normal,
	Failed
	};

    enum class MmwaveMonitor
    {
      Disabled,
	Normal,
	Failed
	};
    
  MonitorAlarms( Temperature t,
		 FrameTime ft,
		 BlockedRadar br,
		 MmwaveMonitor mw ) :
      temperature( t ),
      frame_time( ft ),
      blocked_radar( br ),
      mmwave_monitor( mw )
    {
    }
    MonitorAlarms( void )
    {
      temperature = Temperature::Normal;
      frame_time = FrameTime::Normal;
      blocked_radar = BlockedRadar::Normal;
      mmwave_monitor = MmwaveMonitor::Normal;
    }
    ~MonitorAlarms( void )
      {
      }

    Temperature temperature;
    FrameTime frame_time;
    BlockedRadar blocked_radar;
    MmwaveMonitor mmwave_monitor;
  };

} // namespace ainstein_radar_drivers

#endif // MONITOR_ALARMS_H_
