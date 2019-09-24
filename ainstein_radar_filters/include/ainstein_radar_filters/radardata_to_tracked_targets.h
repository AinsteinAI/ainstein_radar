#ifndef RADAR_DATA_TO_TRACKED_TARGETS_H_
#define RADAR_DATA_TO_TRACKED_TARGETS_H_

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <thread>

#include <ainstein_radar_filters/radar_target_kf.h>
#include <ainstein_radar_filters/TrackingFilterConfig.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_filters
{
  class RadarDataToTrackedTargets
  {
  public:
    RadarDataToTrackedTargets( const ros::NodeHandle& node_handle,
			       const ros::NodeHandle& node_handle_private ) :
    nh_( node_handle ),
    nh_private_( node_handle_private )
    {
      // Set up dynamic reconfigure:
      dynamic_reconfigure::Server<ainstein_radar_filters::TrackingFilterConfig>::CallbackType f;
      f = boost::bind(&RadarDataToTrackedTargets::dynConfigCallback, this, _1, _2);
      dyn_config_server_.setCallback(f);
    }

    ~RadarDataToTrackedTargets() {}

    void dynConfigCallback( const ainstein_radar_filters::TrackingFilterConfig& config, uint32_t level )
    {
      // Copy the new parameter values:
      filter_update_rate_ = config.filter_update_rate;
      filter_min_time_ = config.filter_min_time;
      filter_timeout_ = config.filter_timeout;
      filter_val_gate_thresh_ = config.filter_val_gate_thresh;

      // Set the parameters for the underlying target Kalman Filters:
      RadarTargetKF::FilterParameters kf_params;
      kf_params.init_range_stdev = config.kf_init_range_stdev;
      kf_params.init_speed_stdev = config.kf_init_speed_stdev;
      kf_params.init_azim_stdev = config.kf_init_azim_stdev;
      kf_params.init_elev_stdev = config.kf_init_elev_stdev;

      kf_params.q_speed_stdev = config.kf_q_speed_stdev;
      kf_params.q_azim_stdev = config.kf_q_azim_stdev;
      kf_params.q_elev_stdev = config.kf_q_elev_stdev;
      
      kf_params.r_range_stdev = config.kf_r_range_stdev;
      kf_params.r_speed_stdev = config.kf_r_speed_stdev;
      kf_params.r_azim_stdev = config.kf_r_azim_stdev;
      kf_params.r_elev_stdev = config.kf_r_elev_stdev;

      RadarTargetKF::setFilterParameters( kf_params );
    }
      
    void initialize( void );
    void updateFiltersLoop( double frequency );
    
    void radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::Ptr &msg );

    static const int max_tracked_targets;
    
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Parameters:
    dynamic_reconfigure::Server<ainstein_radar_filters::TrackingFilterConfig> dyn_config_server_;
    double filter_update_rate_;
    double filter_min_time_;
    double filter_timeout_;
    double filter_val_gate_thresh_;
    
    ros::Subscriber sub_radar_data_raw_;
    ros::Publisher pub_radar_data_tracked_;
    ainstein_radar_msgs::RadarTargetArray msg_tracked_;
    
    std::unique_ptr<std::thread> filter_update_thread_;
    
    std::vector<ainstein_radar_filters::RadarTargetKF> filters_;
    std::vector<int> meas_count_vec_;
  };

} // namespace ainstein_radar_filters

#endif // RADAR_DATA_TO_TRACKED_TARGETS_H_
