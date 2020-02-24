#ifndef TRACKING_FILTER_H_
#define TRACKING_FILTER_H_

#include <Eigen/Eigen>
#include <chrono>
#include <mutex>
#include <thread>

#include <ainstein_radar_filters/TrackingFilterConfig.h>
#include <ainstein_radar_filters/radar_target_kf.h>

namespace ainstein_radar_filters
{
// simple, ROS-independent target class
class RadarTarget
{
public:
  RadarTarget(void)
  {
  }
  RadarTarget(double range, double speed, double azimuth, double elevation)
  {
    this->range = range;
    this->speed = speed;
    this->azimuth = azimuth;
    this->elevation = elevation;
  }
  ~RadarTarget(void)
  {
  }

public:
  double range;
  double speed;
  double azimuth;
  double elevation;
};

class TrackingFilter
{
public:
  TrackingFilter(void)
  {
    print_debug_ = false;
    is_running_ = true;
  }
  ~TrackingFilter()
  {
  }

  class FilterParameters
  {
  public:
    FilterParameters(void)
    {
    }
    ~FilterParameters(void)
    {
    }

    double filter_process_rate;
    double filter_min_time;
    double filter_timeout;
    double filter_val_gate_thresh;

    // Underlying Kalman Filter parameters (shared among all KFs):
    RadarTargetKF::FilterParameters kf_params;
  };

  void setFilterParameters(const FilterParameters& params)
  {
    filter_process_rate_ = params.filter_process_rate;
    filter_min_time_ = params.filter_min_time;
    filter_timeout_ = params.filter_timeout;
    filter_val_gate_thresh_ = params.filter_val_gate_thresh;

    RadarTargetKF::setFilterParameters(params.kf_params);
  }

  void initialize(void);
  void processFiltersLoop(double frequency);
  void updateFilters(const std::vector<RadarTarget>& targets);
  void stopRunning(void)
  {
    is_running_ = false;
  }
  void getTrackedObjects(std::vector<RadarTarget>& tracked_objects);
  void getTrackedObjectTargets(std::vector<std::vector<RadarTarget>>& tracked_objects);

  static const int max_tracked_targets;

private:
  // Parameters:
  double filter_process_rate_;
  double filter_min_time_;
  double filter_timeout_;
  double filter_val_gate_thresh_;

  bool print_debug_;

  bool is_running_;

  std::unique_ptr<std::thread> filter_process_thread_;
  std::mutex mutex_;

  std::vector<ainstein_radar_filters::RadarTargetKF> filters_;
  std::vector<std::vector<ainstein_radar_filters::RadarTarget>> filter_targets_;
  std::vector<int> meas_count_vec_;
  std::vector<bool> is_tracked_;
};

}  // namespace ainstein_radar_filters

#endif  // RADAR_TRACKING_FILTER_H_
