#ifndef TRACKING_FILTER_H_
#define TRACKING_FILTER_H_

#include <chrono>
#include <mutex>
#include <thread>

#include <ainstein_radar_filters/TrackingFilterConfig.h>
#include <ainstein_radar_filters/data_conversions.h>
#include <ainstein_radar_filters/radar_target_kf.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

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
  ~RadarTarget(void);

public:
  double range;
  double speed;
  double azimuth;
  double elevation;
};

class TrackingFilter
{
public:
  TrackingFilter(void);
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

  void setFilterParameters(const TrackingFilterParams& params)
  {
    filter_process_rate_ = params.filter_process_rate;
    filter_min_time_ = params.filter_min_time;
    filter_timeout_ = params.filter_timeout;
    filter_val_gate_thresh_ = params.filter_val_gate_thresh;

    RadarTargetKF::setFilterParameters(params.kf_params);
  }

  void initialize(void);
  void processFiltersLoop(double frequency);

  void radarTargetArrayCallback(const ainstein_radar_msgs::RadarTargetArray& msg);

  jsk_recognition_msgs::BoundingBox getBoundingBox(const ainstein_radar_msgs::RadarTarget& tracked_target,
                                                   const ainstein_radar_msgs::RadarTargetArray& targets);

  Eigen::Vector3d radarTargetToPoint(const ainstein_radar_msgs::RadarTarget& target)
  {
    Eigen::Vector3d point;
    point.x() = cos((M_PI / 180.0) * target.azimuth) * cos((M_PI / 180.0) * target.elevation) * target.range;
    point.y() = sin((M_PI / 180.0) * target.azimuth) * cos((M_PI / 180.0) * target.elevation) * target.range;
    point.z() = sin((M_PI / 180.0) * target.elevation) * target.range;

    return point;
  }

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

  std::vector<ainstein_radar_filters::RadarTarget> tracked_targets_;
  std::vector<std::vector<ainstein_radar_filters::RadarTarget>> tracked_clusters_;
};

}  // namespace ainstein_radar_filters

#endif  // RADAR_TRACKING_FILTER_H_
