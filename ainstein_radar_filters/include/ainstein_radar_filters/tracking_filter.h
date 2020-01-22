#ifndef TRACKING_FILTER_H_
#define TRACKING_FILTER_H_

#include <thread>
#include <mutex>

#include <ainstein_radar_filters/data_conversions.h>
#include <ainstein_radar_filters/radar_target_kf.h>
#include <ainstein_radar_filters/TrackingFilterConfig.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>

namespace ainstein_radar_filters
{
  class TrackingFilter
  {
  public:
    TrackingFilter( const ros::NodeHandle& node_handle,
			       const ros::NodeHandle& node_handle_private ) :
    nh_( node_handle ),
    nh_private_( node_handle_private )
    {
      // Set up dynamic reconfigure:
      dynamic_reconfigure::Server<ainstein_radar_filters::TrackingFilterConfig>::CallbackType f;
      f = boost::bind(&TrackingFilter::dynConfigCallback, this, _1, _2);
      dyn_config_server_.setCallback(f);
    }

    ~TrackingFilter() {}

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

    void pointCloudCallback( const sensor_msgs::PointCloud2 &cloud )
    {
      ainstein_radar_msgs::RadarTargetArray msg;
      data_conversions::rosCloudToRadarTargetArray( cloud, msg );
      
      radarTargetArrayCallback( msg );
    }

    void radarTargetArrayCallback( const ainstein_radar_msgs::RadarTargetArray &msg );

    jsk_recognition_msgs::BoundingBox getBoundingBox( const ainstein_radar_msgs::RadarTarget& tracked_target, const ainstein_radar_msgs::RadarTargetArray& targets );

    Eigen::Vector3d radarTargetToPoint( const ainstein_radar_msgs::RadarTarget& target )
    {
      Eigen::Vector3d point;
      point.x() = cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
	* target.range;
      point.y() = sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
	* target.range;
      point.z() = sin( ( M_PI / 180.0 ) * target.elevation ) * target.range;

      return point;
    }
    
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
    ros::Subscriber sub_point_cloud_raw_;
    ros::Publisher pub_radar_data_tracked_;
    ros::Publisher pub_bounding_boxes_;
    
    ainstein_radar_msgs::RadarTargetArray msg_tracked_targets_;
    std::vector<ainstein_radar_msgs::RadarTargetArray> msg_tracked_clusters_;
    jsk_recognition_msgs::BoundingBoxArray msg_tracked_boxes_;

    std::unique_ptr<std::thread> filter_update_thread_;
    std::mutex mutex_;
    
    std::vector<ainstein_radar_filters::RadarTargetKF> filters_;
    std::vector<ainstein_radar_msgs::RadarTargetArray> filter_targets_;
    std::vector<int> meas_count_vec_;
  };

} // namespace ainstein_radar_filters

#endif // RADAR_TRACKING_FILTER_H_
