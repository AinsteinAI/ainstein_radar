/*
  Copyright <2018-2019> <Ainstein, Inc.>

  Redistribution and use in source and binary forms, with or without modification, are permitted
  provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this list of
  conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this list of
  conditions and the following disclaimer in the documentation and/or other materials provided
  with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ainstein_radar_filters/data_conversions.h>
#include <ainstein_radar_filters/tracking_filter.h>
#include <ainstein_radar_filters/TrackingFilterConfig.h>
#include <ainstein_radar_filters/utilities.h>
#include <ainstein_radar_msgs/RadarTarget.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

class TrackingFilterROS
{
public:
  TrackingFilterROS(const ros::NodeHandle& node_handle, const ros::NodeHandle& node_handle_private,
                    double publish_frequency, double filter_speed_diff)
    : nh_(node_handle), nh_private_(node_handle_private)
  {
    // Set up dynamic reconfigure:
    dynamic_reconfigure::Server<ainstein_radar_filters::TrackingFilterConfig>::CallbackType f;
    f = boost::bind(&TrackingFilterROS::dynConfigCallback, this, _1, _2);
    dyn_config_server_.setCallback(f);

    publish_freq_ = publish_frequency;
    filter_speed_diff_ = filter_speed_diff;
  }

  ~TrackingFilterROS()
  {
  }

  void dynConfigCallback(const ainstein_radar_filters::TrackingFilterConfig& config, uint32_t level)
  {
    // Copy the new parameter values:
    ainstein_radar_filters::TrackingFilter::FilterParameters params;
    params.filter_process_rate = config.filter_update_rate;
    params.filter_min_time = config.filter_min_time;
    params.filter_timeout = config.filter_timeout;
    params.filter_val_gate_thresh = config.filter_val_gate_thresh;

    // Set the parameters for the underlying target Kalman Filters:
    params.kf_params.init_range_stdev = config.kf_init_range_stdev;
    params.kf_params.init_speed_stdev = config.kf_init_speed_stdev;
    params.kf_params.init_azim_stdev = config.kf_init_azim_stdev;
    params.kf_params.init_elev_stdev = config.kf_init_elev_stdev;

    params.kf_params.q_speed_stdev = config.kf_q_speed_stdev;
    params.kf_params.q_azim_stdev = config.kf_q_azim_stdev;
    params.kf_params.q_elev_stdev = config.kf_q_elev_stdev;

    params.kf_params.r_range_stdev = config.kf_r_range_stdev;
    params.kf_params.r_speed_stdev = config.kf_r_speed_stdev;
    params.kf_params.r_azim_stdev = config.kf_r_azim_stdev;
    params.kf_params.r_elev_stdev = config.kf_r_elev_stdev;

    tracking_filter_.setFilterParameters(params);
  }

  void initialize(void)
  {
    // Set up raw radar data subscriber and tracked radar data publisher:
    sub_radar_data_raw_ = nh_.subscribe("radar_in", 1, &TrackingFilterROS::radarTargetArrayCallback, this);

    sub_point_cloud_raw_ = nh_.subscribe("cloud_in", 1, &TrackingFilterROS::pointCloudCallback, this);

    pub_radar_data_tracked_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>("tracked", 1);

    pub_bounding_boxes_ = nh_private_.advertise<ainstein_radar_msgs::BoundingBoxArray>("boxes", 1);

    tracking_filter_.initialize();

    // Launch the periodic publishing thread:
    publish_thread_ =
        std::unique_ptr<std::thread>(new std::thread(&TrackingFilterROS::publishLoop, this, publish_freq_));
  }

  void publishLoop(double pub_freq)
  {
    // Set the periodic task rate:
    ros::Rate publish_rate(pub_freq);

    // Enter the main publish loop:
    bool first_time = true;
    ros::Time time_now, time_prev;
    double dt;
    while (ros::ok() && !ros::isShuttingDown())
    {
      // Get tracked object targets from filter:
      msg_tracked_targets_.targets.clear();
      msg_tracked_targets_.header.stamp = ros::Time::now();

      std::vector<ainstein_radar_filters::RadarTarget> tracked_objects;
      tracking_filter_.getTrackedObjects(tracked_objects);

      // Get targets associated with alive filters:
      msg_tracked_boxes_.boxes.clear();
      msg_tracked_boxes_.header.stamp = ros::Time::now();

      std::vector<std::vector<ainstein_radar_filters::RadarTarget>> tracked_object_targets;
      tracking_filter_.getTrackedObjectTargets(tracked_object_targets);

      // Fill the tracked object targets message, optionally filtering, then publish:
      for (int i = 0; i < tracked_objects.size(); ++i)
      {
        double mean_speed = std::accumulate(tracked_object_targets.at(i).begin(), tracked_object_targets.at(i).end(), 0.0,
        [] (double sum, const ainstein_radar_filters::RadarTarget &t) {return sum + t.speed;});
        mean_speed = mean_speed / tracked_object_targets.at(i).size();

        if (std::abs(tracked_objects.at(i).speed - mean_speed) <= filter_speed_diff_)
        {
          ainstein_radar_msgs::RadarTarget t;
          t.target_id = i;
          t.range = tracked_objects.at(i).range;
          t.speed = tracked_objects.at(i).speed;
          t.azimuth = tracked_objects.at(i).azimuth;
          t.elevation = tracked_objects.at(i).elevation;

          msg_tracked_targets_.targets.push_back(t);
        } 
      }

      pub_radar_data_tracked_.publish(msg_tracked_targets_);

      // Compute bounding boxes for each tracked object and publish:
      for (const auto& targets : tracked_object_targets)
      {
        ainstein_radar_msgs::RadarTargetArray msg_targets;
        msg_targets.header.frame_id = msg_tracked_boxes_.header.frame_id;  // need to pass this through
        for (const auto& t : targets)
        {
          ainstein_radar_msgs::RadarTarget target;
          target.range = t.range;
          target.speed = t.speed;
          target.azimuth = t.azimuth;
          target.elevation = t.elevation;
          msg_targets.targets.push_back(target);
        }

        ainstein_radar_msgs::BoundingBox box;
        ainstein_radar_filters::utilities::getTargetsBoundingBox(msg_targets, box);

        msg_tracked_boxes_.boxes.push_back(box);
      }

      pub_bounding_boxes_.publish(msg_tracked_boxes_);

      // Store the current time:
      time_prev = time_now;

      // Spin once to handle callbacks:
      ros::spinOnce();

      // Sleep to maintain desired freq:
      publish_rate.sleep();
    }
  }

  void radarTargetArrayCallback(const ainstein_radar_msgs::RadarTargetArray& msg)
  {
    // Store the frame_id for the messages:
    msg_tracked_targets_.header.frame_id = msg.header.frame_id;
    msg_tracked_boxes_.header.frame_id = msg.header.frame_id;

    std::vector<ainstein_radar_filters::RadarTarget> targets;
    for (const auto& t : msg.targets)
    {
      targets.emplace_back(t.range, t.speed, t.azimuth, t.elevation);
    }

    tracking_filter_.updateFilters(targets);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2& cloud)
  {
    ainstein_radar_msgs::RadarTargetArray msg;
    ainstein_radar_filters::data_conversions::rosCloudToRadarTargetArray(cloud, msg);

    radarTargetArrayCallback(msg);
  }

private:
  ros::NodeHandle nh_, nh_private_;

  ros::Subscriber sub_radar_data_raw_;
  ros::Subscriber sub_point_cloud_raw_;
  ros::Publisher pub_radar_data_tracked_;
  ros::Publisher pub_bounding_boxes_;
  dynamic_reconfigure::Server<ainstein_radar_filters::TrackingFilterConfig> dyn_config_server_;

  ainstein_radar_filters::TrackingFilter tracking_filter_;
  std::unique_ptr<std::thread> publish_thread_;
  double publish_freq_;
  double filter_speed_diff_;

  ainstein_radar_msgs::RadarTargetArray msg_tracked_targets_;
  std::vector<ainstein_radar_msgs::RadarTargetArray> msg_tracked_clusters_;
  ainstein_radar_msgs::BoundingBoxArray msg_tracked_boxes_;
};

int main(int argc, char** argv)
{
  // Initialize ROS node:
  ros::init(argc, argv, "tracking_filter_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  // Usage:
  if (argc < 1)
  {
    std::cerr << "Usage: rosrun ainstein_radar_filters tracking_filter_node" << std::endl;
    return -1;
  }

  // Create node to publish tracked targets:
  double publish_freq = node_handle_private.param("publish_freq", 20.0);
  double filter_speed_diff = node_handle_private.param("filter_speed_diff", 1.0);
  TrackingFilterROS tracking_filter_ros(node_handle, node_handle_private, publish_freq, filter_speed_diff);

  tracking_filter_ros.initialize();

  ros::spin();

  return 0;
}
