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

#include <ros/ros.h>
#include "ainstein_radar_filters/tracking_filter.h"

class TrackingFilterROS
{
public:
  TrackingFilterROS(const ros::NodeHandle& node_handle, const ros::NodeHandle& node_handle_private,
                    double publish_frequency)
    : nh_(node_handle), nh_private_(node_handle_private)
  {
    // Set up dynamic reconfigure:
    dynamic_reconfigure::Server<ainstein_radar_filters::TrackingFilterConfig>::CallbackType f;
    f = boost::bind(&TrackingFilterROS::dynConfigCallback, this, _1, _2);
    dyn_config_server_.setCallback(f);

    publish_freq_ = publish_frequency;
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

  void TrackingFilterROS::initialize(void)
  {
    // Set up raw radar data subscriber and tracked radar data publisher:
    sub_radar_data_raw_ = nh_.subscribe("radar_in", 1, &TrackingFilterROS::radarTargetArrayCallback, this);

    sub_point_cloud_raw_ = nh_.subscribe("cloud_in", 1, &TrackingFilterROS::pointCloudCallback, this);

    pub_radar_data_tracked_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>("tracked", 1);

    pub_bounding_boxes_ = nh_private_.advertise<jsk_recognition_msgs::BoundingBoxArray>("boxes", 1);

    tracking_filter_.initialize();

    // Launch the periodic publishing thread:
    publish_thread_ =
        std::unique_ptr<std::thread>(new std::thread(&TrackingFilterROS::publishLoop, this, publish_freq_));
  }

  void TrackingFilterROS::publishLoop(double pub_freq)
  {
    // Set the periodic task rate:
    ros::Rate publish_rate(pub_freq);

    // Enter the main publish loop:
    bool first_time = true;
    ros::Time time_now, time_prev;
    double dt;
    while (ros::ok() && !ros::isShuttingDown())
    {
      // Add tracked targets for filters which have been running for specified time:
      msg_tracked_targets_.targets.clear();

      // Set timestamp for output messages:
      msg_tracked_targets_.header.stamp = ros::Time::now();
      msg_tracked_boxes_.header.stamp = ros::Time::now();

      // Clear the tracked "clusters" message and publisher:
      msg_tracked_clusters_.clear();
      msg_tracked_boxes_.boxes.clear();

      ainstein_radar_msgs::RadarTarget tracked_target;
      int target_id = 0;
      for (int i = 0; i < filters_.size(); ++i)
      {
        if (filters_.at(i).getTimeSinceStart() >= filter_min_time_)
        {
          // Fill the tracked targets message:
          tracked_target = filters_.at(i).getState().asMsg();
          tracked_target.target_id = target_id;
          msg_tracked_targets_.targets.push_back(tracked_target);

          // Fill the tracked target clusters message:
          msg_tracked_clusters_.push_back(filter_targets_.at(i));

          msg_tracked_boxes_.boxes.push_back(getBoundingBox(tracked_target, filter_targets_.at(i)));

          ++target_id;
        }
      }

      // Publish the tracked targets:
      pub_radar_data_tracked_.publish(msg_tracked_targets_);

      // Publish the bounding boxes:
      pub_bounding_boxes_.publish(msg_tracked_boxes_);

      // Store the current time:
      time_prev = time_now;

      // Spin once to handle callbacks:
      ros::spinOnce();

      // Sleep to maintain desired freq:
      publish_rate.sleep();
    }
  }

  void TrackingFilterROS::radarTargetArrayCallback(const ainstein_radar_msgs::RadarTargetArray& msg)
  {
    // Store the frame_id for the messages:
    msg_tracked_targets_.header.frame_id = msg.header.frame_id;
    msg_tracked_boxes_.header.frame_id = msg.header.frame_id;

    std::vector<ainstein_radar_filters::RadarTarget> targets;
    for (const auto& t : msg.targets)
    {
      targets.push_back(t.range, t.speed, t.azimuth, t.elevation);
    }

    tracking_filter_.updateFilters(targets);
  }

  void TrackingFilterROS::pointCloudCallback(const sensor_msgs::PointCloud2& cloud)
  {
    ainstein_radar_msgs::RadarTargetArray msg;
    data_conversions::rosCloudToRadarTargetArray(cloud, msg);

    radarTargetArrayCallback(msg);
  }

  jsk_recognition_msgs::BoundingBox TrackingFilterROS::getBoundingBox(
      const ainstein_radar_msgs::RadarTarget& tracked_target, const ainstein_radar_msgs::RadarTargetArray& targets)
  {
    // Find the bounding box dimensions:
    Eigen::Vector3d min_point =
        Eigen::Vector3d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_point =
        Eigen::Vector3d(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity());
    if (targets.targets.size() == 0)
    {
      min_point = max_point = radarTargetToPoint(tracked_target);
    }
    else
    {
      for (const auto& t : targets.targets)
      {
        min_point = min_point.cwiseMin(radarTargetToPoint(t));
        max_point = max_point.cwiseMax(radarTargetToPoint(t));
      }
    }

    // Check for the case in which the box is degenerative:
    if ((max_point - min_point).norm() < 1e-6)
    {
      min_point -= 0.1 * Eigen::Vector3d::Ones();
      max_point += 0.1 * Eigen::Vector3d::Ones();
    }

    // Compute box pose (identity orientation, geometric center is position):
    Eigen::Affine3d box_pose;
    box_pose.linear() = Eigen::Matrix3d::Identity();
    box_pose.translation() = min_point + (0.5 * (max_point - min_point));

    // Form the box message:
    jsk_recognition_msgs::BoundingBox box;

    box.header.stamp = targets.header.stamp;
    box.header.frame_id = targets.header.frame_id;

    box.pose = tf2::toMsg(box_pose);

    box.dimensions.x = max_point.x() - min_point.x();
    box.dimensions.y = max_point.y() - min_point.y();
    box.dimensions.z = 0.1;

    return box;
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

  ainstein_radar_msgs::RadarTargetArray msg_tracked_targets_;
  std::vector<ainstein_radar_msgs::RadarTargetArray> msg_tracked_clusters_;
  jsk_recognition_msgs::BoundingBoxArray msg_tracked_boxes_;
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
  TrackingFilterROS tracking_filter;

  tracking_filter.initialize();

  ros::spin();

  return 0;
}
