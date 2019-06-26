#ifndef RADAR_PLUGIN_H
#define RADAR_PLUGIN_H

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <sdf/Param.hh>

namespace ainstein_radar_gazebo_plugins
{

  using namespace gazebo;
  
class RadarPlugin: public RayPlugin
{

    /// \brief Constructor
public:
    RadarPlugin();

    /// \brief Destructor
    ~RadarPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    void Load( sensors::SensorPtr _parent, sdf::ElementPtr _sdf );

    /// \brief Update the controller
protected:
    virtual void OnNewLaserScans();

    /// \brief Put radar data to the ROS topic
private:
    void PutRadarData( common::Time &_updateTime );

    /// \brief Keep track of number of connctions
    int radar_connect_count_;
    void RadarConnect();
    void RadarDisconnect();

    // Pointer to the model
    physics::WorldPtr world_;

    /// \brief The parent sensor
    sensors::SensorPtr parent_sensor_;
    sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
    ros::NodeHandle* rosnode_;
    ros::Publisher pub_;

    /// \brief ros message
    ainstein_radar_msgs::RadarTargetArray radar_msg_;

    /// \brief topic name
    std::string topic_name_;

    /// \brief frame transform name, should match link name
    std::string frame_name_;

    /// \brief radiation type : ultrasound or infrared
    std::string radiation_;

    /// \brief sensor field of view
    double fov_;
    /// \brief noise distribution generator
    std::default_random_engine dist_gen_;
    /// \brief Gaussian distribution, range
    double stdev_range_;
    std::normal_distribution<double> dist_range_;
    /// \brief Gaussian distribution, azimuth
    double stdev_azimuth_;
    std::normal_distribution<double> dist_azimuth_;

    /// \brief mutex to lock access to fields that are used in message callbacks
    boost::mutex lock_;

    /// update rate of this sensor
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;

    /// \brief for setting ROS name space
    std::string robot_namespace_;

    ros::CallbackQueue radar_queue_;
    void RadarQueueThread();
    boost::thread callback_queue_thread_;

    // deferred load in case ros is blocking
    sdf::ElementPtr sdf;
    void LoadThread();
    boost::thread deferred_load_thread_;
    unsigned int seed;
};
}
#endif // RADAR_PLUGIN_H
