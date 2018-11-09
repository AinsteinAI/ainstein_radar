#ifndef GAZEBO_ROS_RADAR_H
#define GAZEBO_ROS_RADAR_H

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <radar_sensor_msgs/RadarData.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <sdf/Param.hh>

namespace gazebo
{

class GazeboRosRadar: public RayPlugin
{

    /// \brief Constructor
public:
    GazeboRosRadar();

    /// \brief Destructor
public:
    ~GazeboRosRadar();

    /// \brief Load the plugin
    /// \param take in SDF root element
public:
    void Load( sensors::SensorPtr _parent, sdf::ElementPtr _sdf );

    /// \brief Update the controller
protected:
    virtual void OnNewLaserScans();

    /// \brief Put radar data to the ROS topic
private:
    void PutRadarData( common::Time &_updateTime );

    /// \brief Keep track of number of connctions
private:
    int radar_connect_count_;
private:
    void RadarConnect();
private:
    void RadarDisconnect();

    // Pointer to the model
private:
    physics::WorldPtr world_;
    /// \brief The parent sensor
private:
    sensors::SensorPtr parent_sensor_;
private:
    sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
private:
    ros::NodeHandle* rosnode_;
private:
    ros::Publisher pub_;

    /// \brief ros message
private:
    radar_sensor_msgs::RadarData radar_msg_;

    /// \brief topic name
private:
    std::string topic_name_;

    /// \brief frame transform name, should match link name
private:
    std::string frame_name_;

    /// \brief radiation type : ultrasound or infrared
private:
    std::string radiation_;

    /// \brief sensor field of view
private:
    double fov_;
    /// \brief Gaussian noise
private:
    double gaussian_noise_;

    /// \brief Gaussian noise generator
private:
    double GaussianKernel( double mu, double sigma );

    /// \brief mutex to lock access to fields that are used in message callbacks
private:
    boost::mutex lock_;

    /// \brief hack to mimic hokuyo intensity cutoff of 100
private:
    double hokuyo_min_intensity_;

    /// update rate of this sensor
private:
    double update_rate_;
private:
    double update_period_;
private:
    common::Time last_update_time_;

    /// \brief for setting ROS name space
private:
    std::string robot_namespace_;

private:
    ros::CallbackQueue radar_queue_;
private:
    void RadarQueueThread();
private:
    boost::thread callback_queue_thread_;

    // deferred load in case ros is blocking
private:
    sdf::ElementPtr sdf;
private:
    void LoadThread();
private:
    boost::thread deferred_load_thread_;
private:
    unsigned int seed;
};
}
#endif // GAZEBO_ROS_RADAR_H
