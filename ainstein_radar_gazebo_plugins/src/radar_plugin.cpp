#include "ainstein_radar_gazebo_plugins/radar_plugin.h"
#include "gazebo_plugins/gazebo_ros_utils.h"

#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <tf/tf.h>

namespace ainstein_radar_gazebo_plugins
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN( RadarPlugin )

////////////////////////////////////////////////////////////////////////////////
// Constructor
RadarPlugin::RadarPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RadarPlugin::~RadarPlugin()
{
    this->radar_queue_.clear();
    this->radar_queue_.disable();
    this->rosnode_->shutdown();
    this->callback_queue_thread_.join();

    delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RadarPlugin::Load( sensors::SensorPtr _parent, sdf::ElementPtr _sdf )
{
    // load plugin
    RayPlugin::Load( _parent, this->sdf );
    // Get then name of the parent sensor
    this->parent_sensor_ = _parent;
    // Get the world name.
    std::string worldName = _parent->WorldName();
    this->world_ = physics::get_world( worldName );
    // save pointers
    this->sdf = _sdf;

    this->last_update_time_ = common::Time( 0 );

    GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
    this->parent_ray_sensor_ = dynamic_pointer_cast < sensors::RaySensor > ( this->parent_sensor_ );

    if( !this->parent_ray_sensor_ )
        gzthrow( "RadarPlugin controller requires a Ray Sensor as its parent" );

    this->robot_namespace_ = "";
    if( this->sdf->HasElement( "robotNamespace" ) )
        this->robot_namespace_ = this->sdf->Get < std::string > ( "robotNamespace" ) + "/";

    if( !this->sdf->HasElement( "frameName" ) )
    {
        ROS_INFO_NAMED( "radar", "Radar plugin missing <frameName>, defaults to /base_link" );
        this->frame_name_ = "/base_link";
    }
    else
        this->frame_name_ = this->sdf->Get < std::string > ( "frameName" );

    if( !this->sdf->HasElement( "topicName" ) )
    {
        ROS_INFO_NAMED( "radar", "Radar plugin missing <topicName>, defaults to /radar" );
        this->topic_name_ = "/radar";
    }
    else
        this->topic_name_ = this->sdf->Get < std::string > ( "topicName" );

    if( !this->sdf->HasElement( "updateRate" ) )
    {
        ROS_INFO_NAMED( "radar", "Radar plugin missing <updateRate>, defaults to 0" );
        this->update_rate_ = 0;
    }
    else
        this->update_rate_ = this->sdf->Get<double>( "updateRate" );

    if( !this->sdf->HasElement( "noiseStdevRange" ) )
    {
        ROS_INFO_NAMED( "radar", "Radar plugin missing <noiseStdevRange>, defaults to 0.0" );
        this->stdev_range_ = 0.0;
    }
    else
      this->stdev_range_ = this->sdf->Get<double>( "noiseStdevRange" );

    dist_range_ = std::normal_distribution<double>( 0.0, stdev_range_ );

    if( !this->sdf->HasElement( "noiseStdevAzimuth" ) )
    {
        ROS_INFO_NAMED( "radar", "Radar plugin missing <noiseStdevAzimuth>, defaults to 0.0" );
        this->stdev_azimuth_ = 0.0;
    }
    else
      this->stdev_azimuth_ = this->sdf->Get<double>( "noiseStdevAzimuth" );

    dist_azimuth_ = std::normal_distribution<double>( 0.0, stdev_azimuth_ );

    // prepare to throttle this plugin at the same rate
    // ideally, we should invoke a plugin update when the sensor updates,
    // have to think about how to do that properly later
    if( this->update_rate_ > 0.0 )
        this->update_period_ = 1.0 / this->update_rate_;
    else
        this->update_period_ = 0.0;

    this->radar_connect_count_ = 0;

    this->radar_msg_.header.frame_id = this->frame_name_;

    // Init ROS
    if( ros::isInitialized() )
    {
        // ros callback queue for processing subscription
        this->deferred_load_thread_ = boost::thread(
                boost::bind( &RadarPlugin::LoadThread, this ) );
    }
    else
    {
        gzerr << "Not loading plugin since ROS hasn't been "
              << "properly initialized.  Try starting gazebo with ros plugin:\n"
              << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RadarPlugin::LoadThread()
{
    this->rosnode_ = new ros::NodeHandle( this->robot_namespace_ );

    // resolve tf prefix
    std::string prefix;
    this->rosnode_->getParam( std::string( "tf_prefix" ), prefix );
    this->frame_name_ = tf::resolve( prefix, this->frame_name_ );

    if( this->topic_name_ != "" )
    {
        ros::AdvertiseOptions ao =
                ros::AdvertiseOptions::create<ainstein_radar_msgs::RadarTargetArray>(
                this->topic_name_, 1, boost::bind( &RadarPlugin::RadarConnect, this ),
                boost::bind( &RadarPlugin::RadarDisconnect, this ), ros::VoidPtr(),
                &this->radar_queue_ );
        this->pub_ = this->rosnode_->advertise( ao );
    }

    // Initialize the controller

    // sensor generation off by default
    this->parent_ray_sensor_->SetActive( false );
    // start custom queue for radar
    this->callback_queue_thread_ = boost::thread(
            boost::bind( &RadarPlugin::RadarQueueThread, this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void RadarPlugin::RadarConnect()
{
    this->radar_connect_count_++;
    this->parent_ray_sensor_->SetActive( true );
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void RadarPlugin::RadarDisconnect()
{
    this->radar_connect_count_--;

    if( this->radar_connect_count_ == 0 )
        this->parent_ray_sensor_->SetActive( false );
}

////////////////////////////////////////////////////////////////////////////////
// Update the plugin
void RadarPlugin::OnNewLaserScans()
{
    if( this->topic_name_ != "" )
    {
#if GAZEBO_MAJOR_VERSION >= 8
        common::Time cur_time = this->world_->SimTime();
#else
        common::Time cur_time = this->world_->GetSimTime();
#endif
        if( cur_time < this->last_update_time_ )
        {
            ROS_WARN_NAMED( "radar", "Negative sensor update time difference detected." );
            this->last_update_time_ = cur_time;
        }

        if( cur_time - this->last_update_time_ >= this->update_period_ )
        {
            common::Time sensor_update_time = this->parent_sensor_->LastUpdateTime();
            this->PutRadarData( sensor_update_time );
            this->last_update_time_ = cur_time;
        }
    }
    else
    {
        ROS_INFO_NAMED( "radar", "gazebo_ros_radar topic name not set" );
    }
}

////////////////////////////////////////////////////////////////////////////////
// Put radar data to the interface
void RadarPlugin::PutRadarData( common::Time &_updateTime )
{
    this->parent_ray_sensor_->SetActive( false );

    /***************************************************************/
    /*                                                             */
    /*  point scan from ray sensor                                 */
    /*                                                             */
    /***************************************************************/
    {
        boost::mutex::scoped_lock lock( this->lock_ );
        // Add Frame Name
        this->radar_msg_.header.frame_id = this->frame_name_;
        this->radar_msg_.header.stamp.sec = _updateTime.sec;
        this->radar_msg_.header.stamp.nsec = _updateTime.nsec;

        // Get the number of rays available from laser scan data:
//        int num_rays = parent_ray_sensor_->LaserShape()->GetSampleCount()
//                * parent_ray_sensor_->LaserShape()->GetVerticalSampleCount();
        int num_rays = parent_ray_sensor_->RayCount();

        // Add targets dynamically to RadarTargetArray:
        this->radar_msg_.targets.clear();
        ainstein_radar_msgs::RadarTarget target;
        int target_id = 0;
        for( int i = 0; i < num_rays; ++i )
        {
            // Get the range of the ray:
            target.range = parent_ray_sensor_->Range( i ) + dist_range_( dist_gen_ );

            // Rays are indexed from right to left, but targets to left of center are positive:
            target.azimuth = ( 180.0 / M_PI ) * ( i * parent_ray_sensor_->AngleResolution()
						  + parent_ray_sensor_->AngleMin().Radian() + dist_azimuth_( dist_gen_ ) );

            // Set the SNR based on reflected intensity (doesn't work as expected):
            target.snr = parent_ray_sensor_->Retro( i );

            // Ray sensor returns max range if nothing detected, need to filter:
            if( target.range < this->parent_ray_sensor_->RangeMax() )
            {
                target.target_id = target_id;
                this->radar_msg_.targets.push_back( target );
                ++target_id;
            }
        }

        this->parent_ray_sensor_->SetActive( true );

        // send data out via ros message
        if( this->radar_connect_count_ > 0 && this->topic_name_ != "" )
            this->pub_.publish( this->radar_msg_ );
    }
}

////////////////////////////////////////////////////////////////////////////////
// Put radar data to the interface
void RadarPlugin::RadarQueueThread()
{
    static const double timeout = 0.01;

    while( this->rosnode_->ok() )
    {
        this->radar_queue_.callAvailable( ros::WallDuration( timeout ) );
    }
}
} // namespace ainstein_radar_gazebo_plugins
