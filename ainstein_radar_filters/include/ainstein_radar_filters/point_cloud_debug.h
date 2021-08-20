#ifndef POINT_CLOUD_DEBUG_H_
#define POINT_CLOUD_DEBUG_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ainstein_radar_filters/PointCloudDebugConfig.h>
#include <ainstein_radar_filters/common_types.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_filters
{
    class PointCloudDebug
    {
    public:
        PointCloudDebug( const ros::NodeHandle& node_handle, 
                         const ros::NodeHandle& node_handle_private ) :
        nh_( node_handle ),
        nh_private_( node_handle_private )
        {
            // Set up dynamic reconfigure:
            dynamic_reconfigure::Server<ainstein_radar_filters::PointCloudDebugConfig>::CallbackType f;
            f = boost::bind(&PointCloudDebug::dynConfigCallback, this, _1, _2);
            dyn_config_server_.setCallback(f);                
        }

        ~PointCloudDebug() {}

        void dynConfigCallback( const ainstein_radar_filters::PointCloudDebugConfig& config, uint32_t level )
        {
            // Copy the new parameter values:
            print_debug_ = config.print_debug;
            print_timestamp_ = config.print_timestamp;
            low_range_limit_m_ = config.low_range_limit_m;
            high_range_limit_m_ = config.high_range_limit_m;
            low_speed_limit_mps_ = config.low_speed_limit_mps;
            high_speed_limit_mps_ = config.high_speed_limit_mps;
            low_azimuth_limit_deg_ = config.low_azimuth_limit_deg;
            high_azimuth_limit_deg_ = config.high_azimuth_limit_deg;
            low_elevation_limit_deg_ = config.low_elevation_limit_deg;
            high_elevation_limit_deg_ = config.high_elevation_limit_deg;
            low_power_limit_ = config.low_power_limit;
            high_power_limit_ = config.high_power_limit;
        }

        void initialize( void );
        void radarTargetArrayCallback( const ainstein_radar_msgs::RadarTargetArray &msg );

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber sub_radar_data_raw_;

        // Parameters:
        dynamic_reconfigure::Server<ainstein_radar_filters::PointCloudDebugConfig> dyn_config_server_;
        bool print_debug_;
        bool print_timestamp_;
        double low_range_limit_m_; 
        double high_range_limit_m_;
        double low_speed_limit_mps_;
        double high_speed_limit_mps_;
        double low_azimuth_limit_deg_;
        double high_azimuth_limit_deg_ ;
        double low_elevation_limit_deg_ ;
        double high_elevation_limit_deg_;
        double low_power_limit_;
        double high_power_limit_;
    };
}


#endif