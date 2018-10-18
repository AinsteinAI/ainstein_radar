/*
 Copyright <2018> <Ainstein, Inc.>

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

#ifndef RADAR_DATA_VIZ_H_
#define RADAR_DATA_VIZ_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <radar_ros_interface/RadarData.h>

class RadarDataViz
{
public:
    RadarDataViz( std::string data_topic, std::string marker_topic ) :
            data_topic_( data_topic ),
            marker_topic_( marker_topic )
    {
        // Initialize tracked marker info:
        marker_tracked_.ns = "tracked";
        marker_tracked_.type = visualization_msgs::Marker::POINTS;
        marker_tracked_.action = visualization_msgs::Marker::ADD;

        marker_tracked_.scale.x = 0.5;
        marker_tracked_.scale.y = 0.5;
        marker_tracked_.scale.z = 0.0;

        marker_tracked_.color.r = 1.0;
        marker_tracked_.color.g = 0.0;
        marker_tracked_.color.b = 0.0;
        marker_tracked_.color.a = 1.0;

        // Initialize raw marker info:
        marker_raw_.ns = "raw";
        marker_raw_.type = visualization_msgs::Marker::POINTS;
        marker_raw_.action = visualization_msgs::Marker::ADD;

        marker_raw_.scale.x = 0.5;
        marker_raw_.scale.y = 0.5;
        marker_raw_.scale.z = 0.0;

        marker_raw_.color.r = 0.0;
        marker_raw_.color.g = 1.0;
        marker_raw_.color.b = 0.0;
        marker_raw_.color.a = 1.0;

        pub_marker_ = node_handle_.advertise<visualization_msgs::Marker>( marker_topic_, 10 );
        sub_radar_data_ = node_handle_.subscribe( data_topic_, 10,
                                                  &RadarDataViz::radarDataCallback,
                                                  this );
    }

    ~RadarDataViz()
    {
    }

    geometry_msgs::Point radarDataToPoint( const radar_ros_interface::RadarTarget &target )
    {
        geometry_msgs::Point p;
        p.x = cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
              * target.range;
        p.y = sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
              * target.range;
        p.z = sin( ( M_PI / 180.0 ) * target.elevation ) * target.range;

        return p;
    }

    void radarDataCallback( const radar_ros_interface::RadarData &msg )
    {
        // First delete, then populate the tracked markers:
        marker_tracked_.action = visualization_msgs::Marker::DELETEALL;
        marker_tracked_.points.clear();
        marker_tracked_.header.frame_id = msg.header.frame_id;
        marker_tracked_.header.stamp = msg.header.stamp;
        pub_marker_.publish( marker_tracked_ );
	
        marker_tracked_.action = visualization_msgs::Marker::ADD;
        for( auto it = msg.tracked_targets.begin(); it != msg.tracked_targets.end(); ++it )
        {
            if( it->snr > RadarDataViz::min_snr )
            {
                marker_tracked_.id = it->target_id;
                marker_tracked_.color.a = std::min( ( it->snr / RadarDataViz::max_snr ), 1.0 );
                marker_tracked_.points.push_back( radarDataToPoint( *it ) );
            }
        }
        pub_marker_.publish( marker_tracked_ );

        // First delete, then populate the raw markers:
        marker_raw_.action = visualization_msgs::Marker::DELETEALL;
        marker_raw_.points.clear();
        marker_raw_.header.frame_id = msg.header.frame_id;
        marker_raw_.header.stamp = msg.header.stamp;
        pub_marker_.publish( marker_raw_ );

        marker_raw_.action = visualization_msgs::Marker::ADD;
        for( auto it = msg.raw_targets.begin(); it != msg.raw_targets.end(); ++it )
        {
            if( it->snr > RadarDataViz::min_snr )
            {
                marker_raw_.id = it->target_id;
                marker_raw_.color.a = std::min( ( it->snr / RadarDataViz::max_snr ), 1.0 );
                marker_raw_.points.push_back( radarDataToPoint( *it ) );
            }
        }
        pub_marker_.publish( marker_raw_ );
    }

    static const double min_snr;
    static const double max_snr;

private:
    std::string data_topic_;
    std::string marker_topic_;
    std::string frame_id_;

    visualization_msgs::Marker marker_tracked_;
    visualization_msgs::Marker marker_raw_;

    ros::NodeHandle node_handle_;
    ros::Subscriber sub_radar_data_;
    ros::Publisher pub_marker_;

};

const double RadarDataViz::min_snr = 15.0;
const double RadarDataViz::max_snr = 20.0;

#endif // RADAR_DATA_VIZ_H_
