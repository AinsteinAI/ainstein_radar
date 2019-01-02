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

#ifndef RADAR_DATA_VIZ_POINT_CLOUD_H_
#define RADAR_DATA_VIZ_POINT_CLOUD_H_

#include <ros/ros.h>
#include <pcl/point_cloud.h>

#include <radar_sensor_msgs/RadarData.h>

class RadarDataVizPointCloud
{
public:
    RadarDataVizPointCloud( std::string data_topic, std::string pcl_topic ) :
            data_topic_( data_topic ),
            pcl_topic_( pcl_topic )
    {
      pub_pcl_ = node_handle_.advertise<sensor_msgs::PointCloud2>( pcl_topic_, 10 );
      sub_radar_data_ = node_handle_.subscribe( data_topic_, 10,
						&RadarDataVizPointCloud::radarDataCallback,
						this );
    }

    ~RadarDataVizPointCloud()
    {
    }

    pcl::PointXYZ radarDataToPclPoint( const radar_sensor_msgs::RadarTarget &target )
    {
      pcl::PointXYZ p;
        p.x = cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
              * target.range;
        p.y = sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation )
              * target.range;
        p.z = sin( ( M_PI / 180.0 ) * target.elevation ) * target.range;

        return p;
    }

    void radarDataCallback( const radar_sensor_msgs::RadarData &msg )
    {
        // First delete, then populate the raw pcls:
        pcl_.clear();
	for( auto it = msg.raw_targets.begin(); it != msg.raw_targets.end(); ++it )
        {
	  pcl_.points.push_back( radarDataToPclPoint( *it ) );
        }

	pcl_.width = pcl_.points.size();
	pcl_.height = 1;
	
	pcl::toROSMsg( pcl_, cloud_msg_ );
        pub_pcl_.publish( cloud_msg_ );
    }

private:
    std::string data_topic_;
    std::string pcl_topic_;
    std::string frame_id_;

    pcl::PointCloud<pcl::PointXYZ> pcl_;
    sensor_msgs::PointCloud2 cloud_msg_;
    
    ros::NodeHandle node_handle_;
    ros::Subscriber sub_radar_data_;
    ros::Publisher pub_pcl_;

};

#endif // RADAR_DATA_VIZ_H_
