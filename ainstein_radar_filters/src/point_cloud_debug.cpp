/*
  Copyright <2021> <Ainstein, Inc.>

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
#include <iomanip>
#include "ainstein_radar_filters/point_cloud_debug.h"

namespace ainstein_radar_filters
{
    void PointCloudDebug::initialize( void )
    {
        // Set up raw radar data subscriber and tracked radar data publisher:
        sub_radar_data_raw_ = nh_.subscribe("radar_in", 1,
                                            &PointCloudDebug::radarTargetArrayCallback,
                                            this );
        print_header_ = true;
        frame_count_ = 0;
    }

    void PointCloudDebug::radarTargetArrayCallback ( const ainstein_radar_msgs::RadarTargetArray& msg )
    {
        // configure cout to always print only 2 decimal places
        std::cout << std::fixed;
        std::cout << std::setprecision(2);

        if(print_debug_)
        {
            if(print_timestamp_)
            {
                std::cout << "Radar Point Cloud Frame " << msg.header.frame_id << " recevied at t = " << ros::Time::now() << std::endl;
            }

            if( print_header_ )
            {
                print_header_ = false;
                // Space the header to display nicely with the data in the terminal
                std::cout << "Power\tRange\tVel\tAzi\tEle\tFrameNo" << std::endl;
            }

            std::string delim = "\t";
            // Iterate through points - printing point data as we go
            for( int j = 0; j < msg.targets.size(); ++j )
            {
                ainstein_radar_msgs::RadarTarget t = msg.targets.at( j );
                if ( t.range >= low_range_limit_m_ && t.range <=  high_range_limit_m_ &&
                    t.speed >= low_speed_limit_mps_ && t.speed <= high_speed_limit_mps_ &&
                    t.azimuth >= low_azimuth_limit_deg_ && t.azimuth <= high_azimuth_limit_deg_ &&
                    t.elevation >= low_elevation_limit_deg_ && t.elevation <= high_elevation_limit_deg_ &&
                    t.snr >= low_power_limit_ && t.snr <= high_power_limit_ )
                {
                    std::cout << t.snr << delim << t.range << delim << t.speed << delim << t.azimuth << delim << t.elevation << delim << frame_count_ << std::endl;
                }
            }
        }

        frame_count_++;
    }
}