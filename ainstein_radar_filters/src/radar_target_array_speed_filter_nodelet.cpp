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

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ainstein_radar_filters/radar_target_array_speed_filter.h"

class NodeletRadarTargetArraySpeedFilter : public nodelet::Nodelet
{
public:
  NodeletRadarTargetArraySpeedFilter( void ) {}
  ~NodeletRadarTargetArraySpeedFilter( void ) {}
  
  virtual void onInit( void )
  {
    radar_speed_filter_ptr_.reset( new ainstein_radar_filters::RadarTargetArraySpeedFilter( getNodeHandle(), getPrivateNodeHandle() ) );
  }

private:
  std::unique_ptr<ainstein_radar_filters::RadarTargetArraySpeedFilter> radar_speed_filter_ptr_;
};

PLUGINLIB_EXPORT_CLASS( NodeletRadarTargetArraySpeedFilter, nodelet::Nodelet )
