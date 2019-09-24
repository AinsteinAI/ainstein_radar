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

#include "ainstein_radar_filters/radardata_to_tracked_targets.h"

namespace ainstein_radar_filters
{
  const int RadarDataToTrackedTargets::max_tracked_targets = 100;

  void RadarDataToTrackedTargets::initialize( void )
  {
    // Set up raw radar data subscriber and tracked radar data publisher:
    sub_radar_data_raw_ = nh_.subscribe( "radardata_in", 10,
					 &RadarDataToTrackedTargets::radarDataCallback,
					 this );

    pub_radar_data_tracked_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "tracked", 10 );

    // Reserve space for the maximum number of target Kalman Filters:
    filters_.reserve( RadarDataToTrackedTargets::max_tracked_targets );

    // Launch the periodic filter update thread:
    filter_update_thread_ = std::unique_ptr<std::thread>( new std::thread( &RadarDataToTrackedTargets::updateFiltersLoop,
									   this,
									   filter_update_rate_ ) );									  
  }
  
  void RadarDataToTrackedTargets::updateFiltersLoop( double frequency )
  {
    // Set the periodic task rate:
    ros::Rate update_filters_rate( frequency );

    // Enter the main filters update loop:
    bool first_time = true;
    ros::Time time_now, time_prev;
    double dt;
    while( ros::ok() && !ros::isShuttingDown() )
      {
	// Compute the actual delta time:
	if( first_time )
	  {
	    time_prev = ros::Time::now();
	    
	    // Wait for simulated clock to start:
	    if( time_prev.toSec() > 0.0 )
	      {
		first_time = false;
	      }
	  }
	
	time_now = ros::Time::now();

	dt = ( time_now - time_prev ).toSec();

	// Remove filters which have not been updated in specified time:
	ROS_DEBUG_STREAM( "Number of filters before pruning: " << filters_.size() << std::endl );
	if( filters_.size() > 0 )
	  {
	    filters_.erase( std::remove_if( filters_.begin(),
	    				    filters_.end(),
	    				    [&]( const RadarTargetKF& kf ){ return ( kf.getTimeSinceUpdate() > filter_timeout_ ); } ),
			    filters_.end() );
	  }
	ROS_DEBUG_STREAM( "Number of filters after pruning: " << filters_.size() << std::endl );
	
	// Run process model for each filter:
	for( auto& kf : filters_ )
	  {    
	    kf.process( dt );
	  }

	// Add tracked targets for filters which have been running for specified time:
	msg_tracked_.targets.clear();
	msg_tracked_.header.stamp = ros::Time::now();
	ainstein_radar_msgs::RadarTarget target;
	int target_id = 0;
	for( const auto& kf : filters_ )
	  {
	    if( kf.getTimeSinceStart() >= filter_min_time_ )
	      {
		target = kf.getState().asMsg();
		target.target_id = target_id;
		msg_tracked_.targets.push_back( target );
		++target_id;
	      }
	  }

	// Publish the data:
	pub_radar_data_tracked_.publish( msg_tracked_ );
	
	// Store the current time and velocity:
	time_prev = time_now;

	// Spin once to handle callbacks:
	ros::spinOnce();
	
	// Sleep to maintain desired freq:
	update_filters_rate.sleep(); 
      }
  }
  
  void RadarDataToTrackedTargets::radarDataCallback( const ainstein_radar_msgs::RadarTargetArray::Ptr& msg )
  {
    // Store the frame_id for the messages:
    msg_tracked_.header.frame_id = msg->header.frame_id;
    
    // Reset the measurement count vector for keeping track of which measurements get used:
    meas_count_vec_.resize( msg->targets.size() );
    std::fill( meas_count_vec_.begin(), meas_count_vec_.end(), 0 );

    // Pass the raw detections to the filters for updating:
    for( auto& kf : filters_ )
      {
	ROS_DEBUG_STREAM( kf );
	for( int i = 0; i < msg->targets.size(); ++i )
	  {
	    // Only use this target if it hasn't already been used by a filter:
	    if( meas_count_vec_.at( i ) == 0 )
	      {
		// Check whether the target should be used as measurement by this filter:
		ainstein_radar_msgs::RadarTarget t = msg->targets.at( i );
		Eigen::Vector4d z = kf.computePredMeas( kf.getState() );
		Eigen::Vector4d y = Eigen::Vector4d( t.range, t.speed, t.azimuth, t.elevation );
	    
		// Compute the normalized measurement error (squared):
		double meas_err = ( y - z ).transpose() * kf.computeMeasCov( kf.getState() ).inverse() * ( y - z );

		ROS_DEBUG_STREAM( "Meas Cov Inv: " << kf.computeMeasCov( kf.getState() ).inverse() << std::endl );
		
		ROS_DEBUG_STREAM( "Target " << i << " meas_err: " << meas_err );
		ROS_DEBUG_STREAM( "Target " << i << ": " << std::endl << t );
		
		// Allow the measurement through the validation gate based on threshold:
		if( meas_err < filter_val_gate_thresh_ )
		  {
		    kf.update( t );
		    ++meas_count_vec_.at( i );
		  }
	      }
	  }
      }

    ROS_DEBUG_STREAM( "meas_count_vec_: " );
    for( const auto& ind : meas_count_vec_ )
      {
	ROS_DEBUG_STREAM( ind << " " );
      }
    ROS_DEBUG_STREAM( std::endl );

    // Iterate through targets and push back new KFs for unused measurements:
    for( int i = 0; i < meas_count_vec_.size(); ++i )
      {
    	if( meas_count_vec_.at( i ) == 0 )
    	  {
	    ROS_DEBUG_STREAM( "Pushing back: " << msg->targets.at( i ) << std::endl );
    	    filters_.emplace_back( msg->targets.at( i ), nh_, nh_private_ );
    	  }
      }
  }
  
} // namespace ainstein_radar_filters
