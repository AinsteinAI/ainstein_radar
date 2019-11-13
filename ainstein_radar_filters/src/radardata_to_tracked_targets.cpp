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
    sub_radar_data_raw_ = nh_.subscribe( "radar_in", 1,
					 &RadarDataToTrackedTargets::radarDataCallback,
					 this );

    pub_radar_data_tracked_ = nh_private_.advertise<ainstein_radar_msgs::RadarTargetArray>( "tracked", 1 );
    
    pub_bounding_boxes_ = nh_private_.advertise<jsk_recognition_msgs::BoundingBoxArray>( "boxes", 1 );
    
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
	msg_tracked_targets_.targets.clear();

	// Set timestamp for output messages:
	msg_tracked_targets_.header.stamp = ros::Time::now();
	msg_tracked_boxes_.header.stamp = ros::Time::now();

	// Clear the tracked "clusters" message and publisher:
	msg_tracked_clusters_.clear();
	msg_tracked_boxes_.boxes.clear();
	
	ainstein_radar_msgs::RadarTarget tracked_target;
	int target_id = 0;
	for( int i = 0; i < filters_.size(); ++i )
	  {
	    if( filters_.at( i ).getTimeSinceStart() >= filter_min_time_ )
	      {
		// Fill the tracked targets message:
		tracked_target = filters_.at( i ).getState().asMsg();
		tracked_target.target_id = target_id;
		msg_tracked_targets_.targets.push_back( tracked_target );

		// Fill the tracked target clusters message:
		msg_tracked_clusters_.push_back( filter_targets_.at( i ) );

		msg_tracked_boxes_.boxes.push_back( getBoundingBox( tracked_target, filter_targets_.at( i ) ) );
		
		++target_id;
	      }
	  }

	// Publish the tracked targets:
	pub_radar_data_tracked_.publish( msg_tracked_targets_ );

	// Publish the bounding boxes:
	pub_bounding_boxes_.publish( msg_tracked_boxes_ );

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
    msg_tracked_targets_.header.frame_id = msg->header.frame_id;
    msg_tracked_boxes_.header.frame_id = msg->header.frame_id;
    
    // Reset the measurement count vector for keeping track of which measurements get used:
    meas_count_vec_.resize( msg->targets.size() );
    std::fill( meas_count_vec_.begin(), meas_count_vec_.end(), 0 );

    // Resize the targets associated with each filter and set headers:
    filter_targets_.clear();
    filter_targets_.resize( filters_.size() );
    for( auto& targets : filter_targets_ )
      {
	targets.header.stamp = ros::Time::now();
	targets.header.frame_id = msg->header.frame_id;
      }
    
    // Pass the raw detections to the filters for updating:
    for( int i = 0; i < filters_.size(); ++i )
      {
	ROS_DEBUG_STREAM( filters_.at( i ) );
	for( int j = 0; j < msg->targets.size(); ++j )
	  {
	    // Only use this target if it hasn't already been used by a filter:
	    if( meas_count_vec_.at( j ) == 0 )
	      {
		// Check whether the target should be used as measurement by this filter:
		ainstein_radar_msgs::RadarTarget t = msg->targets.at( j );
		Eigen::Vector4d z = filters_.at( i ).computePredMeas( filters_.at( i ).getState() );
		Eigen::Vector4d y = Eigen::Vector4d( t.range, t.speed, t.azimuth, t.elevation );
	    
		// Compute the normalized measurement error (squared):
		double meas_err = ( y - z ).transpose() * filters_.at( i ).computeMeasCov( filters_.at( i ).getState() ).inverse() * ( y - z );

		ROS_DEBUG_STREAM( "Meas Cov Inv: " << filters_.at( i ).computeMeasCov( filters_.at( i ).getState() ).inverse() << std::endl );
		
		ROS_DEBUG_STREAM( "Target " << j << " meas_err: " << meas_err );
		ROS_DEBUG_STREAM( "Target " << j << ": " << std::endl << t );
		
		// Allow the measurement through the validation gate based on threshold:
		if( meas_err < filter_val_gate_thresh_ )
		  {
		    filters_.at( i ).update( t );
		    ++meas_count_vec_.at( j );

		    // Store the target associated with the filter:
		    filter_targets_.at( i ).targets.push_back( t );
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

  jsk_recognition_msgs::BoundingBox RadarDataToTrackedTargets::getBoundingBox( const ainstein_radar_msgs::RadarTarget& tracked_target, const ainstein_radar_msgs::RadarTargetArray& targets )
  {
    // Find the bounding box dimensions:
    Eigen::Vector3d min_point = Eigen::Vector3d( std::numeric_limits<double>::infinity(),
						 std::numeric_limits<double>::infinity(),
						     std::numeric_limits<double>::infinity() );
    Eigen::Vector3d max_point = Eigen::Vector3d( -std::numeric_limits<double>::infinity(),
						 -std::numeric_limits<double>::infinity(),
						 -std::numeric_limits<double>::infinity() );    
    if( targets.targets.size() == 0 )
      {
	min_point = max_point = radarTargetToPoint( tracked_target );
      }
    else
      {
	for( const auto& t : targets.targets )
	  {
	    min_point = min_point.cwiseMin( radarTargetToPoint( t ) );
	    max_point = max_point.cwiseMax( radarTargetToPoint( t ) );
	  }
      }
    
    // Check for the case in which the box is degenerative:
    if( ( max_point - min_point ).norm() < 1e-6 )
      {
	min_point -= 0.1 * Eigen::Vector3d::Ones();
	max_point += 0.1 * Eigen::Vector3d::Ones();
      }
    
    // Compute box pose (identity orientation, geometric center is position):
    Eigen::Affine3d box_pose;
    box_pose.linear() = Eigen::Matrix3d::Identity();
    box_pose.translation() = min_point + ( 0.5 * ( max_point - min_point ) );

    // Form the box message:
    jsk_recognition_msgs::BoundingBox box;

    box.header.stamp = targets.header.stamp;
    box.header.frame_id = targets.header.frame_id;
    
    box.pose = tf2::toMsg( box_pose );
    
    box.dimensions.x = max_point.x() - min_point.x();
    box.dimensions.y = max_point.y() - min_point.y();
    box.dimensions.z = 0.1;

    return box;
  }
    
} // namespace ainstein_radar_filters
