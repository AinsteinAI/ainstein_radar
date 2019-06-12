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

#include "ainstein_radar_drivers/radar_target_kf.h"

namespace ainstein_radar_drivers
{
  Eigen::Matrix4d RadarTargetKF::F_ = ( Eigen::Matrix4d() <<
					0.0, 1.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0 ).finished();

  Eigen::Matrix<double, 4, 3> RadarTargetKF::L_ = ( Eigen::Matrix<double, 4, 3>() <<
					0.0, 0.0, 0.0,
					1.0, 0.0, 0.0,
					0.0, 1.0, 0.0,
					0.0, 0.0, 1.0 ).finished();
  
  Eigen::Matrix4d RadarTargetKF::H_ = Eigen::Matrix4d::Identity();
  
  Eigen::Matrix3d RadarTargetKF::Q_ = ( Eigen::Matrix3d() <<
					Q_SPEED, 0.0, 0.0,
					0.0, Q_AZIM, 0.0,
					0.0, 0.0, Q_ELEV ).finished();
  
  Eigen::Matrix4d RadarTargetKF::R_ = ( Eigen::Matrix4d() <<
					R_RANGE, 0.0, 0.0, 0.0,
					0.0, R_SPEED, 0.0, 0.0,
					0.0, 0.0, R_AZIM, 0.0,
					0.0, 0.0, 0.0, R_ELEV ).finished();

  Eigen::Matrix4d RadarTargetKF::P_init_ = ( Eigen::Vector4d() <<
					     std::pow( INIT_VAR_RANGE, 2.0 ),
					     std::pow( INIT_VAR_SPEED, 2.0 ),
					     std::pow( INIT_VAR_AZIM, 2.0 ),
					     std::pow( INIT_VAR_ELEV, 2.0 ) ).finished().asDiagonal();
  
  RadarTargetKF::RadarTargetKF( const ainstein_radar_msgs::RadarTarget& target,
				const ros::NodeHandle& node_handle,
				const ros::NodeHandle& node_handle_private) :
    state_pre_( target, P_init_ ),
    state_post_( target, P_init_ )
  {
    time_first_update_ = ros::Time::now();
    time_last_update_ = time_first_update_;
  }

  void RadarTargetKF::process( double dt )
  {
    // Compute discretized transition and noise matrices:
    Eigen::Matrix4d Fk = ( Eigen::Matrix4d::Identity() + dt * F_ );
    Eigen::Matrix4d Qk = dt * L_ * Q_ * L_.transpose();

    // ROS_DEBUG_STREAM( "State pre-process: " << state_post_ );

    // Update the state and covariance:
    state_pre_.fromVec( Fk * state_post_.asVec() );
    state_pre_.cov = Fk * state_post_.cov * Fk.transpose() + Qk;
    
    // ROS_DEBUG_STREAM( "State post-process: " << state_pre_ );

    // Set the post-measurement state to pre-measurement in case of no new measurements:
    state_post_ = state_pre_;
  }

  void RadarTargetKF::update( const ainstein_radar_msgs::RadarTarget& target )
  {
    // Convert the target to a measurement:
    Eigen::Vector4d meas_vec = Eigen::Vector4d( target.range,
						target.speed,
						target.azimuth,
						target.elevation );

    // Compute the Kalman gain:
    Eigen::Matrix4d meas_cov = ( H_ * state_pre_.cov * H_.transpose() + R_ );
    K_ = state_pre_.cov * H_.transpose() * meas_cov.inverse(); 

    ROS_DEBUG_STREAM( "Kalman gain: " << K_ );

    ROS_DEBUG_STREAM( "State pre-update: " << state_pre_ );
    
    // Update the state from the measurement and Kalman gain:
    state_post_.fromVec( state_pre_.asVec() + K_ *
			 ( meas_vec - computePredMeas( state_post_ ) ) );

    ROS_DEBUG_STREAM( "State post-update: " << state_post_ );
    
    // Update the state covariance:
    state_post_.cov = ( Eigen::Matrix4d::Identity() - K_ * H_ ) * state_pre_.cov.transpose();

    // Copy state post to state pre for next update:
    state_pre_ = state_post_;

    // Set the time of the update for book keeping filters:
    time_last_update_ = ros::Time::now();
  }
  
} // namespace ainstein_radar_drivers
