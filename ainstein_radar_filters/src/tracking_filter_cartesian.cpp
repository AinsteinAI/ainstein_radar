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

#include "ainstein_radar_filters/tracking_filter_cartesian.h"

#define MAX_TARGET_ID 254

namespace ainstein_radar_filters
{
    const int TrackingFilterCartesian::max_tracked_targets = 100;

    void TrackingFilterCartesian::initialize( void )
    {
        // Set up raw radar data subscriber and tracked radar data publisher:
        sub_radar_data_raw_ = nh_.subscribe( "radar_in", 1,
                        &TrackingFilterCartesian::radarTargetArrayCallback,
                        this );

        sub_point_cloud_raw_ = nh_.subscribe( "cloud_in", 1,
                        &TrackingFilterCartesian::pointCloudCallback,
                        this );
        
        // Advertise the O79 tracked targets data:
        pub_radar_data_tracked_ = nh_private_.advertise<ainstein_radar_msgs::RadarTrackedObjectArray>( "objects", 1 );
        pub_bounding_boxes_ = nh_private_.advertise<ainstein_radar_msgs::BoundingBoxArray>( "boxes", 1 );
        
        // Reserve space for the maximum number of target Kalman Filters:
        filters_.reserve( TrackingFilterCartesian::max_tracked_targets );

        // Initialize the counter that keeps track of the max number of filters running at a time
        max_number_of_filters_ = 0;

        tid_counter_ = 0;

        // Launch the periodic filter update thread:
        filter_update_thread_ = std::unique_ptr<std::thread>( new std::thread( &TrackingFilterCartesian::updateFiltersLoop,
                                        this,
                                        filter_update_rate_ ) );  
    }
  
    void TrackingFilterCartesian::updateFiltersLoop( double frequency )
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
            
            // Block callback from modifying the filters
            mutex_.lock();
            
            // Run process model for each filter:
            for( auto& kf : filters_ )
            {    
                kf.process( dt );
            }

            // Add tracked targets for filters which have been running for specified time:
            msg_tracked_objects_.objects.clear();

            // Set timestamp for output messages:
            msg_tracked_objects_.header.stamp = ros::Time::now();
            msg_tracked_boxes_.header.stamp = ros::Time::now();

            // Clear the tracked "clusters" message and publisher:
            msg_tracked_boxes_.boxes.clear();
            
            ainstein_radar_msgs::RadarTrackedObject obj;
            for( int i = 0; i < filters_.size(); ++i )
            {
                if( filters_.at(i).get_status() == t_e_tracked 
                    || filters_.at(i).get_status() == t_e_extended )
                {
                    // Fill the tracked targets message:
                    obj = filters_.at(i).getState().asObjMsg( filters_.at(i).get_tid() );
                    msg_tracked_objects_.objects.push_back( obj );

                    // msg_tracked_boxes_.boxes.push_back( getBoundingBox( obj, filter_targets_.at( i ) ) );
                }
            }

            // Release lock on filter state
            mutex_.unlock();
            
            // Publish the tracked targets:
            pub_radar_data_tracked_.publish( msg_tracked_objects_ );

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

    void TrackingFilterCartesian::radarTargetArrayCallback( const ainstein_radar_msgs::RadarTargetArray& msg )
    {
        // Store the frame_id for the messages:
        msg_tracked_objects_.header.frame_id = msg.header.frame_id;
        msg_tracked_boxes_.header.frame_id = msg.header.frame_id;
        
        // Reset the measurement count vector for keeping track of which measurements get used:
        meas_count_vec_.resize( msg.targets.size() );
        std::fill( meas_count_vec_.begin(), meas_count_vec_.end(), 0 );

        // Block update loop from modifying the filters
        mutex_.lock();
        
        // Resize the targets associated with each filter and set headers:
        filter_targets_.clear();
        filter_targets_.resize( filters_.size() );
        for( auto& targets : filter_targets_ )
        {
            targets.header.stamp = ros::Time::now();
            targets.header.frame_id = msg.header.frame_id;
        }

        // Prepare for new KFs for unused measurements:
        ainstein_radar_msgs::RadarTargetArray arr;
        arr.header.stamp = ros::Time::now();
        arr.header.frame_id = msg.header.frame_id;

        for( int i = 0; i < filters_.size(); ++i )
        {
            // Form the state-dependent measurement Jacobian:
            filters_.at( i ).updateMeasJacobian( filters_.at( i ).getState() );
        }

        // Iterate through points as the outer loop - pushing back new KF's as we go
        for( int j = 0; j < msg.targets.size(); ++j )
        {
            ainstein_radar_msgs::RadarTarget t = msg.targets.at( j );
            if (t.range < min_range_)
            {
                ++meas_count_vec_.at( j ); /* mark this point as used; probably unnecessary */
            }
            else
            {
                for( int i = 0; i < filters_.size(); ++i )
                {
                    // Check whether the target should be used as measurement by this filter:
                    Eigen::Vector4d z = filters_.at( i ).computePredMeas( filters_.at( i ).getState() );
                    float vel_norm = filters_.at( i ).getState().vel.norm();
                    Eigen::Vector4d y = filters_.at( i ).computeMeas( t );

                    // std::cout << "Pred meas: " << z << std::endl;


                    // Compute the normalized measurement error (squared):
                    double meas_err = ( y - z ).transpose() * filters_.at( i ).computeMeasCov( filters_.at( i ).getState() ).inverse() * ( y - z );

                    Eigen::Vector3d target_pos;
                    data_conversions::sphericalToCartesian( t.range,
                                        ( M_PI / 180.0 ) * t.azimuth,
                                        ( M_PI / 180.0 ) * t.elevation,
                                        target_pos );
                    
                    // Allow the measurement through the validation gate based on threshold:
                    if( meas_err < filter_val_gate_thresh_ )
                    {
                        filters_.at( i ).update( t );
                        ++meas_count_vec_.at( j );

                        // Store the target associated with the filter:
                        filter_targets_.at( i ).targets.push_back( t );
                        break; // don't iterate through the rest of the filters if this point has already been associated
                    }
                }

                if( meas_count_vec_.at( j ) == 0 )
                {
                    // this point wasn't associated with any existing filter - instantiate a new filter based on it
                    filters_.emplace_back( msg.targets.at( j ), nh_, nh_private_ );
                    // Make sure to push back an empty array of targets associated with the new filter
                    filter_targets_.push_back( arr );
                }
            }
        }
        
        // Pass the raw detections to the filters for updating:
        for( int i = 0; i < filters_.size(); ++i )
        {
            // Update the filter's status (state machine)
            switch (filters_.at(i).get_status())
            {
            case t_e_pre_tracked:
                filters_.at(i).inc_pretrack_frame_cnt();
                if( filter_targets_.at(i).targets.size() > 0 )
                {
                    filters_.at(i).inc_pretrack_targ_cnt();
                    if( filters_.at(i).get_pretrack_targ_cnt() >= tracked_min_cnt_ )
                    {
                        tid_counter_++;
                        filters_.at(i).set_tid(tid_counter_);
                        
                        if(tid_counter_ > MAX_TARGET_ID)
                        {
                            tid_counter_ = 0;
                        }

                        filters_.at(i).set_status(t_e_tracked);
                    }
                }

                if ( filters_.at(i).get_pretrack_frame_cnt() >= pre_tracked_max_cnt_ )
                {
                    // This filter hasn't had enough frames with detections, mark it for deletion
                    filters_.at(i).set_status(t_e_delete_track);
                }
                break;

            case t_e_tracked:
                if( filter_targets_.at(i).targets.size() == 0 )
                {
                    filters_.at(i).set_status(t_e_extended);
                }
                break;
            
            case t_e_extended:
                if( filter_targets_.at(i).targets.size() > 0 )
                {
                    filters_.at(i).set_status(t_e_tracked);
                    filters_.at(i).reset_ext_frame_cnt();
                }
                else
                {
                    filters_.at(i).inc_ext_frame_cnt();
                    if( filters_.at(i).get_ext_frame_cnt() >= extended_max_cnt_ )
                    {
                        filters_.at(i).set_status(t_e_delete_track);
                    }
                }
                break;
            
            default:
                // should never get here
                break;
            }
        }

        // Remove filters which have not been updated in specified time:
        //ROS_DEBUG_STREAM( "Number of filters before pruning: " << filters_.size() << std::endl );
        if( filters_.size() > 0 )
        {
            filters_.erase( std::remove_if( filters_.begin(),
                                filters_.end(),
                                [&]( const RadarTargetCartesianKF& kf )
                                    { return ( kf.get_status() == t_e_delete_track ); } ),
                            filters_.end() );
        }
        //ROS_DEBUG_STREAM( "Number of filters after pruning: " << filters_.size() << std::endl );

        ROS_DEBUG_STREAM( "meas_count_vec_: " );
        for( const auto& ind : meas_count_vec_ )
        {
            ROS_DEBUG_STREAM( ind << " " );
        }
        ROS_DEBUG_STREAM( std::endl );

        if( filters_.size() > max_number_of_filters_ )
        {
            max_number_of_filters_ = filters_.size();
            ROS_DEBUG_STREAM("Max number of filters: " << max_number_of_filters_ << std::endl );
        }
        // Release lock on filter state
        mutex_.unlock();
    }

    ainstein_radar_msgs::BoundingBox TrackingFilterCartesian::getBoundingBox( const ainstein_radar_msgs::RadarTarget& tracked_target, const ainstein_radar_msgs::RadarTargetArray& targets )
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
        ainstein_radar_msgs::BoundingBox box;

        box.pose = tf2::toMsg( box_pose );
        
        box.dimensions.x = max_point.x() - min_point.x();
        box.dimensions.y = max_point.y() - min_point.y();
        box.dimensions.z = 0.1;

        return box;
    } 
} // namespace ainstein_radar_filters
