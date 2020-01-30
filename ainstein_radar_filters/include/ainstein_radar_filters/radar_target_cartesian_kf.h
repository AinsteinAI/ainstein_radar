#ifndef RADAR_TARGET_CART_KF_H_
#define RADAR_TARGET_CART_KF_H_

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <ainstein_radar_filters/data_conversions.h>
#include <ainstein_radar_msgs/RadarTarget.h>

#define Q_VEL_STDEV 5.0

#define R_SPEED_STDEV 1.0
#define R_POS_STDEV 0.1

#define INIT_POS_STDEV 1.0
#define INIT_VEL_STDEV 2.0

namespace ainstein_radar_filters
{
  class RadarTargetCartesianKF
  {

    class FilterState
    {
    public:
      FilterState( const ainstein_radar_msgs::RadarTarget& target,
		   const Eigen::Matrix<double, 6, 6>& initial_covariance )
      {
	data_conversions::sphericalToCartesian( target.range,
						( M_PI / 180.0 ) * target.azimuth,
						( M_PI / 180.0 ) * target.elevation,
						pos );

	data_conversions::sphericalToCartesian( target.speed,
						( M_PI / 180.0 ) * target.azimuth,
						( M_PI / 180.0 ) * target.elevation,
						vel );
		    
	cov = initial_covariance;
      }
      FilterState( const FilterState& state )
      {
	pos = state.pos;
	vel = state.vel;

	cov = state.cov;
      }
      ~FilterState() {}

      friend std::ostream& operator<< ( std::ostream& out, const FilterState& state )
      {
	out << "Position: " << state.pos.transpose() << std::endl
	    << "Velocity: " << state.vel.transpose() << std::endl
	    << "Covariance: " << std::endl << state.cov << std::endl;
      }

      Eigen::Vector3d pos;
      Eigen::Vector3d vel;
	
      Eigen::Matrix<double, 6, 6> cov;

      Eigen::Matrix<double, 6, 1> asVec( void ) const
      {
	Eigen::Matrix<double, 6, 1> state_vec;
	state_vec.segment( 0, 3 ) = this->pos;
	state_vec.segment( 3, 3 ) = this->vel;
	    
	return state_vec;
      }
      void fromVec( const Eigen::Matrix<double, 6, 1>& state_vec )
      {
	this->pos = state_vec.segment( 0, 3 );
	this->vel = state_vec.segment( 3, 3 );
      }

      ainstein_radar_msgs::RadarTarget asMsg( void ) const
      {
	ainstein_radar_msgs::RadarTarget t;
	double range, azimuth, elevation;
	data_conversions::cartesianToSpherical( pos, range, azimuth, elevation );

	t.range = range;
	t.speed = ( pos / pos.norm() ).transpose() * vel;
	t.azimuth = ( 180.0 / M_PI ) * azimuth;
	t.elevation = ( 180.0 / M_PI ) * elevation;

	return t;
      }

      geometry_msgs::Pose asPose( void ) const
      {
	Eigen::Affine3d pose_eigen;
	pose_eigen.translation() = pos;

	// Compute the pose assuming the +x direction is the current
	// estimated Cartesian velocity direction
	Eigen::Matrix3d rot_mat;
	rot_mat.col( 0 ) = vel / vel.norm();
	rot_mat.col( 1 ) = Eigen::Vector3d::UnitZ().cross( rot_mat.col( 0 ) );
	rot_mat.col( 2 ) = rot_mat.col( 0 ).cross( rot_mat.col( 1 ) );
	pose_eigen.linear() = rot_mat;
	
	geometry_msgs::Pose pose_msg;
	pose_msg = tf2::toMsg( pose_eigen );
	
	return pose_msg;
      }
    };

  public:
    RadarTargetCartesianKF( const ainstein_radar_msgs::RadarTarget& target,
			    const ros::NodeHandle& node_handle,
			    const ros::NodeHandle& node_handle_private );
    ~RadarTargetCartesianKF() {}

    class FilterParameters
    {
    public:
      FilterParameters( void ) {}
      ~FilterParameters( void )	{}
      
      double init_pos_stdev;
      double init_vel_stdev;

      double q_vel_stdev;

      double r_speed_stdev;
      double r_pos_stdev;
    };

    friend std::ostream& operator<< ( std::ostream& out, const RadarTargetCartesianKF& kf )
    {
      out << "State: " << kf.state_post_;
      out << "Time Since Start: " << kf.getTimeSinceStart() << std::endl
	  << "Time Since Update: " << kf.getTimeSinceUpdate() << std::endl;
    }
	
    void process( double dt );
    void updateMeasJacobian( const FilterState& state );
    void update( const ainstein_radar_msgs::RadarTarget& target );

    FilterState getState( void ) const
    {
      return state_post_;
    }
    Eigen::Vector4d computePredMeas( const FilterState& state )
    {
      Eigen::Vector3d n_unit = state.pos / state.pos.norm();
      double speed = n_unit.transpose() * state.vel;

      return Eigen::Vector4d( speed,
			      state.pos.x(),
			      state.pos.y(),
			      state.pos.z() );
    }
    Eigen::Vector4d computeMeas( const ainstein_radar_msgs::RadarTarget& target )
    {
      Eigen::Vector3d pos;
      data_conversions::sphericalToCartesian( target.range,
					      ( M_PI / 180.0 ) * target.azimuth,
					      ( M_PI / 180.0 ) * target.elevation,
					      pos );

      return Eigen::Vector4d( target.speed,
			      pos.x(),
			      pos.y(),
			      pos.z() );
    }
    Eigen::Matrix4d computeMeasCov( const FilterState& state )
    {
      return H_ * state.cov * H_.transpose() + R_;
    }

    double getTimeSinceStart( void ) const
    {
      return ( ros::Time::now() - time_first_update_ ).toSec();
    }

    double getTimeSinceUpdate( void ) const 
    {
      return ( ros::Time::now() - time_last_update_ ).toSec();
    }

    static void setFilterParameters( const FilterParameters& params );
      
  private:
    FilterState state_pre_;
    FilterState state_post_;
      
    ros::Time time_first_update_;
    ros::Time time_last_update_;
      
    Eigen::Matrix<double, 6, 4> K_;
    
    // Shared among all filters:
    static Eigen::Matrix<double, 6, 6> F_;
    static Eigen::Matrix<double, 6, 3> L_;
    static Eigen::Matrix<double, 4, 6> H_;

    static Eigen::Matrix3d Q_;
    static Eigen::Matrix<double, 4, 4> R_;

    static Eigen::Matrix<double, 6, 6> P_init_;
  };

} // namespace ainstein_radar_filters

#endif // RADAR_TARGET_CART_KF_H_
