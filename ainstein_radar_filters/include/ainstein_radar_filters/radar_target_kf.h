#ifndef RADAR_TARGET_KF_H_
#define RADAR_TARGET_KF_H_

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <ainstein_radar_msgs/RadarTarget.h>

#define Q_SPEED_STDEV 5.0
#define Q_AZIM_STDEV 10.0
#define Q_ELEV_STDEV 10.0

#define R_RANGE_STDEV 1.0
#define R_SPEED_STDEV 5.0
#define R_AZIM_STDEV 20.0
#define R_ELEV_STDEV 20.0

#define INIT_RANGE_STDEV 1.0
#define INIT_SPEED_STDEV 2.0
#define INIT_AZIM_STDEV 10.0
#define INIT_ELEV_STDEV 10.0

namespace ainstein_radar_filters
{
  class RadarTargetKF
  {      
    class FilterState
      {
      public:
	FilterState( const ainstein_radar_msgs::RadarTarget& target,
		     const Eigen::Matrix4d& initial_covariance )
	  {
	    range = target.range;
	    speed = target.speed;
	    azimuth = target.azimuth;
	    elevation = target.elevation;

	    cov = initial_covariance;
	  }
	FilterState( const FilterState& state )
	  {
	    range = state.range;
	    speed = state.speed;
	    azimuth = state.azimuth;
	    elevation = state.elevation;

	    cov = state.cov;
	  }
	~FilterState() {}

	friend std::ostream& operator<< ( std::ostream& out, const FilterState& state )
	{
	  out << "Range: " << state.range << std::endl
	      << "Speed: " << state.speed << std::endl
	      << "Azimuth: " << state.azimuth << std::endl
	      << "Elevation: " << state.elevation << std::endl
	      << "Covariance: " << std::endl << state.cov << std::endl;
	}
	
	double range;
	double speed;
	double azimuth;
	double elevation;

	Eigen::Matrix4d cov;

	Eigen::Vector4d asVec( void ) const
	  {
	    return Eigen::Vector4d( this->range,
				    this->speed,
				    this->azimuth,
				    this->elevation );
	  }
	void fromVec( const Eigen::Vector4d& state_vec )
	  {
	    this->range = state_vec( 0 );
	    this->speed = state_vec( 1 );
	    this->azimuth = state_vec( 2 );
	    this->elevation = state_vec( 3 );
	  }

	ainstein_radar_msgs::RadarTarget asMsg( void ) const
	  {
	    ainstein_radar_msgs::RadarTarget t;
	    t.range = range;
	    t.speed = speed;
	    t.azimuth = azimuth;
	    t.elevation = elevation;

	    return t;
	  }
      };

  public:
    RadarTargetKF( const ainstein_radar_msgs::RadarTarget& target,
		   const ros::NodeHandle& node_handle,
		   const ros::NodeHandle& node_handle_private );
    ~RadarTargetKF() {}

    class FilterParameters
    {
    public:
      FilterParameters( void ) {}
      ~FilterParameters( void )	{}
      
      double init_range_stdev;
      double init_speed_stdev;
      double init_azim_stdev;
      double init_elev_stdev;

      double q_speed_stdev;
      double q_azim_stdev;
      double q_elev_stdev;
      
      double r_range_stdev;
      double r_speed_stdev;
      double r_azim_stdev;
      double r_elev_stdev;
    };

      friend std::ostream& operator<< ( std::ostream& out, const RadarTargetKF& kf )
	{
	  out << "State: " << kf.state_post_;
	  out << "Time Since Start: " << kf.getTimeSinceStart() << std::endl
	      << "Time Since Update: " << kf.getTimeSinceUpdate() << std::endl;
	}
	
      void process( double dt );
      void update( const ainstein_radar_msgs::RadarTarget& target );

      FilterState getState( void ) const
      {
	return state_post_;
      }
      Eigen::Vector4d computePredMeas( const FilterState& state )
	{
	  return H_ * state.asVec();
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
      
      Eigen::Matrix4d K_;

      // Shared among all filters:
      static Eigen::Matrix4d F_;
      static Eigen::Matrix<double, 4, 3> L_;
      static Eigen::Matrix4d H_;

      static Eigen::Matrix3d Q_;
      static Eigen::Matrix4d R_;

      static Eigen::Matrix4d P_init_;
  };

} // namespace ainstein_radar_filters

#endif // RADAR_TARGET_KF_H_
