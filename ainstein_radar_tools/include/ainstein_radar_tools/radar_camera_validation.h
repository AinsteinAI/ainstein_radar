#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>
#include <ainstein_radar_msgs/RadarInfo.h>

namespace ainstein_radar_tools
{

  class RadarCameraValidation
  {
  public:
    RadarCameraValidation(  ros::NodeHandle node_handle,
			    ros::NodeHandle node_handle_private ) :
      nh_( node_handle ),
      nh_private_( node_handle_private ),
      it_( nh_ ),
      it_private_( nh_private_ ), 
      listen_tf_( buffer_tf_ )
    {
      sub_radar_ = nh_.subscribe( "radar_topic", 1, &RadarCameraValidation::radarCallback, this );
      sub_radar_info_ = nh_.subscribe( "radar_info_topic", 1, &RadarCameraValidation::radarInfoCallback, this );
      sub_image_ = it_.subscribeCamera( "camera_topic", 1, &RadarCameraValidation::imageCallback, this );
      pub_image_ = it_private_.advertise( "image_out", 1 );

      nh_private_.param( "use_snr_alpha", use_snr_alpha_, false );
    }
    ~RadarCameraValidation( void ){}

    void radarCallback( const ainstein_radar_msgs::RadarTargetArray& targets )
    {
      // Store the target array for processing in the next image callback
      targets_msg_ = targets;
    }

    void radarInfoCallback( const ainstein_radar_msgs::RadarInfo& info )
    {
      // Store the radar info for use in the next image callback
      info_msg_ = info;
    }

    void imageCallback( const sensor_msgs::ImageConstPtr& image_msg,
			const sensor_msgs::CameraInfoConstPtr& info_msg );

    static const double SNR_MIN;
    static const double SNR_MAX;
    static const int RECT_THICKNESS;
    
  private:
    ros::NodeHandle nh_, nh_private_;

    ros::Subscriber sub_radar_;
    ros::Subscriber sub_radar_info_;
    ainstein_radar_msgs::RadarTargetArray targets_msg_;
    ainstein_radar_msgs::RadarInfo info_msg_;

    image_transport::ImageTransport it_, it_private_;
    image_transport::CameraSubscriber sub_image_;
    image_transport::Publisher pub_image_;

    bool use_snr_alpha_;
    
    image_geometry::PinholeCameraModel cam_model_;

    tf2_ros::TransformListener listen_tf_;
    tf2_ros::Buffer buffer_tf_;
  };

} // namespace ainstein_radar_tools
