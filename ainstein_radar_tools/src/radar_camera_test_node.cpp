#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <ainstein_radar_filters/radar_target_array_to_point_cloud.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_tools
{

  #define RECT_THICKNESS 4
  #define SNR_MIN 100.0
  #define SNR_MAX 200.0

  #define RANGE_ACC 0.3
  #define AZIM_ACC ( M_PI / 180.0 ) * 1.5
  
  class RadarCameraTest
  {
  public:
    RadarCameraTest(  ros::NodeHandle node_handle,
		      ros::NodeHandle node_handle_private ) :
      nh_( node_handle ),
      nh_private_( node_handle_private ),
      it_( nh_ ),
      listen_tf_( buffer_tf_ )
    {
      sub_radar_ = nh_.subscribe( "radar_topic", 1, &RadarCameraTest::radarCallback, this );
      sub_image_ = it_.subscribeCamera( "camera_topic", 1, &RadarCameraTest::imageCallback, this );
      pub_image_ = it_.advertise( "/radar_camera_test/image_out", 1 );
    }
    ~RadarCameraTest( void ){}

    void radarCallback( const ainstein_radar_msgs::RadarTargetArray& targets )
    {
      // Store the target array for processing in the next image callback
      targets_msg_ = targets;
    }
    
    void imageCallback( const sensor_msgs::ImageConstPtr& image_msg,
			const sensor_msgs::CameraInfoConstPtr& info_msg )
    {
      // Get the input image and convert to OpenCV
      cv::Mat image_in, image_out;
      cv_bridge::CvImagePtr input_bridge, output_bridge;
      try
	{
	  input_bridge = cv_bridge::toCvCopy( image_msg, sensor_msgs::image_encodings::BGR8 );
	  image_in = input_bridge->image;

	  output_bridge = cv_bridge::toCvCopy( image_msg, sensor_msgs::image_encodings::BGR8 );
	  image_out = output_bridge->image;
	}
      catch( cv_bridge::Exception& e )
	{
	  ROS_ERROR( "Failed to load input image" );
	}

      // Build the camera model from CameraInfo msg data
      cam_model_.fromCameraInfo( info_msg );

      // Get the transform from radar to camera frame 
      Eigen::Affine3d tf_radar_to_camera;
      if( buffer_tf_.canTransform( "camera_color_optical_frame", "radar_frame", ros::Time( 0 ) ) )
	{
	  tf_radar_to_camera =
	    tf2::transformToEigen( buffer_tf_.lookupTransform( "camera_color_optical_frame", "radar_frame", ros::Time( 0 ) ) );
	}
      else
	{
	  ROS_WARN( "Timeout while waiting for transform." );
	}

      // Transform radar data to Cartesian coordinates and into camera frame
      PointRadarTarget pcl_point;
      cv::Point2d uv, uv_top_left, uv_bot_right;
      cv::Mat marker_overlay;
      for( const auto& t : targets_msg_.targets )
	{
	  // Convert the radar target to a 3d point
	  ainstein_radar_filters::RadarTargetArrayToPointCloud::radarTargetToPclPoint( t, pcl_point );
	  Eigen::Vector3d target_point = Eigen::Vector3d( pcl_point.x, pcl_point.y, pcl_point.z );

	  // Transform the 3d point into the camera frame
	  Eigen::Vector3d target_point_camera_frame = tf_radar_to_camera.linear() * target_point;
	 
	  // Form the camera frame rotated to align z-axis with the target
	  Eigen::Matrix3d rot_mat_camera_to_camera_target_aligned = Eigen::AngleAxisd( ( M_PI / 180.0 ) * pcl_point.azimuth,
										       Eigen::Vector3d::UnitY() ).toRotationMatrix();
	  
	  // Compute corners of the box in 3d camera frame based on radar specs
	  double box_size = pcl_point.range * tan( AZIM_ACC ); 
	  Eigen::Vector3d rect_top_left = target_point_camera_frame +
	    rot_mat_camera_to_camera_target_aligned.transpose() * Eigen::Vector3d( -box_size, -box_size, 0.0 );
	  Eigen::Vector3d rect_bot_right = target_point_camera_frame +
	    rot_mat_camera_to_camera_target_aligned.transpose() * Eigen::Vector3d( box_size, box_size, 0.0 );
	  
	  // Project the 3d point to 2d pixel coordinates
	  uv = cam_model_.project3dToPixel( cv::Point3d( target_point_camera_frame.x(), target_point_camera_frame.y(), target_point_camera_frame.z() ) );
	  uv_top_left = cam_model_.project3dToPixel( cv::Point3d( rect_top_left.x(), rect_top_left.y(), rect_top_left.z() ) );
	  uv_bot_right = cam_model_.project3dToPixel( cv::Point3d( rect_bot_right.x(), rect_bot_right.y(), rect_bot_right.z() ) );

	  // Scale the size of the rectangle:
	  double rect_size = 50.0 * ( 1.0 - ( pcl_point.range / 30.0 ) ); 
	  rect_size = rect_size < 0.0 ? 0.0 : rect_size;
	  	  
	  // Compute rectangle coorners in pixel coordinates
	  uv_top_left = uv - cv::Point2d( rect_size, rect_size ); 
	  uv_bot_right = uv + cv::Point2d( rect_size, rect_size );

	  // Scale the transparency of the rectangle based on SNR:
	  double rect_alpha = std::max( 0.0, std::min( 1.0, 1.0 - ( ( SNR_MAX - pcl_point.snr ) / ( SNR_MAX - SNR_MIN ) ) ) );
	  
	  // Render the projected point to the marker overlay
	  image_in.copyTo( marker_overlay );
	  cv::rectangle( marker_overlay, uv_top_left, uv_bot_right, CV_RGB( 255, 0, 0 ), RECT_THICKNESS );

	  // Add text:
	  // cv::putText( marker_overlay, "test", uv_top_left, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255,255) );
	  
	  // Add the marker overlay to the original image with transparency
	  cv::addWeighted( marker_overlay, rect_alpha, image_out, 1.0 - rect_alpha, 0, image_out ); 
	}
      
      // Publish the modified image
      pub_image_.publish( output_bridge->toImageMsg() );
    }

  private:
    ros::NodeHandle nh_, nh_private_;

    ros::Subscriber sub_radar_;
    ainstein_radar_msgs::RadarTargetArray targets_msg_;
    
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_image_;
    image_transport::Publisher pub_image_;
    
    image_geometry::PinholeCameraModel cam_model_;

    tf2_ros::TransformListener listen_tf_;
    tf2_ros::Buffer buffer_tf_;
  };

} // namespace ainstein_radar_tools

int main( int argc, char** argv )
{
  ros::init( argc, argv, "radar_camera_test" );
  ainstein_radar_tools::RadarCameraTest cam_test( ros::NodeHandle(), ros::NodeHandle( "~" ) );
  ros::spin();
}
