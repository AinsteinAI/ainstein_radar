#include <ainstein_radar_tools/radar_camera_validation.h>
#include <ainstein_radar_filters/data_conversions.h>

namespace ainstein_radar_tools
{

  const double RadarCameraValidation::SNR_MIN = 0.0;
  const double RadarCameraValidation::SNR_MAX = 200.0;
  const int RadarCameraValidation::RECT_THICKNESS = 4;
  
  void RadarCameraValidation::imageCallback( const sensor_msgs::ImageConstPtr& image_msg,
					     const sensor_msgs::CameraInfoConstPtr& image_info_msg )
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
    cam_model_.fromCameraInfo( image_info_msg );

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
	ainstein_radar_filters::data_conversions::radarTargetToPclPoint( t, pcl_point );
	Eigen::Vector3d target_point = Eigen::Vector3d( pcl_point.x, pcl_point.y, pcl_point.z );

	// Transform the 3d point into the camera frame
	Eigen::Vector3d target_point_camera_frame = tf_radar_to_camera.linear() * target_point;
	 
	// Form the camera frame rotated to align z-axis with the target
	Eigen::Matrix3d rot_mat_camera_to_camera_target_aligned = Eigen::AngleAxisd( ( M_PI / 180.0 ) * pcl_point.azimuth,
										     Eigen::Vector3d::UnitY() ).toRotationMatrix();
	  
	// Compute corners of the box in 3d camera frame based on radar specs
	double box_size = pcl_point.range * tan( radar_info_msg_.azimuth_accuracy ); 
	Eigen::Vector3d rect_top_left = target_point_camera_frame +
	  rot_mat_camera_to_camera_target_aligned.transpose() * Eigen::Vector3d( -box_size, -box_size, 0.0 );
	Eigen::Vector3d rect_bot_right = target_point_camera_frame +
	  rot_mat_camera_to_camera_target_aligned.transpose() * Eigen::Vector3d( box_size, box_size, 0.0 );
	  
	// Project the 3d point to 2d pixel coordinates
	uv = cam_model_.project3dToPixel( cv::Point3d( target_point_camera_frame.x(), target_point_camera_frame.y(), target_point_camera_frame.z() ) );
	uv_top_left = cam_model_.project3dToPixel( cv::Point3d( rect_top_left.x(), rect_top_left.y(), rect_top_left.z() ) );
	uv_bot_right = cam_model_.project3dToPixel( cv::Point3d( rect_bot_right.x(), rect_bot_right.y(), rect_bot_right.z() ) );

	// only render if the center of the rectangle lies inside the image
	if( ( uv.x >= 0 && uv.x <= image_info_msg->width ) && ( uv.y >= 0 && uv.y <= image_info_msg->height ) )
	  {
	    // Scale the size of the rectangle:
	    double rect_size = 50.0 * ( 1.0 - ( pcl_point.range / 30.0 ) ); 
	    rect_size = rect_size < 0.0 ? 0.0 : rect_size;
	    
	    // Compute rectangle coorners in pixel coordinates
	    uv_top_left = uv - cv::Point2d( rect_size, rect_size ); 
	    uv_bot_right = uv + cv::Point2d( rect_size, rect_size );

	    // Scale the transparency of the rectangle based on SNR:
	    if( use_snr_alpha_ )
	      {
		double rect_alpha = std::max( 0.0, std::min( 1.0, 1.0 - ( ( RadarCameraValidation::SNR_MAX - pcl_point.snr ) / ( RadarCameraValidation::SNR_MAX - RadarCameraValidation::SNR_MIN ) ) ) );

		// Render the projected point to the marker overlay
		image_in.copyTo( marker_overlay );
		cv::rectangle( marker_overlay, uv_top_left, uv_bot_right, CV_RGB( 255, 0, 0 ), RadarCameraValidation::RECT_THICKNESS );

		// Add text:
		// cv::putText( marker_overlay, "test", uv_top_left, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255,255) );
	    
		// Add the marker overlay to the original image with transparency
		cv::addWeighted( marker_overlay, rect_alpha, image_out, 1.0 - rect_alpha, 0, image_out );
	      }
	    else
	      {
		cv::rectangle( image_out, uv_top_left, uv_bot_right, CV_RGB( 255, 0, 0 ), RadarCameraValidation::RECT_THICKNESS );
	      }		
	  }
      }
      
    // Publish the modified image
    pub_image_.publish( output_bridge->toImageMsg() );
  }

} // namespace ainstein_radar_tools
