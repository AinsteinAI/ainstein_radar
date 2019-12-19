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

#include "ainstein_radar_tools/radar_camera_fusion.h"
#include <ainstein_radar_filters/data_conversions.h>

namespace ainstein_radar_tools
{
  // 128 predefined distinct RGB colors in hex format
  const int RadarCameraFusion::colormap[] = // copied from https://stackoverflow.com/a/20298116
    {
      0x000000, 0xFFFF00, 0x1CE6FF, 0xFF34FF, 0xFF4A46, 0x008941, 0x006FA6, 0xA30059,
      0xFFDBE5, 0x7A4900, 0x0000A6, 0x63FFAC, 0xB79762, 0x004D43, 0x8FB0FF, 0x997D87,
      0x5A0007, 0x809693, 0xFEFFE6, 0x1B4400, 0x4FC601, 0x3B5DFF, 0x4A3B53, 0xFF2F80,
      0x61615A, 0xBA0900, 0x6B7900, 0x00C2A0, 0xFFAA92, 0xFF90C9, 0xB903AA, 0xD16100,
      0xDDEFFF, 0x000035, 0x7B4F4B, 0xA1C299, 0x300018, 0x0AA6D8, 0x013349, 0x00846F,
      0x372101, 0xFFB500, 0xC2FFED, 0xA079BF, 0xCC0744, 0xC0B9B2, 0xC2FF99, 0x001E09,
      0x00489C, 0x6F0062, 0x0CBD66, 0xEEC3FF, 0x456D75, 0xB77B68, 0x7A87A1, 0x788D66,
      0x885578, 0xFAD09F, 0xFF8A9A, 0xD157A0, 0xBEC459, 0x456648, 0x0086ED, 0x886F4C,

      0x34362D, 0xB4A8BD, 0x00A6AA, 0x452C2C, 0x636375, 0xA3C8C9, 0xFF913F, 0x938A81,
      0x575329, 0x00FECF, 0xB05B6F, 0x8CD0FF, 0x3B9700, 0x04F757, 0xC8A1A1, 0x1E6E00,
      0x7900D7, 0xA77500, 0x6367A9, 0xA05837, 0x6B002C, 0x772600, 0xD790FF, 0x9B9700,
      0x549E79, 0xFFF69F, 0x201625, 0x72418F, 0xBC23FF, 0x99ADC0, 0x3A2465, 0x922329,
      0x5B4534, 0xFDE8DC, 0x404E55, 0x0089A3, 0xCB7E98, 0xA4E804, 0x324E72, 0x6A3A4C,
      0x83AB58, 0x001C1E, 0xD1F7CE, 0x004B28, 0xC8D0F6, 0xA3A489, 0x806C66, 0x222800,
      0xBF5650, 0xE83000, 0x66796D, 0xDA007C, 0xFF1A59, 0x8ADBB4, 0x1E0200, 0x5B4E51,
      0xC895C5, 0x320033, 0xFF6832, 0x66E1D3, 0xCFCDAC, 0xD0AC94, 0x7ED379, 0x012C58
    };
  
  // Constants for OpenCV rendering
  const int RadarCameraFusion::text_fontface = cv::FONT_HERSHEY_SIMPLEX;
  const double RadarCameraFusion::text_scale = 0.6;
  const int RadarCameraFusion::text_thickness = 1;
  const int RadarCameraFusion::text_offset = 5;
  const int RadarCameraFusion::text_box_offset = 0;
  const int RadarCameraFusion::text_height_offset = 5;
  const int RadarCameraFusion::bbox_thickness = 1;
    
  // Helper function to convert to string with specified number of decimals
  template <typename T>
  std::string to_string_with_precision(const T a_value, const int n = 2)
  {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
  }

  // Helper function to convert hex int to OpenCV RGB (scalar) color
  cv::Scalar hex_to_RGB( int hexValue )
  {
    cv::Scalar rgbColor;
    int r = ((hexValue >> 16) & 0xFF);  // Extract the RR byte
    int g = ((hexValue >> 8) & 0xFF);   // Extract the GG byte
    int b = ((hexValue) & 0xFF);        // Extract the BB byte
    return CV_RGB( r, g, b );
  }
    
  RadarCameraFusion::RadarCameraFusion( ros::NodeHandle node_handle,
					ros::NodeHandle node_handle_private ) :
  nh_( node_handle ),
  nh_private_( node_handle_private ),
  it_( nh_ ),
  it_private_( nh_private_ ),
  listen_tf_( buffer_tf_ )
  {
    sub_radar_ = nh_.subscribe( "radar_topic", 1, &RadarCameraFusion::radarCallback, this );
    sub_radar_bbox_ = nh_.subscribe( "radar_bbox_topic", 1, &RadarCameraFusion::radarBboxCallback, this );
    sub_objects_ = nh_.subscribe( "objects_topic", 1, &RadarCameraFusion::objectsCallback, this );
    sub_image_ = it_.subscribeCamera( "camera_topic", 1, &RadarCameraFusion::imageCallback, this );

    pub_image_ = it_private_.advertise( "image_out", 1 );
    pub_bounding_boxes_ = nh_private_.advertise<jsk_recognition_msgs::BoundingBoxArray>( "boxes", 1 );

    has_radar_boxes_ = false;
    nh_private_.param( "use_object_width_for_bbox", use_object_width_for_bbox_, false );
    
    // Wait for the detection node to be ready
    ros::service::waitForService( "object_detector_is_ready", ros::Duration( 5 ) );
      
    // Get the list of object labels per id
    if( !nh_.getParam( "object_labels", object_labels_ ) )
      {
	ROS_WARN_STREAM( "Failed to load object labels!" );
      }
  }

  void RadarCameraFusion::radarCallback( const ainstein_radar_msgs::RadarTargetArray& targets )
  {
    // Store the target array for processing in the next image callback
    targets_msg_ = targets;
  }
  
  void RadarCameraFusion::radarBboxCallback( const jsk_recognition_msgs::BoundingBoxArray& bboxes )
  {
    // Store the radar bounding box array for processing in the next image callback
    radar_boxes_msg_ = bboxes;

    // Enables optional processing when boxes are available
    if( radar_boxes_msg_.boxes.size() > 0 )
      {
	has_radar_boxes_ = true;
      }
  }
  
  void RadarCameraFusion::objectsCallback( const vision_msgs::Detection2DArray& objects )
  {
    // Store the detected objects for processing in the next image callback
    objects_msg_ = objects;
  }
  
  void RadarCameraFusion::imageCallback( const sensor_msgs::ImageConstPtr& image_msg,
					 const sensor_msgs::CameraInfoConstPtr& info_msg ) 
  {
    // Get the input image and convert to OpenCV
    cv::Mat image_in;
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy( image_msg, sensor_msgs::image_encodings::BGR8 );
	image_in = cv_ptr->image;
      }
    catch( cv_bridge::Exception& e )
      {
	ROS_ERROR_STREAM( "Failed to load input image, exception " << e.what() );
	return;
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
	ROS_WARN_STREAM( "Timeout while waiting for transform." );
      }

    // Get the transform from camera to world frame 
    Eigen::Affine3d tf_camera_to_world;
    if( buffer_tf_.canTransform( "map", "camera_color_optical_frame", ros::Time( 0 ) ) )
      {
	tf_camera_to_world =
	  tf2::transformToEigen( buffer_tf_.lookupTransform( "map", "camera_color_optical_frame", ros::Time( 0 ) ) );
      }
    else
      {
	ROS_WARN_STREAM( "Timeout while waiting for transform." );
      }

    // Transform targets to Cartesian coordinates and project to camera frame
    uv_targets_.clear();

    for( const auto& t : targets_msg_.targets )
      {
	// Convert the radar target to a 3d Cartesian point
	PointRadarTarget pcl_point;
	ainstein_radar_filters::data_conversions::radarTargetToPclPoint( t, pcl_point );
	Eigen::Vector3d target_point = Eigen::Vector3d( pcl_point.x, pcl_point.y, pcl_point.z );

	// Transform the 3d point into the camera coordinate frame
	Eigen::Vector3d target_point_camera_frame = tf_radar_to_camera.linear() * target_point;
	  
	// Project the 3d point to 2d pixel coordinates
	uv_targets_.push_back( cam_model_.project3dToPixel( cv::Point3d( target_point_camera_frame.x(), target_point_camera_frame.y(), target_point_camera_frame.z() ) ) );
      }

    // Clear the output bounding box message if applicable
    if( has_radar_boxes_ )
      {
	boxes_msg_.boxes.clear();
	boxes_msg_.header = radar_boxes_msg_.header;
      }
	
    //ROS_INFO_STREAM( "Check projected targets against detected object boxes" );
    
    // Check projected targets against each detected object bounding box.
    // In the case that multiple 3d points alias to be within the same 2d
    // bounding box (bbox), only use the nearest one. If radar and camera
    // data overlap, render the detected object and display radar data as
    // bounding box annotations.
    double min_range;
    int ind_min_range;
    for( const auto& object : objects_msg_.detections )
      {
	// Set the min range to infinity before checking targets
	min_range = std::numeric_limits<double>::infinity();
	ind_min_range = -1;
	
	for( int i = 0; i < uv_targets_.size(); ++i )
	  {
	    cv::Point2d uv_target = uv_targets_.at( i );

	    // Check if target is within bbox
	    if( ( uv_target.x <= ( object.bbox.center.x + object.bbox.size_x ) ) &&
		( uv_target.x >= ( object.bbox.center.x - object.bbox.size_x ) ) &&
		( uv_target.y <= ( object.bbox.center.y + object.bbox.size_y ) ) &&
		( uv_target.y >= ( object.bbox.center.y - object.bbox.size_y ) ) )
	      {
		// Keep track of minimum range and target index
		if( targets_msg_.targets.at( i ).range < min_range )
		  {
		    min_range = targets_msg_.targets.at( i ).range;
		    ind_min_range = i;
		  }
	      }
	  }

	// If we have a valid radar-camera association, render it
	if( std::isfinite( min_range ) && ind_min_range >= 0 )
	  {
	    // Get the center of the detected object's 2d bounding box
	    cv::Point2d uv_bbox_center = cv::Point2d( object.bbox.center.x, object.bbox.center.y );
		  
	    // Compute bounding box corners in pixel coordinates
	    cv::Point2d uv_bbox_top_left = uv_bbox_center -
	      cv::Point2d( 0.5 * object.bbox.size_x, 0.5 * object.bbox.size_y );
	    cv::Point2d uv_bbox_bot_right = uv_bbox_center +
	      cv::Point2d( 0.5 * object.bbox.size_x, 0.5 * object.bbox.size_y );

	    // Set the 3d bounding box geometry
	    if( has_radar_boxes_ )
	      {
		jsk_recognition_msgs::BoundingBox box_3d = radar_boxes_msg_.boxes.at( ind_min_range );
		
		// Project the image coordinates bbox corners to 3d rays in camera frame
		cv::Point3d cam_bbox_top_left = cam_model_.projectPixelTo3dRay( uv_bbox_top_left );
		cv::Point3d cam_bbox_bot_right = cam_model_.projectPixelTo3dRay( uv_bbox_bot_right );

		// Convert 3d rays to unit Eigen vectors
		Eigen::Vector3d unit_bbox_top_left = Eigen::Vector3d( cam_bbox_top_left.x,
								      cam_bbox_top_left.y,
								      cam_bbox_top_left.z );
		unit_bbox_top_left *= ( 1.0 / unit_bbox_top_left.norm() );
		
		Eigen::Vector3d unit_bbox_bot_right = Eigen::Vector3d( cam_bbox_bot_right.x,
								       cam_bbox_bot_right.y,
								       cam_bbox_bot_right.z );
		unit_bbox_bot_right *= ( 1.0 / unit_bbox_bot_right.norm() );

		// Set the bounding box width from the detected object width
		if( use_object_width_for_bbox_ )
		  {
		    box_3d.dimensions.y = ( targets_msg_.targets.at( ind_min_range ).range *
					    tf_camera_to_world.linear() * ( unit_bbox_top_left - unit_bbox_bot_right ) ).y();
		  }

		// Set the bounding box height from the detected object height
		box_3d.dimensions.z = ( targets_msg_.targets.at( ind_min_range ).range *
					tf_camera_to_world.linear() * ( unit_bbox_top_left - unit_bbox_bot_right ) ).z();

		boxes_msg_.boxes.push_back( box_3d );
	      }
	    
	    // Get the detected object's class ID
	    int object_id = object.results.at( 0 ).id;
	  
	    // Define the bounding box and text and textbox colors based on object ID.
	    // The colors come from a predefined set of 128 colors defined above. The
	    // text color is set to the inverse of the textbox color for readability.
	    cv::Scalar bbox_color = hex_to_RGB( colormap[object_id] );
	    cv::Scalar text_box_color = bbox_color;
	    cv::Scalar text_color = hex_to_RGB( 0xFFFFFF - colormap[object_id] );
	      
	    // Scale the transparency of the rendered data
	    double object_alpha = 1.0; //targets_msg_.targets.at( ind_min_range ).snr;
	  
	    // Create an overlay for blending annotations at desired transparency
	    cv::Mat marker_overlay;
	    image_in.copyTo( marker_overlay );
	      
	    // Render the bounding box itself
	    cv::rectangle( marker_overlay, uv_bbox_top_left, uv_bbox_bot_right, bbox_color, bbox_thickness );

	    // Add text to display information. Note that labels rendered in reverse
	    // order with respect to order they're pushed back here.
	    std::vector<std::string> text_labels;
	    text_labels.push_back( "Speed (m/s): " + to_string_with_precision( targets_msg_.targets.at( ind_min_range ).speed ) );
	    text_labels.push_back( "Range (m): " + to_string_with_precision( targets_msg_.targets.at( ind_min_range ).range ) );

	    // Only use the object label string if the database was loaded correctly
	    if( object_labels_.size() > 0 )
	      {
		text_labels.push_back( object_labels_.at( std::to_string( object_id ) ) +
				       " (" + to_string_with_precision( 100.0 * object.results.at( 0 ).score, 0 ) + "%)" );
	      }
	    else
	      {
		text_labels.push_back( std::to_string( object_id ) +
				       " (" + to_string_with_precision( 100.0 * object.results.at( 0 ).score, 1 ) + "%)" );

	      }
	    
	    // Get the max text size among all labels, used to render the textbox size correctly
	    cv::Size max_text_size( 0, 0 );
	    int baseline = 0; // output needed for call to getTextSize
	    for( const auto& label : text_labels )
	      {
		cv::Size text_size = cv::getTextSize( label,
						      RadarCameraFusion::text_fontface,
						      RadarCameraFusion::text_scale,
						      RadarCameraFusion::text_thickness,
						      &baseline );
		
		// Keep track of the maximum text width/height, adding an offset for vertical spacing
		if( text_size.width > max_text_size.width )
		  {
		    max_text_size.width = text_size.width;
		  }
		if( text_size.height > max_text_size.height )
		  {
		    max_text_size.height = text_size.height + RadarCameraFusion::text_height_offset;
		  }	
	      }

	    // Compute the origin (lower left corner) for each of the textbox and text itself. The
	    // textbox offset sets spacing (in pixels) between the top of the bounding box and the
	    // bottom of the first textbox. The text offset sets spacing (in pixels) between the
	    // bottom of each textbox and the text inside it. These were tuned visually.
	    cv::Point2d text_box_origin = uv_bbox_top_left +
	      cv::Point2d( 0, -RadarCameraFusion::text_box_offset );
	    cv::Point2d text_origin = uv_bbox_top_left +
	      cv::Point2d( 0, -RadarCameraFusion::text_offset );

	    // Render the textbox and text inside it for each text label
	    for( int i = 0; i < text_labels.size(); ++i )
	      {
		// Render the textbox
		cv::rectangle( marker_overlay, text_box_origin + cv::Point2d( 0, -i * max_text_size.height ),
			       text_origin + cv::Point2d( max_text_size.width, -(i + 1) * max_text_size.height ),
			       text_box_color, CV_FILLED);

		// Render the text inside the above textbox
		cv::putText( marker_overlay, text_labels.at( i ), text_origin + cv::Point2d( 0, -i * max_text_size.height),
			     RadarCameraFusion::text_fontface,
			     RadarCameraFusion::text_scale,
			     text_color,
			     RadarCameraFusion::text_thickness );
	      }
	      
	    // Add the annotated overlay to the original image with transparency
	    cv::addWeighted( marker_overlay, object_alpha, image_in, 1.0 - object_alpha, 0, image_in ); 
	  }
      }    
      
    // Publish the modified image
    pub_image_.publish( cv_ptr->toImageMsg() );

    // Publish the 3d bounding boxes
    pub_bounding_boxes_.publish( boxes_msg_ );
  }


  
} // namespace ainstein_radar_tools
