#include <limits>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/Detection2DArray.h>

#include <ainstein_radar_filters/radar_target_array_to_point_cloud.h>
#include <ainstein_radar_msgs/RadarTargetArray.h>

namespace ainstein_radar_tools
{
  class RadarCameraFusion
  {
  public:
    RadarCameraFusion( ros::NodeHandle node_handle,
		       ros::NodeHandle node_handle_private );
    ~RadarCameraFusion( void )
      {
      }
    
    // Store the target array for processing in the next image callback
    void radarCallback( const ainstein_radar_msgs::RadarTargetArray& targets );

    void radarBboxCallback( const jsk_recognition_msgs::BoundingBoxArray& bboxes );

    // Store the detected objects for processing in the next image callback
    void objectsCallback( const vision_msgs::Detection2DArray& objects );

    void imageCallback( const sensor_msgs::ImageConstPtr& image_msg,
			const sensor_msgs::CameraInfoConstPtr& info_msg );

    // Constants related to OpenCV rendering
    static const int colormap[];
    static const int text_fontface;
    static const double text_scale;
    static const int text_thickness;
    static const int text_offset;
    static const int text_box_offset;
    static const int text_height_offset;
    static const int bbox_thickness;
    
  private:
    ros::NodeHandle nh_, nh_private_;

    ros::Subscriber sub_radar_;
    ros::Subscriber sub_radar_bbox_;
    ainstein_radar_msgs::RadarTargetArray targets_msg_;
    jsk_recognition_msgs::BoundingBoxArray radar_boxes_msg_;

    ros::Subscriber sub_objects_;
    vision_msgs::Detection2DArray objects_msg_;
    std::map<std::string,std::string> object_labels_;
    
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_image_;
    image_transport::Publisher pub_image_;

    ros::Publisher pub_bounding_boxes_;
    jsk_recognition_msgs::BoundingBoxArray boxes_msg_;
    bool has_radar_boxes_;
    bool use_object_width_for_bbox_;
    
    image_geometry::PinholeCameraModel cam_model_;
    std::vector<cv::Point2d> uv_targets_;
    
    tf2_ros::TransformListener listen_tf_;
    tf2_ros::Buffer buffer_tf_;
  };

} // namespace ainstein_radar_tools
