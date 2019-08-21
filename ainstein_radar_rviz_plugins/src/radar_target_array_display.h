#ifndef RADAR_TARGET_ARRAY_DISPLAY_H
#define RADAR_TARGET_ARRAY_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <rviz/tool.h>

#include <ainstein_radar_msgs/RadarTargetArray.h>
#endif 

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
  class ColorProperty;
  class FloatProperty;
  class IntProperty;
  class BoolProperty;
  class EnumProperty;
  class ViewportMouseEvent;
}

namespace ainstein_radar_rviz_plugins
{  
  class RadarTargetArrayVisual;
  
  // Declare a new subclass of rviz::Display.
  class RadarTargetArrayDisplay: public rviz::MessageFilterDisplay<ainstein_radar_msgs::RadarTargetArray>
  {
    Q_OBJECT
  public:
    // Default constructor for the pluginlib::ClassLoader.
    RadarTargetArrayDisplay();
    virtual ~RadarTargetArrayDisplay();

    // Color method "defines".
    static const int COLOR_METHOD_FLAT = 0;
    static const int COLOR_METHOD_COLLISION_TIME = 1;

  protected:
    // Initialize class members.
    virtual void onInitialize();
    
    // A helper to clear this display back to the initial state.
    virtual void reset();

    // These Qt slots get connected to signals indicating changes in the user-editable properties.
    private Q_SLOTS:
      void updateColorAndAlpha();
      void updateScale();
      void updateHistoryLength();
      void updateMinRange();
      void updateMaxRange();
      void updateShowSpeedArrows();
      void updateShowTargetInfo();
      void updateInfoTextHeight();
      void updateTargetShape();

      // Function to handle an incoming ROS message.
  private:
      void processMessage( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg );
      
      // Storage for the list of visuals.  It is a circular buffer where
      // data gets popped from the front (oldest) and pushed to the back (newest)
      boost::circular_buffer<boost::shared_ptr<RadarTargetArrayVisual>> visuals_;

      // User-editable property variables.
      std::unique_ptr<rviz::ColorProperty> color_property_;
      std::unique_ptr<rviz::EnumProperty> color_method_property_;
      std::unique_ptr<rviz::FloatProperty> alpha_property_;
      std::unique_ptr<rviz::FloatProperty> scale_property_;
      std::unique_ptr<rviz::EnumProperty> shape_property_;
  
      std::unique_ptr<rviz::IntProperty> history_length_property_;
      std::unique_ptr<rviz::FloatProperty> min_range_property_;
      std::unique_ptr<rviz::FloatProperty> max_range_property_;
      std::unique_ptr<rviz::BoolProperty> show_speed_property_;
      std::unique_ptr<rviz::BoolProperty> show_info_property_;
      std::unique_ptr<rviz::FloatProperty> info_text_height_property_;
  };
  
} // namespace ainstein_radar_rviz_plugins

#endif // RADAR_TARGET_ARRAY_DISPLAY_H

