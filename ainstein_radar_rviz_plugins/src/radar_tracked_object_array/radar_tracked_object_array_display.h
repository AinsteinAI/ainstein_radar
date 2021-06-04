#ifndef RADAR_TRACKED_OBJECT_ARRAY_DISPLAY_H
#define RADAR_TRACKED_OBJECT_ARRAY_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz/message_filter_display.h>
#include <rviz/tool.h>

#include <ainstein_radar_msgs/RadarTrackedObjectArray.h>
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
  class RadarTrackedObjectArrayVisual;
  
  // Declare a new subclass of rviz::Display.
  class RadarTrackedObjectArrayDisplay: public rviz::MessageFilterDisplay<ainstein_radar_msgs::RadarTrackedObjectArray>
  {
    Q_OBJECT
  public:
    // Default constructor for the pluginlib::ClassLoader.
    RadarTrackedObjectArrayDisplay();
    virtual ~RadarTrackedObjectArrayDisplay();
    
    // Color method "defines".
    static const int COLOR_METHOD_FLAT = 0;
    static const int COLOR_METHOD_OBJECT_ID = 1;

  protected:
    // Initialize class members.
    virtual void onInitialize();
    
    // A helper to clear this display back to the initial state.
    virtual void reset();

    // These Qt slots get connected to signals indicating changes in the user-editable properties.
    private Q_SLOTS:
      void updateDisplayAlert();
      void updateAlertScale();
      void updateAlertRangeMax();
      void updateColorAndAlpha();
      void updateScale();
      void updateObjectShape();

      // Function to handle an incoming ROS message.
  private:
      void processMessage( const ainstein_radar_msgs::RadarTrackedObjectArray::ConstPtr& msg );
      
      // Storage for the visual.
      boost::shared_ptr<RadarTrackedObjectArrayVisual> visual_;

      // User-editable property variables.
      std::unique_ptr<rviz::ColorProperty> color_property_;
      std::unique_ptr<rviz::EnumProperty> color_method_property_;
      std::unique_ptr<rviz::BoolProperty> display_alert_property_;
      std::unique_ptr<rviz::Property> display_alert_options_property_;
      std::unique_ptr<rviz::FloatProperty> alpha_property_;
      std::unique_ptr<rviz::FloatProperty> scale_property_;
      std::unique_ptr<rviz::EnumProperty> shape_property_;
  };
  
} // namespace ainstein_radar_rviz_plugins

#endif // RADAR_TRACKED_OBJECT_ARRAY_DISPLAY_H

