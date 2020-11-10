#ifndef BOUNDING_BOX_ARRAY_DISPLAY_H
#define BOUNDING_BOX_ARRAY_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz/message_filter_display.h>
#include <rviz/tool.h>

#include <ainstein_radar_msgs/BoundingBoxArray.h>
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
  class BoundingBoxArrayVisual;
  
  // Declare a new subclass of rviz::Display.
  class BoundingBoxArrayDisplay: public rviz::MessageFilterDisplay<ainstein_radar_msgs::BoundingBoxArray>
  {
    Q_OBJECT
  public:
    // Default constructor for the pluginlib::ClassLoader.
    BoundingBoxArrayDisplay();
    virtual ~BoundingBoxArrayDisplay();

  protected:
    // Initialize class members.
    virtual void onInitialize();
    
    // A helper to clear this display back to the initial state.
    virtual void reset();

    // These Qt slots get connected to signals indicating changes in the user-editable properties.
    private Q_SLOTS:
      void updateColorAndAlpha();

      // Function to handle an incoming ROS message.
  private:
      void processMessage( const ainstein_radar_msgs::BoundingBoxArray::ConstPtr& msg );
      
      // Storage for the visual.
      boost::shared_ptr<BoundingBoxArrayVisual> visual_;

      // User-editable property variables.
      std::unique_ptr<rviz::ColorProperty> color_property_;
      std::unique_ptr<rviz::FloatProperty> alpha_property_;
  };
  
} // namespace ainstein_radar_rviz_plugins

#endif // BOUNDING_BOX_ARRAY_DISPLAY_H

