#ifndef RADAR_DISPLAY_H_
#define RADAR_DISPLAY_H_

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

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace ainstein_radar_rviz_plugins
{

class RadarVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// RadarDisplay will show a 3D arrow showing the direction and magnitude
// of the RADAR acceleration vector.  The base of the arrow will be at
// the frame listed in the header of the Radar message, and the
// direction of the arrow will be relative to the orientation of that
// frame.  It will also optionally show a history of recent
// acceleration vectors, which will be stored in a circular buffer.
//
// The RadarDisplay class itself just implements the circular buffer,
// editable parameters, and Display subclass machinery.  The visuals
// themselves are represented by a separate class, RadarVisual.  The
// idiom for the visuals is that when the objects exist, they appear
// in the scene, and when they are deleted, they disappear.
 class RadarDisplay: public rviz::MessageFilterDisplay<ainstein_radar_msgs::RadarTargetArray>
 {
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  RadarDisplay();
  virtual ~RadarDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
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
  boost::circular_buffer<boost::shared_ptr<RadarVisual>> visuals_;

  // User-editable property variables.
  std::unique_ptr<rviz::ColorProperty> color_property_;
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

} // end namespace ainstein_radar_rviz_plugins

#endif // RADAR_DISPLAY_H_

