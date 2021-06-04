#ifndef RADAR_TRACKED_OBJECT_ARRAY_VISUAL_H
#define RADAR_TRACKED_OBJECT_ARRAY_VISUAL_H

#include <rviz/ogre_helpers/shape.h>

#include <ainstein_radar_msgs/RadarTrackedObjectArray.h>
#include "radar_tracked_object_visual.h"

namespace rviz
{
  class Shape;
  class Arrow;
  class MovableText;
}

namespace ainstein_radar_rviz_plugins
{
class RadarTrackedObjectArrayVisual
{
public:
  RadarTrackedObjectArrayVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );      
  virtual ~RadarTrackedObjectArrayVisual();

  void setMessage( const ainstein_radar_msgs::RadarTrackedObjectArray::ConstPtr& msg );
  void clearMessage( void );

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  void setColor( int color_method, float r, float g, float b, float a );
  void setScale( float scale );

  void setObjectShape( int type )
  {
    obj_shape_type_ = static_cast<rviz::Shape::Type>( type );
  }

  // Set whether to display an alert when the object is within a specified area:
  void setDisplayAlert( bool display_alert );
  void setAlertScale( float alert_scale );
  void setAlertRangeMax( float alert_range_max );

  static const int max_radar_tracked_object_visuals;
  
private:
  std::vector<RadarTrackedObjectVisual> radar_tracked_object_visuals_;
  std::map<int, Ogre::ColourValue> obj_id_colors_;
  
  Ogre::SceneNode* frame_node_;

  Ogre::SceneManager* scene_manager_;

  // Shape to use for position rendering:
  rviz::Shape::Type obj_shape_type_;

  // Nominal tracked object shape scale (size):
  float scale_;

  // Determined whether to display the object alert:
  bool display_alert_;
  float alert_scale_;
  float alert_range_max_;
};
 
} // namespace ainstein_radar_rviz_plugins

#endif // RADAR_TRACKED_OBJECT_ARRAY_VISUAL_H
