#ifndef RADAR_TARGET_ARRAY_VISUAL_H
#define RADAR_TARGET_ARRAY_VISUAL_H

#include <ainstein_radar_msgs/RadarTargetArray.h>
#include "radar_target_visual.h"

namespace ainstein_radar_rviz_plugins
{

// Declare the visual class for this display.
//
// Each instance of RadarTargetArrayVisual represents the visualization of
// a single sensor_msgs::RadarTargetArray message.
class RadarTargetArrayVisual
{
public:
  RadarTargetArrayVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
  virtual ~RadarTargetArrayVisual();

  void setMessage( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg );
  void clearMessage( void );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way RadarTargetArrayVisual is
  // only responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the radar message.
  void setColor( int color_method, float r, float g, float b, float a );

  // Set the scale of the visual, which are user-editable
  // parameters and therefore don't come from the radar message.
  void setScale( float scale );

  // Set the min and max range, which are user-editable
  // parameters and therefore don't come from the radar message.
  void setMinRange( float min_range );
  void setMaxRange( float max_range );

  // Set whether to render speed arrows:
  void setShowSpeedArrows( bool show_speed_arrows );

  // Set whether to render target info text:
  void setShowTargetInfo( bool show_target_info );
  void setInfoTextHeight( float info_text_height );

  // Set target shapes for rendering:
  void setTargetShape( int type )
  {
    shape_type_ = static_cast<rviz::Shape::Type>( type );
  }
  
  // Maximum number of target visuals:
  static const int max_target_visuals;
  
private:
  // The objects implementing the radar target visuals
  std::vector<RadarTargetVisual> radar_target_visuals_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Radar message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  // Minimum and maximum range, user-configurable filtering parameters:
  float min_range_;
  float max_range_;

  // Shapes to use for rendering:
  rviz::Shape::Type shape_type_;
  
  // Determines whether speed arrows are rendered with zero length:
  bool show_speed_arrows_;

  // Determines whether target info is rendered:
  bool show_target_info_;

  // Target info character (text) height:
  float info_text_height_;
};
 
} // namespace ainstein_radar_rviz_plugins

#endif // RADAR_TARGET_ARRAY_VISUAL_H
