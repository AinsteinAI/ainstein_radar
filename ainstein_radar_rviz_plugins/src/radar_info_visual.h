#ifndef RADAR_INFO_VISUAL_H
#define RADAR_INFO_VISUAL_H

#include <rviz/ogre_helpers/shape.h>

#include <ainstein_radar_msgs/RadarInfo.h>

namespace rviz
{
  class Shape;
  class Arrow;
  class MovableText;
}

namespace ainstein_radar_rviz_plugins
{

// Declare the visual class for this display.
//
// Each instance of RadarInfoVisual represents the visualization of
// a single sensor_msgs::RadarInfo message.
class RadarInfoVisual
{
public:
  RadarInfoVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );      
  virtual ~RadarInfoVisual();

  void setMessage( const ainstein_radar_msgs::RadarInfo::ConstPtr& msg );
  void clearMessage( void );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way RadarInfoVisual is
  // only responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the colOAor and alpha of the visual:
  void setColor( float r, float g, float b, float a );
  
private:
  // The objects contained in the info visual:
  rviz::Shape fov_cone_azimuth;
  
  // A SceneNode whose pose is set to match the coordinate frame of
  // the Radar message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};
 
} // namespace ainstein_radar_rviz_plugins

#endif // RADAR_INFO_VISUAL_H
