#ifndef BOUNDING_BOX_ARRAY_VISUAL_H
#define BOUNDING_BOX_ARRAY_VISUAL_H

#include <rviz/ogre_helpers/shape.h>

#include <ainstein_radar_msgs/BoundingBoxArray.h>
#include "bounding_box_visual.h"

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
// Each instance of BoundingBoxArrayVisual represents the visualization of
// a single ainstein_radar_msgs::BoundingBoxArray message.
class BoundingBoxArrayVisual
{
public:
  BoundingBoxArrayVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );      
  virtual ~BoundingBoxArrayVisual();

  void setMessage( const ainstein_radar_msgs::BoundingBoxArray::ConstPtr& msg );
  void clearMessage( void );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way BoundingBoxArrayVisual is
  // only responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual:
  void setColor( float r, float g, float b, float a );
    
  // Maximum number of bounding box visuals:
  static const int max_bounding_box_visuals;
  
private:
  // The objects contained in the visual:
  std::vector<BoundingBoxVisual> bounding_box_visuals_;
  
  // A SceneNode whose pose is set to match the coordinate frame of
  // the Radar message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};
 
} // namespace ainstein_radar_rviz_plugins

#endif // BOUNDING_BOX_ARRAY_VISUAL_H
