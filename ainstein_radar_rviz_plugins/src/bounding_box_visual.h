#ifndef BOUNDING_BOX_VISUAL_H
#define BOUNDING_BOX_VISUAL_H

#include <rviz/ogre_helpers/shape.h>

namespace rviz
{
  class Shape;
}

namespace ainstein_radar_rviz_plugins
{
  // Create an aggregate class for different types of basic visual elements
  // to be used in visualizing radar data.
  //
  // Each BoundingBoxVisual contains a Shape for the box itself.
  class BoundingBoxVisual
  {
  public:
    BoundingBoxVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) :
      box( rviz::Shape::Cube, scene_manager, parent_node )
    {
	  }      
    ~BoundingBoxVisual() {};

    rviz::Shape box;
  };

} // namespace ainstein_radar_rviz_plugins

#endif // BOUNDING_BOX_VISUAL_H
