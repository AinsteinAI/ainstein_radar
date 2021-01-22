#ifndef RADAR_TRACKED_OBJECT_VISUAL_H
#define RADAR_TRACKED_OBJECT_VISUAL_H

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>

namespace rviz
{
  class Shape;
  class Arrow;
}

namespace ainstein_radar_rviz_plugins
{
  class RadarTrackedObjectVisual
  {
  public:
    RadarTrackedObjectVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz::Shape::Type type ) :
      pos( type, scene_manager, parent_node ),
      vel( scene_manager, parent_node ),
      box( rviz::Shape::Cube, scene_manager, parent_node )
    {
	  }      
    ~RadarTrackedObjectVisual() {};

    rviz::Shape pos;
    rviz::Arrow vel;
    rviz::Shape box;

    int id;
  };

} // namespace ainstein_radar_rviz_plugins

#endif // RADAR_TRACKED_OBJECT_VISUAL_H
