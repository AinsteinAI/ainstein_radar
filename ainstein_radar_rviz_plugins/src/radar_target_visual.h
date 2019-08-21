#ifndef RADAR_TARGET_VISUAL_H
#define RADAR_TARGET_VISUAL_H

#include <ainstein_radar_msgs/RadarTarget.h>

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/movable_text.h>

namespace rviz
{
  class Shape;
  class Arrow;
  class MovableText;
}

namespace ainstein_radar_rviz_plugins
{
  // Create an aggregate class for different types of basic visual elements
  // to be used in visualizing radar data.
  //
  // Each RadarTargetVisual contains a Shape for the target itself and an Arrow
  // to visualize the target speed (velocity). The info text is mainly for
  // debugging, used to render the radar target message on-screen in RViz.
  class RadarTargetVisual
  {
  public:
  RadarTargetVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz::Shape::Type type ) :
    pos( type, scene_manager, parent_node ),
      speed( scene_manager, parent_node ),
      info( "test" )
	{
	  parent_node->attachObject( &info );
	  info.setTextAlignment( rviz::MovableText::H_LEFT,
				 rviz::MovableText::V_CENTER );
	}      
    ~RadarTargetVisual() {};

    rviz::Shape pos;
    rviz::Arrow speed;
    rviz::MovableText info;

    ainstein_radar_msgs::RadarTarget t;
  };

} // namespace ainstein_radar_rviz_plugins

#endif // RADAR_TARGET_VISUAL_H
