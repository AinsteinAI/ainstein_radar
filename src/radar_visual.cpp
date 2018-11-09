#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <geometry_msgs/Vector3.h>
#include "radar_visual.h"

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
RadarVisual::RadarVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Radar's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
}

RadarVisual::~RadarVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void RadarVisual::setMessage( const radar_sensor_msgs::RadarData::ConstPtr& msg )
{
  geometry_msgs::Vector3 a;
  a.x = 0.0;
  a.y = 0.0;
  a.z = 9.81;

  // Create the shape to represent the radar target:
  int n_raw_targets = msg->raw_targets.size();
  if( n_raw_targets > 0 )
    {
      // Create the radar target shapes:
      radar_target_shapes_.resize( n_raw_targets );
      for( auto it = radar_target_shapes_.begin(); it != radar_target_shapes_.end(); ++it )
	{
	  it->reset( new rviz::Shape( rviz::Shape::Cube, scene_manager_, frame_node_ ) );
	}

      // Fill the target shapes from RadarData message:
      double r;
      double th;
      Ogre::Vector3 pos;
      Ogre::Vector3 scale;
      for( int i = 0; i < n_raw_targets; ++i )
	{
	  r = msg->raw_targets.at( i ).range;
	  th = msg->raw_targets.at( i ).azimuth;
	  pos = Ogre::Vector3(r * cos( th ),
			      r * sin( th ),
			      0.0 );
	  radar_target_shapes_.at( i )->setPosition( pos );
	  
	  scale = Ogre::Vector3( 0.1, 0.1, 0.1 );
	  radar_target_shapes_.at( i )->setScale( scale );
	}
    }
  // // Convert the geometry_msgs::Vector3 to an Ogre::Vector3.
  // Ogre::Vector3 acc( a.x, a.y, a.z );

  // // Find the magnitude of the acceleration vector.
  // float length = acc.length();

  // // Scale the arrow's thickness in each dimension along with its length.
  // Ogre::Vector3 scale( length, length, length );
  // acceleration_arrow_->setScale( scale );

  // // Set the orientation of the arrow to match the direction of the
  // // acceleration vector.
  // acceleration_arrow_->setDirection( acc );
}

// Position and orientation are passed through to the SceneNode.
void RadarVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void RadarVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through to the Arrow object.
void RadarVisual::setColor( float r, float g, float b, float a )
{
  Ogre::ColourValue c( r, b, g, a);
  for( auto it = radar_target_shapes_.begin(); it != radar_target_shapes_.end(); ++it )
    {
      (*it)->setColor( c );
    }
}
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

