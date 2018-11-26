#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <geometry_msgs/Vector3.h>
#include "radar_visual.h"

namespace rviz_radar_plugin
{

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

void RadarVisual::setMessageRaw( const radar_sensor_msgs::RadarData::ConstPtr& msg )
{
  double r;
  double th;
  Ogre::Vector3 pos;
  Ogre::Vector3 scale;

  // Display the raw targets:
  // Create the shape to represent the radar target:
  int n_raw_targets = msg->raw_targets.size();
  radar_target_shapes_raw_.resize( n_raw_targets );
  if( n_raw_targets > 0 )
    {
      // Create the radar target shapes:
      for( auto it = radar_target_shapes_raw_.begin(); it != radar_target_shapes_raw_.end(); ++it )
	{
	  it->reset( new rviz::Shape( rviz::Shape::Cube, scene_manager_, frame_node_ ) );
	}

      // Fill the target shapes from RadarData message:
      for( int i = 0; i < n_raw_targets; ++i )
	{
	  r = msg->raw_targets.at( i ).range;
	  th = msg->raw_targets.at( i ).azimuth;
	  pos = Ogre::Vector3(r * cos( th ),
			      r * sin( th ),
			      0.0 );
	  radar_target_shapes_raw_.at( i )->setPosition( pos );
	  
	  scale = Ogre::Vector3( 0.1, 0.1, 0.1 );
	  radar_target_shapes_raw_.at( i )->setScale( scale );
	}
    }
}

void RadarVisual::setMessageTracked( const radar_sensor_msgs::RadarData::ConstPtr& msg )
{
  double r;
  double th;
  Ogre::Vector3 pos;
  Ogre::Vector3 scale;

  // Create the shape to represent the radar target:
  int n_tracked_targets = msg->tracked_targets.size();
  radar_target_shapes_tracked_.resize( n_tracked_targets );
  if( n_tracked_targets > 0 )
    {
      // Create the radar target shapes:
      for( auto it = radar_target_shapes_tracked_.begin(); it != radar_target_shapes_tracked_.end(); ++it )
	{
	  it->reset( new rviz::Shape( rviz::Shape::Cube, scene_manager_, frame_node_ ) );
	}

      // Fill the target shapes from RadarData message:
      for( int i = 0; i < n_tracked_targets; ++i )
	{
	  r = msg->tracked_targets.at( i ).range;
	  th = msg->tracked_targets.at( i ).azimuth;
	  pos = Ogre::Vector3(r * cos( th ),
			      r * sin( th ),
			      0.0 );
	  radar_target_shapes_tracked_.at( i )->setPosition( pos );
	  
	  scale = Ogre::Vector3( 0.1, 0.1, 0.1 );
	  radar_target_shapes_tracked_.at( i )->setScale( scale );
	}
    }
}

void RadarVisual::clearMessageRaw( void )
{
  radar_target_shapes_raw_.clear();
}

void RadarVisual::clearMessageTracked( void )
{
  radar_target_shapes_tracked_.clear();
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
  
// Color is passed through to the Shape object.
void RadarVisual::setColorRaw( float r, float g, float b, float a )
{
  Ogre::ColourValue c( r, g, b, a);
    for( auto it = radar_target_shapes_raw_.begin(); it != radar_target_shapes_raw_.end(); ++it )
    {
      (*it)->setColor( c );
    }
}

// Color is passed through to the Shape object.
void RadarVisual::setColorTracked( float r, float g, float b, float a )
{
  Ogre::ColourValue c( r, g, b, a);
  for( auto it = radar_target_shapes_tracked_.begin(); it != radar_target_shapes_tracked_.end(); ++it )
    {
      (*it)->setColor( c );
    }
  
}
  
} // end namespace rviz_radar_plugin

