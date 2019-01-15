#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/console.h>
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
  radar_target_shapes_raw_.clear();

  if( n_raw_targets > 0 )
    {
      // Fill the target shapes from RadarData message:
      for( int i = 0; i < n_raw_targets; ++i )
	{
	  boost::shared_ptr<rviz::Shape> s( new rviz::Shape( rviz::Shape::Cube, scene_manager_, frame_node_ ) );

	  r = msg->raw_targets.at( i ).range;
	  th = msg->raw_targets.at( i ).azimuth;
	  pos = Ogre::Vector3(r * cos( ( M_PI / 180.0 ) * th ),
			      r * sin( ( M_PI / 180.0 ) * th ),
			      0.0 );
	  s->setPosition( pos );

	  if( r > min_range_ && r < max_range_ )
	    {
	      radar_target_shapes_raw_.push_back( s );
	    }
	  else
	    {
	      s.reset(); // explicitly delete new shape
	    }
	}
    }
}

void RadarVisual::setMessageTracked( const radar_sensor_msgs::RadarData::ConstPtr& msg )
{
  double r;
  double th;
  Ogre::Vector3 pos;
  Ogre::Vector3 scale;

  // Display the tracked targets:
  // Create the shape to represent the radar target:
  int n_tracked_targets = msg->tracked_targets.size();
  radar_target_shapes_tracked_.clear();

  if( n_tracked_targets > 0 )
    {
      // Fill the target shapes from RadarData message:
      for( int i = 0; i < n_tracked_targets; ++i )
	{
	  boost::shared_ptr<rviz::Shape> s( new rviz::Shape( rviz::Shape::Cube, scene_manager_, frame_node_ ) );

	  r = msg->tracked_targets.at( i ).range;
	  th = msg->tracked_targets.at( i ).azimuth;
	  pos = Ogre::Vector3(r * cos( ( M_PI / 180.0 ) * th ),
			      r * sin( ( M_PI / 180.0 ) * th ),
			      0.0 );
	  s->setPosition( pos );

	  if( r > min_range_ )
	    {
	      radar_target_shapes_tracked_.push_back( s );
	    }
	  else
	    {
	      s.reset(); // explicitly delete new shape
	    }
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

  
// Scale is passed through to the Shape object.
void RadarVisual::setScaleRaw( float scale )
{
  Ogre::Vector3 s( scale, scale, scale );
  for( auto it = radar_target_shapes_raw_.begin(); it != radar_target_shapes_raw_.end(); ++it )
    {
      (*it)->setScale( s );
    }
}

// Scale is passed through to the Shape object.
void RadarVisual::setScaleTracked( float scale )
{
  Ogre::Vector3 s( scale, scale, scale );
  for( auto it = radar_target_shapes_tracked_.begin(); it != radar_target_shapes_tracked_.end(); ++it )
    {
      (*it)->setScale( s );
    }
}

  void RadarVisual::updateTargets( void )
  {
    radar_target_shapes_raw_.erase( std::remove_if( radar_target_shapes_raw_.begin(),
						    radar_target_shapes_raw_.end(),
						    [this]( boost::shared_ptr<rviz::Shape> s )
						    {
						      Ogre::Vector3 pos = s->getPosition();
						      return ( pos.length() > max_range_ ||
							       pos.length() < min_range_ );
						    }), radar_target_shapes_raw_.end() );

  }
 
  void RadarVisual::setMinRange( float min_range )
  {
    min_range_ = min_range;
  }
    
  void RadarVisual::setMaxRange( float max_range )
  {
    max_range_ = max_range;
  }
 
} // end namespace rviz_radar_plugin

