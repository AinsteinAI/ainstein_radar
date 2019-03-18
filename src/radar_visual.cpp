#include <sstream>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/console.h>
#include <geometry_msgs/Vector3.h>

#include "radar_visual.h"

namespace rviz_radar_plugin
{

  const int RadarVisual::max_target_visuals = 1000;
  
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

  // Set showing speed arrows to false by default:
  show_speed_arrows_ = false;

  // Set showing target info to false by default:
  show_target_info_ = false;

  // Set the target visuals vector capacities:
  radar_target_visuals_raw_.reserve( RadarVisual::max_target_visuals );
  radar_target_visuals_tracked_.reserve( RadarVisual::max_target_visuals );
}

RadarVisual::~RadarVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void RadarVisual::setMessageRaw( const radar_sensor_msgs::RadarData::ConstPtr& msg )
{
  // Resize the target shapes vector:
  radar_target_visuals_raw_.clear();

  // Fill the target shapes from RadarData message:
  for( const auto& target : msg->raw_targets )
    {
      if( target.range > min_range_ && target.range < max_range_ )
	{
	  // Create the new target shape, fill it and push back:
	  radar_target_visuals_raw_.emplace_back( scene_manager_, frame_node_ );
	  
	  // Compute the target's Cartesian position:
	  radar_target_visuals_raw_.back().pos.setPosition( Ogre::Vector3(cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation ) * target.range,
					sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation ) * target.range,
					sin( ( M_PI / 180.0 ) * target.elevation ) * target.range ) );
	  // Set the target speed arrow length:
	  if( show_speed_arrows_ )
	    {
	      radar_target_visuals_raw_.back().speed.set( std::abs( target.speed ), // shaft length
			    0.01, // shaft diameter
			    0.1, // arrow head length
			    0.05 ); // arrow head diameter
	    }
	  else
	    {
	      radar_target_visuals_raw_.back().speed.set( 0.0, 0.0, 0.0, 0.0 );
	    }

	  // Set the target speed arrow position to the target position:
	  radar_target_visuals_raw_.back().speed.setPosition( radar_target_visuals_raw_.back().pos.getPosition() );

	  // The target speed points toward the radar sensor:
	  radar_target_visuals_raw_.back().speed.setDirection( radar_target_visuals_raw_.back().pos.getPosition() /
				 std::copysign( target.range, target.speed ) );

	  // Set the info text:
	  // Set the target speed arrow length:
	  if( show_target_info_ )
	    {
	      radar_target_visuals_raw_.back().info.setLocalTranslation( radar_target_visuals_raw_.back().pos.getPosition() );
	      std::ostringstream ss;
	      ss << target;
	      radar_target_visuals_raw_.back().info.setCaption( ss.str() );
	    }
	  else
	    {
	      radar_target_visuals_raw_.back().info.setColor( Ogre::ColourValue( 0.0, 0.0, 0.0, 0.0 ) );
	    }
	}
    }
}

void RadarVisual::clearMessageRaw( void )
{
  radar_target_visuals_raw_.clear();
}

void RadarVisual::setMessageTracked( const radar_sensor_msgs::RadarData::ConstPtr& msg )
{
  // Resize the target shapes vector:
  radar_target_visuals_tracked_.clear();
  
  // Fill the target shapes from RadarData message:
  for( const auto& target : msg->tracked_targets )
    {
      if( target.range > min_range_ && target.range < max_range_ )
	{
	  // Create the new target shape, fill it and push back:	
	  radar_target_visuals_tracked_.emplace_back( scene_manager_, frame_node_ );
	  
	  // Compute the target's Cartesian position:
	  radar_target_visuals_tracked_.back().pos.setPosition( Ogre::Vector3(cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation ) * target.range,
					sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation ) * target.range,
					sin( ( M_PI / 180.0 ) * target.elevation ) * target.range ) );
	  // Set the target speed arrow length:
	  if( show_speed_arrows_ )
	    {
	      radar_target_visuals_tracked_.back().speed.set( std::abs( target.speed ), // shaft length
			    0.01, // shaft diameter
			    0.1, // arrow head length
			    0.05 ); // arrow head diameter
	    }
	  else
	    {
	      radar_target_visuals_tracked_.back().speed.set( 0.0, 0.0, 0.0, 0.0 );
	    }

	  // Set the target speed arrow position to the target position:
	  radar_target_visuals_tracked_.back().speed.setPosition( radar_target_visuals_tracked_.back().pos.getPosition() );

	  // The target speed points toward the radar sensor:
	  radar_target_visuals_tracked_.back().speed.setDirection( radar_target_visuals_tracked_.back().pos.getPosition() /
				 std::copysign( target.range, target.speed ) );

	  // Set the info text:
	  // Set the target speed arrow length:
	  if( show_target_info_ )
	    {
	      radar_target_visuals_tracked_.back().info.setLocalTranslation( radar_target_visuals_tracked_.back().pos.getPosition() );
	      std::ostringstream ss;
	      ss << target;
	      radar_target_visuals_tracked_.back().info.setCaption( ss.str() );
	    }
	  else
	    {
	      radar_target_visuals_tracked_.back().info.setColor( Ogre::ColourValue( 0.0, 0.0, 0.0, 0.0 ) );
	    }
	}
    }
}

void RadarVisual::clearMessageTracked( void )
{
  radar_target_visuals_tracked_.clear();
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
  for( auto& shape : radar_target_visuals_raw_ )
    {
      shape.pos.setColor( Ogre::ColourValue( r, g, b, a) );
      shape.speed.setColor( Ogre::ColourValue( r, g, b, a) );
    }
}

  void RadarVisual::setColorTracked( float r, float g, float b, float a )
{
  for( auto& shape : radar_target_visuals_tracked_ )
    {
      shape.pos.setColor( Ogre::ColourValue( r, g, b, a) );
      shape.speed.setColor( Ogre::ColourValue( r, g, b, a) );
    }
}

// Scale is passed through to the Shape object.
void RadarVisual::setScaleRaw( float scale )
{
  for( auto& shape : radar_target_visuals_raw_ )
    {
      shape.pos.setScale( Ogre::Vector3( scale, scale, scale ) );
    }
}

  // Scale is passed through to the Shape object.
void RadarVisual::setScaleTracked( float scale )
{
  for( auto& shape : radar_target_visuals_tracked_ )
    {
      shape.pos.setScale( Ogre::Vector3( scale, scale, scale ) );
    }
}

  void RadarVisual::updateFilteredTargets( void )
  {
    // Remove raw targets based on updated range filters:
    radar_target_visuals_raw_.erase( std::remove_if( radar_target_visuals_raw_.begin(),
						     radar_target_visuals_raw_.end(),
						     [this]( TargetVisual s )
						     {
						       Ogre::Vector3 pos = s.pos.getPosition();
						       return ( pos.length() > max_range_ ||
								pos.length() < min_range_ );
						     }), radar_target_visuals_raw_.end() );
    
    // Remove tracked targets based on updated range filters:
    radar_target_visuals_tracked_.erase( std::remove_if( radar_target_visuals_tracked_.begin(),
							 radar_target_visuals_tracked_.end(),
							 [this]( TargetVisual s )
							 {
							   Ogre::Vector3 pos = s.pos.getPosition();
							   return ( pos.length() > max_range_ ||
								    pos.length() < min_range_ );
							 }), radar_target_visuals_tracked_.end() );
  }
 
  void RadarVisual::setMinRange( float min_range )
  {
    min_range_ = min_range;
  }
    
  void RadarVisual::setMaxRange( float max_range )
  {
    max_range_ = max_range;
  }

  void RadarVisual::setShowSpeedArrows( bool show_speed_arrows )
  {
    // Store the desired state:
    show_speed_arrows_ = show_speed_arrows;

    // Update all the existing arrows if switched off:
    if( !show_speed_arrows_ )
      {
	// Easiest way to hide arrows is to scale them down to zero:
	for( auto& t : radar_target_visuals_raw_ )
	  {
	    t.speed.set( 0.0, 0.0, 0.0, 0.0 );
	  }
	for( auto& t : radar_target_visuals_tracked_ )
	  {
	    t.speed.set( 0.0, 0.0, 0.0, 0.0 );
	  }
	
      }
  }
  
  void RadarVisual::setShowTargetInfo( bool show_target_info )
  {
    // Store the desired state:
    show_target_info_ = show_target_info;

    // Update all the existing text if switched off:
    if( !show_target_info_ )
      {
	for( auto& t : radar_target_visuals_raw_ )
	  {
	    t.info.setColor( Ogre::ColourValue( 0.0, 0.0, 0.0, 0.0 ) );
	  }
	for( auto& t : radar_target_visuals_tracked_ )
	  {
	    t.info.setColor( Ogre::ColourValue( 0.0, 0.0, 0.0, 0.0 ) );
	  }       
      }
  }

  void RadarVisual::setInfoTextHeight( float info_text_height )
  {
    // Store the desired state:
    info_text_height_ = info_text_height;

    // Update all the existing text if switched off:
    for( auto& t : radar_target_visuals_raw_ )
      {
	t.info.setCharacterHeight( info_text_height_ );
      }
    for( auto& t : radar_target_visuals_tracked_ )
      {
	t.info.setCharacterHeight( info_text_height_ );
      }    
  }


} // end namespace rviz_radar_plugin

