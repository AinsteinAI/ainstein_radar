/*
  Copyright <2018-2019> <Ainstein, Inc.>

  Redistribution and use in source and binary forms, with or without modification, are permitted 
  provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this list of 
  conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this list of 
  conditions and the following disclaimer in the documentation and/or other materials provided 
  with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may be used to 
  endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <sstream>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/console.h>
#include <geometry_msgs/Vector3.h>

#include "radar_visual.h"

namespace ainstein_radar_rviz_plugins
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
  radar_target_visuals_.reserve( RadarVisual::max_target_visuals );
}

RadarVisual::~RadarVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void RadarVisual::setMessage( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg )
{
  // Resize the target shapes vector:
  radar_target_visuals_.clear();

  // Fill the target shapes from RadarTargetArray message:
  for( const auto& target : msg->targets )
    {
      if( target.range > min_range_ && target.range < max_range_ )
	{
	  // Create the new target shape, fill it and push back:
	  radar_target_visuals_.emplace_back( scene_manager_, frame_node_, shape_type_ );
	  
	  // Compute the target's Cartesian position:
	  radar_target_visuals_.back().pos.setPosition( Ogre::Vector3(cos( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation ) * target.range,
	  							      sin( ( M_PI / 180.0 ) * target.azimuth ) * cos( ( M_PI / 180.0 ) * target.elevation ) * target.range,
	  							      sin( ( M_PI / 180.0 ) * target.elevation ) * target.range ) );

	  // Set the target speed arrow length:
	  if( show_speed_arrows_ )
	    {
	      radar_target_visuals_.back().speed.set( std::abs( target.speed ), // shaft length
	  					      0.1, // shaft diameter
	  					      0.2, // arrow head length
	  					      0.2 ); // arrow head diameter
	    }
	  else
	    {
	      radar_target_visuals_.back().speed.set( 0.0, 0.0, 0.0, 0.0 );
	    }

	  // Set the target speed arrow position to the target position:
	  radar_target_visuals_.back().speed.setPosition( radar_target_visuals_.back().pos.getPosition() );
	  
	  // The target speed points toward the radar sensor:
	  radar_target_visuals_.back().speed.setDirection( radar_target_visuals_.back().pos.getPosition() /
	  						   std::copysign( target.range, target.speed ) );

	  // Set the info text:
	  // Set the target speed arrow length:
	  if( show_target_info_ )
	    {
	      radar_target_visuals_.back().info.setLocalTranslation( radar_target_visuals_.back().pos.getPosition() );
	      std::ostringstream ss;
	      ss << target;
	      radar_target_visuals_.back().info.setCaption( ss.str() );
	    }
	  else
	    {
	      radar_target_visuals_.back().info.setColor( Ogre::ColourValue( 0.0, 0.0, 0.0, 0.0 ) );
	    }
	}
    }
}

void RadarVisual::clearMessage( void )
{
  radar_target_visuals_.clear();
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
void RadarVisual::setColor( float r, float g, float b, float a )
{
  for( auto& shape : radar_target_visuals_ )
    {
      shape.pos.setColor( Ogre::ColourValue( r, g, b, a) );
      shape.speed.setColor( Ogre::ColourValue( r, g, b, a) );
    }
}

// Scale is passed through to the Shape object.
void RadarVisual::setScale( float scale )
{
  for( auto& shape : radar_target_visuals_ )
    {
      shape.pos.setScale( Ogre::Vector3( scale, scale, scale ) );
    }
}
  
  void RadarVisual::updateFilteredTargets( void )
  {
    // // Remove raw targets based on updated range filters:
    // radar_target_visuals_raw_.erase( std::remove_if( radar_target_visuals_raw_.begin(),
    // 						     radar_target_visuals_raw_.end(),
    // 						     [this]( TargetVisual s )
    // 						     {
    // 						       Ogre::Vector3 pos = s.pos.getPosition();
    // 						       return ( pos.length() > max_range_ ||
    // 								pos.length() < min_range_ );
    // 						     }), radar_target_visuals_raw_.end() );
    
    // // Remove tracked targets based on updated range filters:
    // radar_target_visuals_tracked_.erase( std::remove_if( radar_target_visuals_tracked_.begin(),
    // 							 radar_target_visuals_tracked_.end(),
    // 							 [this]( TargetVisual s )
    // 							 {
    // 							   Ogre::Vector3 pos = s.pos.getPosition();
    // 							   return ( pos.length() > max_range_ ||
    // 								    pos.length() < min_range_ );
    // 							 }), radar_target_visuals_tracked_.end() );
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
	for( auto& t : radar_target_visuals_ )
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
	for( auto& t : radar_target_visuals_ )
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
    for( auto& t : radar_target_visuals_ )
      {
	t.info.setCharacterHeight( info_text_height_ );
      }
  }


} // namespace ainstein_radar_rviz_plugins

