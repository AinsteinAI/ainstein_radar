/*
  Copyright <2020> <Ainstein, Inc.>

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
#include <cstdlib>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/console.h>
#include <geometry_msgs/Vector3.h>

#include "radar_tracked_object_array_visual.h"
#include "radar_tracked_object_array_display.h"

namespace ainstein_radar_rviz_plugins
{
  const int RadarTrackedObjectArrayVisual::max_radar_tracked_object_visuals = 100;
  
RadarTrackedObjectArrayVisual::RadarTrackedObjectArrayVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;
        
  // Create a node to store the pose of the header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // Set the object visuals vector capacities:
  radar_tracked_object_visuals_.reserve( RadarTrackedObjectArrayVisual::max_radar_tracked_object_visuals );
}
  
RadarTrackedObjectArrayVisual::~RadarTrackedObjectArrayVisual()
{
  radar_tracked_object_visuals_.clear();

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
} 

void RadarTrackedObjectArrayVisual::setMessage( const ainstein_radar_msgs::RadarTrackedObjectArray::ConstPtr& msg )
{
  // Resize the object visuals vector:
  clearMessage();

  // Create a new colormap to be filled from message:
  std::map<int, Ogre::ColourValue> obj_id_colors_new;
  
  // Fill the object position visual from the RadarTrackedObjectArray message:
  for( const auto& obj : msg->objects )
  {
    // Create the new object position visual and push it back:
    radar_tracked_object_visuals_.emplace_back( scene_manager_, frame_node_, obj_shape_type_ );

    // Populate the object ID colormap from an empty map each scan:
    auto obj_id_iter = obj_id_colors_.find( obj.id );
    if( obj_id_iter != obj_id_colors_.end() ) // object ID already has a color value associated with it
      {
	// Copy the color value to the new map:
	obj_id_colors_new[obj.id] = obj_id_iter->second; 
      }
    else // add a new entry to map with an "empty" color value
      {
	obj_id_colors_new[obj.id] = Ogre::ColourValue( 0.0f, 0.0f, 0.0f, 0.0f );
      }

    // Set the object ID in the visual:
    radar_tracked_object_visuals_.back().id = obj.id;
    
    // Define the position visual:
    radar_tracked_object_visuals_.back().pos.setPosition( 
      Ogre::Vector3( obj.pose.position.x, obj.pose.position.y, obj.pose.position.z ) );
    
    radar_tracked_object_visuals_.back().pos.setOrientation( 
      Ogre::Quaternion( obj.pose.orientation.w, 
                        obj.pose.orientation.x,
                        obj.pose.orientation.y,
                        obj.pose.orientation.z ) );

    radar_tracked_object_visuals_.back().pos.setScale( Ogre::Vector3( scale_, scale_, scale_ ) );
    
    // If display alert is true and the object is within the specified area,
    // then scale it for visual effect:
    if( display_alert_ )
    {
      // Convert from Cartesian to spherical coordinates:
      double range = std::sqrt( std::pow( obj.pose.position.x, 2.0 ) +
			 std::pow( obj.pose.position.y, 2.0 ) +
			 std::pow( obj.pose.position.z, 2.0 ) );
      double azimuth = std::atan2( obj.pose.position.y, obj.pose.position.x );
      double elevation = std::asin( obj.pose.position.z / range );
 
      if( range <= alert_range_max_ )
      {
        radar_tracked_object_visuals_.back().pos.setScale( Ogre::Vector3( alert_scale_, alert_scale_, alert_scale_ ) );
      }
    }

    // Define the velocity visual:
    double speed = std::sqrt( std::pow( obj.velocity.linear.x, 2.0 ) +
			      std::pow( obj.velocity.linear.y, 2.0 ) +
			      std::pow( obj.velocity.linear.z, 2.0 ) );
 
    radar_tracked_object_visuals_.back().vel.set( speed, // shaft length
						  0.05, // shaft diameter
						  0.1, // arrow head length
						  0.1 ); // arrow head diameter
    
    radar_tracked_object_visuals_.back().vel.setPosition( 
      Ogre::Vector3( obj.pose.position.x, obj.pose.position.y, obj.pose.position.z ) );
    
    radar_tracked_object_visuals_.back().vel.setDirection(
      Ogre::Vector3( obj.velocity.linear.x, obj.velocity.linear.y, obj.velocity.linear.z ) );
    
    // Define the bounding box visual:
    radar_tracked_object_visuals_.back().box.setPosition( 
      Ogre::Vector3( obj.box.pose.position.x, obj.box.pose.position.y, obj.box.pose.position.z ) );
 
    radar_tracked_object_visuals_.back().box.setScale( 
      Ogre::Vector3( obj.box.dimensions.x, obj.box.dimensions.y, obj.box.dimensions.z ) ); 
    
    radar_tracked_object_visuals_.back().box.setOrientation( 
      Ogre::Quaternion( obj.box.pose.orientation.w, 
                        obj.box.pose.orientation.x,
                        obj.box.pose.orientation.y,
                        obj.box.pose.orientation.z ) );
  }
  
  // Copy new colormap:
  obj_id_colors_ = obj_id_colors_new;
}

void RadarTrackedObjectArrayVisual::clearMessage( void )
{
  radar_tracked_object_visuals_.clear();
}
  
// Position is passed through to the SceneNode.
void RadarTrackedObjectArrayVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

// Orientation is passed through to the SceneNode.
void RadarTrackedObjectArrayVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}
  
// Color is passed through to the Shape object.
void RadarTrackedObjectArrayVisual::setColor( int color_method, float r, float g, float b, float a )
{
  switch( color_method )
    {
    case RadarTrackedObjectArrayDisplay::COLOR_METHOD_FLAT: // Flat coloring
      {
	for( auto& v : radar_tracked_object_visuals_ )
	  {
	    v.pos.setColor( Ogre::ColourValue( r, g, b, a) );
	    v.vel.setColor( Ogre::ColourValue( r, g, b, a) );
	    v.box.setColor( Ogre::ColourValue( r, g, b, a) );
	  }
      }
      break;

    case RadarTrackedObjectArrayDisplay::COLOR_METHOD_OBJECT_ID:
      {
	// Starting random value for hue and golden ratio inverse:
	float hue_rand = rand() / float( RAND_MAX );
	float golden_ratio_inv = 0.618033988749895;
	Ogre::ColourValue hsb_color;
	
	for( auto& v : radar_tracked_object_visuals_ )
	  {
	    // Check if the object already has a valid color value:
	    auto obj_id_iter = obj_id_colors_.find( v.id );
	    
	    if( obj_id_iter->second != Ogre::ColourValue( 0.0f, 0.0f, 0.0f, 0.0f ) ) // valid
	      {
		v.pos.setColor( obj_id_iter->second );
		v.vel.setColor( obj_id_iter->second );
		v.box.setColor( obj_id_iter->second );		
	      }
	    else // invalid color, set colormap entry
	      {
		hue_rand += golden_ratio_inv;
		hue_rand = fmod( hue_rand, 1.0f );
		hsb_color.setHSB( hue_rand, 0.99, 0.99 );
		v.pos.setColor( hsb_color );
		v.vel.setColor( hsb_color );
		v.box.setColor( hsb_color );

		obj_id_colors_[v.id] = hsb_color;
	      }
	  }
      }
      break;

    default:
      ROS_ERROR_STREAM( "Invalid color method passed to setColor." );
      break;
    }
}
  
// Scale is passed through to the Shape object.
void RadarTrackedObjectArrayVisual::setScale( float scale )
{
  scale_ = scale;
  for( auto& v : radar_tracked_object_visuals_ )
    {
      v.pos.setScale( Ogre::Vector3( scale_, scale_, scale_ ) );
    }
}

void RadarTrackedObjectArrayVisual::setDisplayAlert( bool display_alert )
{
  display_alert_ = display_alert;
}

void RadarTrackedObjectArrayVisual::setAlertScale( float alert_scale )
{
  alert_scale_ = alert_scale;

  if( display_alert_ )
  {
    for( auto& v : radar_tracked_object_visuals_ )
      {
        v.pos.setScale( Ogre::Vector3( alert_scale_, alert_scale_, alert_scale_ ) );
      }
  }
}

void RadarTrackedObjectArrayVisual::setAlertRangeMax( float alert_range_max )
{
  alert_range_max_ = alert_range_max;
}

} // namespace ainstein_radar_rviz_plugins

