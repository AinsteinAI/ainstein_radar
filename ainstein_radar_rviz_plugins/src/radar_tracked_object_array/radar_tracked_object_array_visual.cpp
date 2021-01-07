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

  // Fill the object position visual from the RadarTrackedObjectArray message:
  for( const auto& obj : msg->objects )
  {
    // Create the new object position visual and push it back:
    radar_tracked_object_visuals_.emplace_back( scene_manager_, frame_node_, obj_shape_type_ );

    // Define the position visual:
    radar_tracked_object_visuals_.back().pos.setPosition( 
      Ogre::Vector3( obj.pose.position.x, obj.pose.position.y, obj.pose.position.z ) );
    
    radar_tracked_object_visuals_.back().pos.setOrientation( 
      Ogre::Quaternion( obj.pose.orientation.w, 
                        obj.pose.orientation.x,
                        obj.pose.orientation.y,
                        obj.pose.orientation.z ) );

    // Define the velocity visual:
    radar_tracked_object_visuals_.back().vel.set( std::abs( obj.target.speed ), // shaft length
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
	for( auto& v : radar_tracked_object_visuals_ )
	  {
	    v.pos.setColor( Ogre::ColourValue( r, g, b, a) );
	    v.vel.setColor( Ogre::ColourValue( r, g, b, a) );
	    v.box.setColor( Ogre::ColourValue( r, g, b, a) );
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
  for( auto& v : radar_tracked_object_visuals_ )
    {
      v.pos.setScale( Ogre::Vector3( scale, scale, scale ) );
    }
}

} // namespace ainstein_radar_rviz_plugins

