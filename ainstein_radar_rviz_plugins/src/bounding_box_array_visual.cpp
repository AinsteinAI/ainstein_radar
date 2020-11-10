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

#include "bounding_box_array_visual.h"
#include "bounding_box_array_display.h"

namespace ainstein_radar_rviz_plugins
{
  const int BoundingBoxArrayVisual::max_bounding_box_visuals = 100;
  
BoundingBoxArrayVisual::BoundingBoxArrayVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;
        
  // Create a node to store the pose of the header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // Set the bounding box visuals vector capacities:
  bounding_box_visuals_.reserve( BoundingBoxArrayVisual::max_bounding_box_visuals );
}
  
BoundingBoxArrayVisual::~BoundingBoxArrayVisual()
{
  bounding_box_visuals_.clear();

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
} 

void BoundingBoxArrayVisual::setMessage( const ainstein_radar_msgs::BoundingBoxArray::ConstPtr& msg )
{
  // Resize the bounding box shapes vector:
  clearMessage();

  // Fill the bounding box shapes from the BoundingBoxArray message:
  for( const auto& box : msg->boxes )
  {
    // Create the new bounding box shape and push it back:
    bounding_box_visuals_.emplace_back( scene_manager_, frame_node_ );

    // Define the shape from the bounding box info:
    bounding_box_visuals_.back().box.setPosition( 
      Ogre::Vector3( box.pose.position.x, box.pose.position.y, box.pose.position.z ) );
 
    bounding_box_visuals_.back().box.setScale( 
      Ogre::Vector3( box.dimensions.x, box.dimensions.y, box.dimensions.z ) ); 
    
    bounding_box_visuals_.back().box.setOrientation( 
      Ogre::Quaternion( box.pose.orientation.w, 
                        box.pose.orientation.x,
                        box.pose.orientation.y,
                        box.pose.orientation.z ) );
  }

}

void BoundingBoxArrayVisual::clearMessage( void )
{
  bounding_box_visuals_.clear();
}
  
// Position is passed through to the SceneNode.
void BoundingBoxArrayVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

// Orientation is passed through to the SceneNode.
void BoundingBoxArrayVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}
  
// Color is passed through to the Shape object.
  void BoundingBoxArrayVisual::setColor( float r, float g, float b, float a )
{
  for( auto& v : bounding_box_visuals_ )
  {
    v.box.setColor( Ogre::ColourValue( r, g, b, a) );
  }
}

} // namespace ainstein_radar_rviz_plugins

