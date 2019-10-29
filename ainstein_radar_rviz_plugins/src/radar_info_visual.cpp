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
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/console.h>
#include <geometry_msgs/Vector3.h>

#include "radar_info_visual.h"
#include "radar_info_display.h"

namespace ainstein_radar_rviz_plugins
{
  
  RadarInfoVisual::RadarInfoVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) :
    fov_cone_azimuth( rviz::Shape::Cone, scene_manager, parent_node )
  {
    scene_manager_ = scene_manager;
        
    // Create a node to store the pose of the Radar's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();
  }
  
  RadarInfoVisual::~RadarInfoVisual()
  {
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void RadarInfoVisual::setMessage( const ainstein_radar_msgs::RadarInfo::ConstPtr& msg )
{
  fov_cone_azimuth.setScale( Ogre::Vector3( 0.5 * ( msg->azimuth_max - msg->azimuth_min ),
					    0.5 * ( msg->azimuth_max - msg->azimuth_min ),
					    0.5 * ( msg->azimuth_max - msg->azimuth_min ) ) );

  // Rotate the cone to the proper alignment:
  Ogre::Matrix3 rot_mat;
  rot_mat.FromEulerAnglesYXZ( Ogre::Degree( 0.0 ), Ogre::Degree( 0.0 ), Ogre::Degree( 90.0 ) );
  Ogre::Quaternion quat;
  quat.FromRotationMatrix( rot_mat );
  fov_cone_azimuth.setOrientation( quat );
}

// Position is passed through to the SceneNode.
void RadarInfoVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

// Orientation is passed through to the SceneNode.
void RadarInfoVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}
  
// Color is passed through to the Shape object.
  void RadarInfoVisual::setColor( float r, float g, float b, float a )
{
  fov_cone_azimuth.setColor( Ogre::ColourValue( r, g, b, a) );
}

} // namespace ainstein_radar_rviz_plugins

