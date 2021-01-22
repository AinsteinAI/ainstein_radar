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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>

#include "radar_info_visual.h"
#include "radar_info_display.h"

namespace ainstein_radar_rviz_plugins
{

  RadarInfoDisplay::RadarInfoDisplay()
  {
    // Options for displaying visual:
    color_property_.reset( new rviz::ColorProperty( "Color", QColor( 255, 0, 0 ),
						    "Color to draw the FOV cone.",
						    this, SLOT( updateColorAndAlpha() ) ) );
    
    alpha_property_.reset( new rviz::FloatProperty( "Alpha", 1.0,
						    "FOV cone opacity. 0 is fully transparent, 1 is fully opaque.",
						    this, SLOT( updateColorAndAlpha() ) ) );    
  }

  
  RadarInfoDisplay::~RadarInfoDisplay()
  {
  }

// Set up class and call superclass onInitialize().
void RadarInfoDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

// Clear the visuals by deleting their objects.
void RadarInfoDisplay::reset()
{
  MFDClass::reset();
}

// Set the current color and alpha values for each visual.
void RadarInfoDisplay::updateColorAndAlpha()
{
  // Set visual color and alpha:
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  visual_->setColor( color.r, color.g, color.b, alpha );
}

// This is our callback to handle an incoming message.
void RadarInfoDisplay::processMessage( const ainstein_radar_msgs::RadarInfo::ConstPtr& msg )
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Radar message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ) )
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ) );
    return;
  }

  // Create the RadarInfo visual:
  visual_.reset( new RadarInfoVisual( context_->getSceneManager(), scene_node_ ) );
  
  // Now set or update the contents of the chosen visual.
  float alpha;
  Ogre::ColourValue color;

  // Set the visual from message:
  visual_->setMessage( msg );

  // Set the visual options:
  alpha = alpha_property_->getFloat();
  color = color_property_->getOgreColor();
  visual_->setColor( color.r, color.g, color.b, alpha );
      
  visual_->setFramePosition( position );
  visual_->setFrameOrientation( orientation );
}

} // namespace ainstein_radar_rviz_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ainstein_radar_rviz_plugins::RadarInfoDisplay, rviz::Display )
