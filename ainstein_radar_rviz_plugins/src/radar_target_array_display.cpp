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

#include "radar_target_array_visual.h"
#include "radar_target_array_display.h"

namespace ainstein_radar_rviz_plugins
{

  RadarTargetArrayDisplay::RadarTargetArrayDisplay()
  {
    // Options for displaying targets:
    color_property_.reset( new rviz::ColorProperty( "Color", QColor( 255, 0, 0 ),
					       "Color to draw the target markers.",
					       this, SLOT( updateColorAndAlpha() ) ) );
    
    color_method_property_.reset( new rviz::EnumProperty( "Color Method", "Flat",
							  "Color display method.",
							  this, SLOT( updateColorAndAlpha() ) ) );    
    color_method_property_->addOptionStd( "Flat", RadarTargetArrayDisplay::COLOR_METHOD_FLAT );
    color_method_property_->addOptionStd( "Collision Time", RadarTargetArrayDisplay::COLOR_METHOD_COLLISION_TIME );
    
    alpha_property_.reset( new rviz::FloatProperty( "Alpha", 1.0,
					      "Marker opacity. 0 is fully transparent, 1 is fully opaque.",
					      this, SLOT( updateColorAndAlpha() ) ) );

    scale_property_.reset( new rviz::FloatProperty( "Scale", 0.2,
					       "Marker scale, in meters.",
					       this, SLOT( updateScale() ) ) );
  
    shape_property_.reset( new rviz::EnumProperty( "Shape", "Cube",
					      "Target shape type.",
					      this, SLOT( updateTargetShape() ) ) );    
    shape_property_->addOptionStd( "Cube", 1 );
    shape_property_->addOptionStd( "Sphere", 3 );

    // Create the history length option:
    history_length_property_.reset( new rviz::IntProperty( "Number of Scans", 1,
						      "Number of radar scans to display.",
						      this, SLOT( updateHistoryLength() )) );
    history_length_property_->setMin( 1 );
    history_length_property_->setMax( 1000000 );

    // Create the minimum target range option:
    min_range_property_.reset( new rviz::FloatProperty( "Min Range", 0.0,
						   "Minimum distance of targets to be displayed.",
						   this, SLOT( updateMinRange() )) );
    min_range_property_->setMin( 0.0 );
    min_range_property_->setMax( 100.0 );

    // Create the maximum target range option:
    max_range_property_.reset( new rviz::FloatProperty( "Max Range", 100.0,
						   "Maximum distance of targets to be displayed.",
						   this, SLOT( updateMaxRange() )) );
    max_range_property_->setMin( 0.0 );
    max_range_property_->setMax( 100.0 );

    // Determines whether to show the speed arrows:
    show_speed_property_.reset( new rviz::BoolProperty( "Show Speed", false,
						   "Toggles display of arrows indicating target speed.",
						   this, SLOT( updateShowSpeedArrows() ) ) );

    // Determines whether to show the speed arrows:
    show_info_property_.reset( new rviz::BoolProperty( "Show Info", false,
						  "Toggles display of target info text.",
						  this, SLOT( updateShowTargetInfo() ) ) );

    info_text_height_property_.reset( new rviz::FloatProperty( "Info Text Height", 0.05,
							  "Target info text height.",
							  this, SLOT( updateInfoTextHeight() ) ) );
  }

  
  RadarTargetArrayDisplay::~RadarTargetArrayDisplay()
  {
    // Clear the content of the visuals:
    for( auto& v : visuals_ )
      {
	v->clearMessage();
      }
  }

// Set up class and call superclass onInitialize().
void RadarTargetArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
  updateMinRange();
  updateMaxRange();
  updateTargetShape();
}

// Clear the visuals by deleting their objects.
void RadarTargetArrayDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Set the current color and alpha values for each visual.
void RadarTargetArrayDisplay::updateColorAndAlpha()
{
  // Set targets color and alpha:
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  int color_method = color_method_property_->getOptionInt();

  for( const auto& v : visuals_ )
    {
      v->setColor( color_method, color.r, color.g, color.b, alpha );
    }
}
  
// Set the current scale values for each visual.
void RadarTargetArrayDisplay::updateScale()
{
  // Set targets scale:
  float scale = scale_property_->getFloat();
    
  for( const auto& v : visuals_ )
    {
      v->setScale( scale );
    }
}

// Set the number of past visuals to show.
void RadarTargetArrayDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// Set the minimum range for displayed targets.
void RadarTargetArrayDisplay::updateMinRange()
{
  // Set the max range min to the min range:  
  max_range_property_->setMin( min_range_property_->getFloat() );

  // Iterate through visuals to update range limits:
  for( const auto& v : visuals_ )
    {
      v->setMinRange( min_range_property_->getFloat() );
    }   
}

// Set the maximum range for displayed targets.
void RadarTargetArrayDisplay::updateMaxRange()
{
  // Set the min range max to the max range:  
  min_range_property_->setMax( max_range_property_->getFloat() );

  // Iterate through visuals to update range limits:
  for( const auto& v : visuals_ )
    {
      v->setMaxRange( max_range_property_->getFloat() );
    }   
}

// This is our callback to handle an incoming message.
void RadarTargetArrayDisplay::processMessage( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg )
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

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full.
  boost::shared_ptr<RadarTargetArrayVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset( new RadarTargetArrayVisual( context_->getSceneManager(), scene_node_ ) );
  }

  // Now set or update the contents of the chosen visual.
  int shape;
  float alpha;
  float scale;
  Ogre::ColourValue color;
  int color_method;
  float info_text_height;

  // Set min/max range filter:
  visual->setMinRange( min_range_property_->getFloat() );
  visual->setMaxRange( max_range_property_->getFloat() );

  // Set diagnostics options:
  visual->setShowSpeedArrows( show_speed_property_->getBool() );
  visual->setShowTargetInfo( show_info_property_->getBool() );
  info_text_height = info_text_height_property_->getFloat();
  visual->setInfoTextHeight( info_text_height );

  // Set target data and visual options:
  // First set target shapes:
  shape = shape_property_->getOptionInt();
  visual->setTargetShape( shape );
  
  // Then set the target data from message:
  visual->setMessage( msg );

  // Set the target visual options:
  alpha = alpha_property_->getFloat();
  color = color_property_->getOgreColor();
  color_method = color_method_property_->getOptionInt();
  visual->setColor( color_method, color.r, color.g, color.b, alpha );
  scale = scale_property_->getFloat();
  visual->setScale( scale );
      
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  // And send it to the end of the circular buffer
  visuals_.push_back( visual );
}

  void RadarTargetArrayDisplay::updateShowSpeedArrows( void )
  {
    for( const auto& v : visuals_ )
      {
	v->setShowSpeedArrows( show_speed_property_->getBool() );
      }   
  }    

  void RadarTargetArrayDisplay::updateShowTargetInfo( void )
  {
    for( const auto& v : visuals_ )
      {
	v->setShowTargetInfo( show_info_property_->getBool() );
      }   
  }    

  void RadarTargetArrayDisplay::updateInfoTextHeight( void )
  {
    for( const auto& v : visuals_ )
      {
	v->setInfoTextHeight( info_text_height_property_->getFloat() );
      }   
  }

  void RadarTargetArrayDisplay::updateTargetShape( void )
  {
    int shape;
    for( const auto& v : visuals_ )
      {
	shape = shape_property_->getOptionInt();
	v->setTargetShape( shape );
      }       
  }
} // namespace ainstein_radar_rviz_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ainstein_radar_rviz_plugins::RadarTargetArrayDisplay, rviz::Display )
