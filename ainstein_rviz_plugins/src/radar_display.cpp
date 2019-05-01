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

#include "radar_visual.h"
#include "radar_display.h"

namespace ainstein_rviz_plugins
{

  // The constructor must have no arguments, so we can't give the
  // constructor the parameters it needs to fully initialize.
  RadarDisplay::RadarDisplay()
  {
    // Options for displaying targets:
    show_targets_property_ = new rviz::BoolProperty( "Show Targets", true,
						     "Toggles display of target markers.",
						     this, SLOT( updateShowTargets() ) );

    list_targets_ = new rviz::Property( "Targets", QVariant(),
					"Target display options.",
					this, 0 );

    list_targets_->addChild( new rviz::ColorProperty( "Color", QColor( 255, 0, 0 ),
						  "Color to draw the target markers.",
						  this, SLOT( updateColorAndAlpha() ) ) );
  
    list_targets_->addChild( new rviz::FloatProperty( "Alpha", 1.0,
						      "Marker opacity. 0 is fully transparent, 1 is fully opaque.",
						      this, SLOT( updateColorAndAlpha() ) ) );

    list_targets_->addChild( new rviz::FloatProperty( "Scale", 0.2,
						      "Marker scale, in meters.",
						      this, SLOT( updateScale() ) ) );
    
    list_targets_->addChild( new rviz::EnumProperty( "Shape", "Cube",
						     "Target shape type.",
						     this, SLOT( updateTargetShape() ) ) );
    
    rviz::EnumProperty* shape = static_cast<rviz::EnumProperty*>( list_targets_->subProp( "Shape" ) );
    shape->addOptionStd( "Cube", 1 );
    shape->addOptionStd( "Sphere", 3 );

    // Create the history length option:
    history_length_property_ = new rviz::IntProperty( "Number of Scans", 1,
						      "Number of radar scans to display.",
						      this, SLOT( updateHistoryLength() ));
    history_length_property_->setMin( 1 );
    history_length_property_->setMax( 1000000 );

    // Create the minimum target range option:
    min_range_property_ = new rviz::FloatProperty( "Min Range", 0.0,
						   "Minimum distance of targets to be displayed.",
						   this, SLOT( updateMinRange() ));
    min_range_property_->setMin( 0.0 );
    min_range_property_->setMax( 100.0 );

    // Create the maximum target range option:
    max_range_property_ = new rviz::FloatProperty( "Max Range", 100.0,
						   "Maximum distance of targets to be displayed.",
						   this, SLOT( updateMaxRange() ));
    max_range_property_->setMin( 0.0 );
    max_range_property_->setMax( 100.0 );

    // Determines whether to show the speed arrows:
    show_speed_property_ = new rviz::BoolProperty( "Show Speed", false,
						   "Toggles display of arrows indicating target speed.",
						   this, SLOT( updateShowSpeedArrows() ) );

    // Determines whether to show the speed arrows:
    show_info_property_ = new rviz::BoolProperty( "Show Info", false,
						  "Toggles display of target info text.",
						  this, SLOT( updateShowTargetInfo() ) );

    info_text_height_property_ = new rviz::FloatProperty( "Info Text Height", 0.05,
							  "Target info text height.",
							  this, SLOT( updateInfoTextHeight() ) );
  }

  
  RadarDisplay::~RadarDisplay()
  {
  }

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void RadarDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
  updateMinRange();
  updateMaxRange();
  updateTargetShape();
}

// Clear the visuals by deleting their objects.
void RadarDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Set the current color and alpha values for each visual.
void RadarDisplay::updateColorAndAlpha()
{
  float alpha;
  Ogre::ColourValue color;

  // Set targets color and alpha:
  if( show_targets_property_->getBool() )
    {
      alpha = static_cast<rviz::FloatProperty*>( list_targets_->subProp( "Alpha" ) )->getFloat();
      color = static_cast<rviz::ColorProperty*>( list_targets_->subProp( "Color" ) )->getOgreColor();      for( const auto& v : visuals_ )
  	{
  	  v->setColor( color.r, color.g, color.b, alpha );
  	}
    }
}
  
// Set the current scale values for each visual.
void RadarDisplay::updateScale()
{
  float scale;

  // Set targets scale:
  if( show_targets_property_->getBool() )
    {
      scale = static_cast<rviz::FloatProperty*>( list_targets_->subProp( "Scale" ) )->getFloat();
      for( const auto& v : visuals_ )
  	{
  	  v->setScale( scale );
  	}
    }
}

// Set the number of past visuals to show.
void RadarDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// Set the minimum range for displayed targets.
void RadarDisplay::updateMinRange()
{
  // Set the max range min to the min range:  
  max_range_property_->setMin( min_range_property_->getFloat() );

  // Iterate through visuals to update range limits:
  for( const auto& v : visuals_ )
    {
      v->setMinRange( min_range_property_->getFloat() );
      v->updateFilteredTargets();
    }   
}

// Set the maximum range for displayed targets.
void RadarDisplay::updateMaxRange()
{
  // Set the min range max to the max range:  
  min_range_property_->setMax( max_range_property_->getFloat() );

  // Iterate through visuals to update range limits:
  for( const auto& v : visuals_ )
    {
      v->setMaxRange( max_range_property_->getFloat() );
      v->updateFilteredTargets();
    }   
}

// Set whether to display targets.
void RadarDisplay::updateShowTargets()
{
  // Get the visual options settings:
  show_targets_ = show_targets_property_->getBool();

  // Visual options:
  if( show_targets_ )
    {
      list_targets_->show();
      list_targets_->expand();
    }
  else
    {
      list_targets_->hide();
      visuals_.clear();
    }

  // General visual options:
  if( show_targets_ )
    {
      history_length_property_->show();
      min_range_property_->show();
      max_range_property_->show();
      show_speed_property_->show();
      show_info_property_->show();
      info_text_height_property_->show();
    }
  else
    {
      history_length_property_->hide();
      min_range_property_->hide();
      max_range_property_->hide();
      show_speed_property_->hide();
      show_info_property_->hide();
      info_text_height_property_->hide();
    }      
}

// This is our callback to handle an incoming message.
void RadarDisplay::processMessage( const ainstein_radar_msgs::RadarTargetArray::ConstPtr& msg )
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
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<RadarVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset( new RadarVisual( context_->getSceneManager(), scene_node_ ) );
  }

  // Now set or update the contents of the chosen visual.
  int shape;
  float alpha;
  float scale;
  Ogre::ColourValue color;
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
  if( show_targets_property_->getBool() )
    {
      // First set target shapes:
      shape = static_cast<rviz::EnumProperty*>( list_targets_->subProp( "Shape" ) )->getOptionInt();
      visual->setTargetShape( shape );

      // Then set the target data from message:
      visual->setMessage( msg );

      // Set the target visual options:
      alpha = static_cast<rviz::FloatProperty*>( list_targets_->subProp( "Alpha" ) )->getFloat();
      color = static_cast<rviz::ColorProperty*>( list_targets_->subProp( "Color" ) )->getOgreColor();
      visual->setColor( color.r, color.g, color.b, alpha );
      scale = static_cast<rviz::FloatProperty*>( list_targets_->subProp( "Scale" ) )->getFloat();
      visual->setScale( scale );
    }
  else 
    {
      visual->clearMessage();
    }
  
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  // And send it to the end of the circular buffer
  visuals_.push_back( visual );
}

  void RadarDisplay::updateShowSpeedArrows( void )
  {
    for( const auto& v : visuals_ )
      {
	v->setShowSpeedArrows( show_speed_property_->getBool() );
      }   
  }    

  void RadarDisplay::updateShowTargetInfo( void )
  {
    for( const auto& v : visuals_ )
      {
	v->setShowTargetInfo( show_info_property_->getBool() );
      }   
  }    

  void RadarDisplay::updateInfoTextHeight( void )
  {
    for( const auto& v : visuals_ )
      {
	v->setInfoTextHeight( info_text_height_property_->getFloat() );
      }   
  }

  void RadarDisplay::updateTargetShape( void )
  {
    int shape;
    for( const auto& v : visuals_ )
      {
	shape = static_cast<rviz::EnumProperty*>( list_targets_->subProp( "Shape" ) )->getOptionInt();
	v->setTargetShape( shape );
      }       
  }
} // namespace ainstein_rviz_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ainstein_rviz_plugins::RadarDisplay,rviz::Display )
