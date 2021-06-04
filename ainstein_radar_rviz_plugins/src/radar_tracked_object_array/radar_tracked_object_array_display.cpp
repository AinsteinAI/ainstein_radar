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

#include "radar_tracked_object_array_visual.h"
#include "radar_tracked_object_array_display.h"

namespace ainstein_radar_rviz_plugins
{

  RadarTrackedObjectArrayDisplay::RadarTrackedObjectArrayDisplay()
  {
    // Options for displaying visual:
    color_property_.reset( new rviz::ColorProperty( "Color", QColor( 255, 0, 0 ),
						    "Color to draw the object.",
						    this, SLOT( updateColorAndAlpha() ) ) );

    color_method_property_.reset( new rviz::EnumProperty( "Color Method", "Flat",
							  "Color display method.",
							  this, SLOT( updateColorAndAlpha() ) ) );    
    color_method_property_->addOptionStd( "Flat", RadarTrackedObjectArrayDisplay::COLOR_METHOD_FLAT );
    color_method_property_->addOptionStd( "Object ID", RadarTrackedObjectArrayDisplay::COLOR_METHOD_OBJECT_ID );

    // Determines whether to alert the user when objects cross within the defined area:
    display_alert_property_.reset( new rviz::BoolProperty( "Display Alert", false,
							   "Toggles display of object alert for defined area.",
							   this, SLOT( updateDisplayAlert() ) ) );

    display_alert_options_property_.reset( new rviz::Property( "Alert Options", QVariant(),
				    "Display alert options.",
				    this, 0 ) );

    display_alert_options_property_->addChild( new rviz::FloatProperty( "Alert Scale", 1.0,
						  "Object scale to be used when alert is active.",
						  this, SLOT( updateAlertScale() ) ) );

    display_alert_options_property_->addChild( new rviz::FloatProperty( "Max Range", 2.0,
						  "Maximum range of alert area.",
						  this, SLOT( updateAlertRangeMax() ) ) );

    display_alert_options_property_->hide();
        
    alpha_property_.reset( new rviz::FloatProperty( "Alpha", 1.0,
						    "Object opacity. 0 is fully transparent, 1 is fully opaque.",
						    this, SLOT( updateColorAndAlpha() ) ) );    

    scale_property_.reset( new rviz::FloatProperty( "Scale", 0.2,
						    "Object position scale, in meters.",
						    this, SLOT( updateScale() ) ) );
  
    shape_property_.reset( new rviz::EnumProperty( "Shape", "Cube",
						   "Object position shape type.",
						   this, SLOT( updateObjectShape() ) ) );    
    shape_property_->addOptionStd( "Cube", 1 );
    shape_property_->addOptionStd( "Sphere", 3 );
  }

  
  RadarTrackedObjectArrayDisplay::~RadarTrackedObjectArrayDisplay()
  {
    visual_->clearMessage();
  }

// Set up class and call superclass onInitialize().
void RadarTrackedObjectArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();

  // Create the RadarTrackedObjectArray visual:
  visual_.reset( new RadarTrackedObjectArrayVisual( context_->getSceneManager(), scene_node_ ) );

  // Initialize display options:
  updateObjectShape();
  updateScale();
  updateDisplayAlert();
  updateAlertScale();
  updateAlertRangeMax();
}

// Clear the visuals by deleting their objects.
void RadarTrackedObjectArrayDisplay::reset()
{
  MFDClass::reset();
}

  // Show options specific to display alert when enabled.
void RadarTrackedObjectArrayDisplay::updateDisplayAlert()
{
  visual_->setDisplayAlert( display_alert_property_->getBool() );

  if( display_alert_property_->getBool() )
  {
    display_alert_options_property_->show();
    display_alert_options_property_->expand();
  }
  else
  {
    display_alert_options_property_->hide();
  }
}

// Set the alert scale value for each visual.
void RadarTrackedObjectArrayDisplay::updateAlertScale()
{
  // Set targets scale:
  float scale = static_cast<rviz::FloatProperty*>( display_alert_options_property_->subProp( "Alert Scale" ) )->getFloat();
  
  visual_->setAlertScale( scale );
}

// Set the alert area maximum range.
void RadarTrackedObjectArrayDisplay::updateAlertRangeMax()
{
  // Set targets scale:
  float range_max = static_cast<rviz::FloatProperty*>( display_alert_options_property_->subProp( "Max Range" ) )->getFloat();
  
  visual_->setAlertRangeMax( range_max );
}

// Set the current color and alpha values for each visual.
void RadarTrackedObjectArrayDisplay::updateColorAndAlpha()
{
  // Set targets color and alpha:
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  int color_method = color_method_property_->getOptionInt();

  visual_->setColor( color_method, color.r, color.g, color.b, alpha );
}
    
// Set the current scale values for each visual.
void RadarTrackedObjectArrayDisplay::updateScale()
{
  // Set targets scale:
  float scale = scale_property_->getFloat();
    
  visual_->setScale( scale );
}

void RadarTrackedObjectArrayDisplay::updateObjectShape( void )
{
  int shape = shape_property_->getOptionInt();
  
  visual_->setObjectShape( shape );
}       

// This is our callback to handle an incoming message.
void RadarTrackedObjectArrayDisplay::processMessage( const ainstein_radar_msgs::RadarTrackedObjectArray::ConstPtr& msg )
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this message.  If
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
  
  // Now set or update the contents of the chosen visual.
  float alpha;
  Ogre::ColourValue color;
  int color_method;
  float scale;
  int shape;

  // Set the visual from message:
  visual_->setMessage( msg );

  // Set the visual options:
  alpha = alpha_property_->getFloat();
  color = color_property_->getOgreColor();
  color_method = color_method_property_->getOptionInt();
  visual_->setColor( color_method, color.r, color.g, color.b, alpha );

  shape = shape_property_->getOptionInt();
  visual_->setObjectShape( shape );
      
  visual_->setFramePosition( position );
  visual_->setFrameOrientation( orientation );
}

} // namespace ainstein_radar_rviz_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ainstein_radar_rviz_plugins::RadarTrackedObjectArrayDisplay, rviz::Display )
