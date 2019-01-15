#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/frame_manager.h>

#include "radar_visual.h"

#include "radar_display.h"

namespace rviz_radar_plugin
{

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
RadarDisplay::RadarDisplay()
{
  // For now, turning off "show raw" is equivalent to turning off the visualization
  // entirely. May add tracked targets back in if desired, with "show tracked".
  show_raw_property_ = new rviz::BoolProperty( "Show Raw Targets", true,
					       "Toggles display of raw target markers.",
					       this, SLOT( updateShowRaw() ) );

  color_raw_ = new rviz::ColorProperty( "Color", QColor( 255, 0, 0 ),
					"Color to draw the target markers.",
					this, SLOT( updateColorAndAlpha() ) );
  
  alpha_raw_ = new rviz::FloatProperty( "Alpha", 1.0,
					"Marker opacity. 0 is fully transparent, 1 is fully opaque.",
					this, SLOT( updateColorAndAlpha() ) );

  scale_raw_ = new rviz::FloatProperty( "Scale", 0.2,
					"Marker scale, in meters.",
					this, SLOT( updateScale() ) );

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
  min_range_property_->setMax( 20.0 );

  // Create the maximum target range option:
  max_range_property_ = new rviz::FloatProperty( "Max Range", 20.0,
						 "Maximum distance of targets to be displayed.",
						 this, SLOT( updateMaxRange() ));
  max_range_property_->setMin( 0.0 );
  max_range_property_->setMax( 20.0 );

  // Determines whether to show the speed arrows:
  show_speed_property_ = new rviz::BoolProperty( "Show Speed", false,
						 "Toggles display of arrows indicating target speed.",
						 this, SLOT( updateShowSpeedArrows() ) );
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
}

RadarDisplay::~RadarDisplay()
{
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
  if( show_raw_property_->getBool() )
    {
      alpha = alpha_raw_->getFloat();
      color = color_raw_->getOgreColor();
      for( const auto& v : visuals_ )
  	{
  	  v->setColorRaw( color.r, color.g, color.b, alpha );
  	}
    }
}
  
// Set the current scale values for each visual.
void RadarDisplay::updateScale()
{
  float scale;
  if( show_raw_property_->getBool() )
    {
      scale = scale_raw_->getFloat();
      for( const auto& v : visuals_ )
  	{
  	  v->setScaleRaw( scale );
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
      v->updateTargets();
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
      v->updateTargets();
    }   
}

// Set whether to display raw markers.
void RadarDisplay::updateShowRaw()
{
  show_raw_ = show_raw_property_->getBool();
  if( show_raw_ )
    {
      color_raw_->show();
      alpha_raw_->show();
      scale_raw_->show();
    }
  else
    {
      color_raw_->hide();
      alpha_raw_->hide();
      scale_raw_->hide();
      visuals_.clear();
    }
}
  
// This is our callback to handle an incoming message.
void RadarDisplay::processMessage( const radar_sensor_msgs::RadarData::ConstPtr& msg )
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Radar message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
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
    visual.reset(new RadarVisual( context_->getSceneManager(), scene_node_ ));
  }

  // Now set or update the contents of the chosen visual.
  float alpha;
  float scale;
  Ogre::ColourValue color;
  if( show_raw_property_->getBool() )
    {
      visual->setMinRange( min_range_property_->getFloat() );
      visual->setMaxRange( max_range_property_->getFloat() );
      visual->setShowSpeedArrows( show_speed_property_->getBool() );
      visual->setMessageRaw( msg );
      alpha = alpha_raw_->getFloat();
      color = color_raw_->getOgreColor();
      visual->setColorRaw( color.r, color.g, color.b, alpha );
      scale = scale_raw_->getFloat();
      visual->setScaleRaw( scale );
    }
  else 
    {
      visual->clearMessageRaw();
    }
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

  void RadarDisplay::updateShowSpeedArrows( void )
  {
    for( const auto& v : visuals_ )
      {
	v->setShowSpeedArrows( show_speed_property_->getBool() );
      }   
  }    
  
} // end namespace rviz_radar_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_radar_plugin::RadarDisplay,rviz::Display )
