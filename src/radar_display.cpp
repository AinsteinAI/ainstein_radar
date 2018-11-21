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

  // Create the dropdown list of raw target options:
  show_raw_property_ = new rviz::BoolProperty( "Show Raw Targets", true,
					       "Toggles display of raw target markers.",
					       this, SLOT( updateShowRaw() ) );

  list_raw_ = new rviz::Property( "Raw Targets", QVariant(),
				  "Raw target display options.",
				  this, 0 );

  list_raw_->addChild( new rviz::ColorProperty( "Color", QColor( 255, 0, 0 ),
						"Color to draw the target markers.",
						this, SLOT( updateColorAndAlpha() ) ) );

  list_raw_->addChild( new rviz::FloatProperty( "Alpha", 1.0,
						"Marker opacity. 0 is fully transparent, 1 is fully opaque.",
						this, SLOT( updateColorAndAlpha() ) ) );

  list_raw_->addChild( new rviz::FloatProperty( "Scale", 0.1,
						"Marker scale, in meters.",
						this, SLOT( updateScale() ) ) );

  // Create the dropdown list of tracked target options:
  show_tracked_property_ = new rviz::BoolProperty( "Show Tracked Targets", true,
						   "Toggles display of tracked target markers.",
						   this, SLOT( updateShowTracked() ) );

  list_tracked_ = new rviz::Property( "Tracked Targets", QVariant(),
				      "Tracked target display options.",
				      this, 0 );

  list_tracked_->addChild( new rviz::ColorProperty( "Color", QColor( 255, 0, 0 ),
						    "Color to draw the target markers.",
						    this, SLOT( updateColorAndAlpha() ) ) );

  list_tracked_->addChild( new rviz::FloatProperty( "Alpha", 1.0,
						    "Marker opacity. 0 is fully transparent, 1 is fully opaque.",
						    this, SLOT( updateColorAndAlpha() ) ) );

  list_tracked_->addChild( new rviz::FloatProperty( "Scale", 0.1,
						    "Marker scale, in meters.",
						    this, SLOT( updateScale() ) ) );

  // Create the history length option:
  history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT( updateHistoryLength() ));
  history_length_property_->setMin( 1 );
  history_length_property_->setMax( 100000 );
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
      alpha = static_cast<rviz::FloatProperty*>( list_raw_->subProp( "Alpha" ) )->getFloat();
      color = static_cast<rviz::ColorProperty*>( list_raw_->subProp( "Color" ) )->getOgreColor();
      for( size_t i = 0; i < visuals_.size(); i++ )
	{
	  visuals_[ i ]->setColorRaw( color.r, color.g, color.b, alpha );
	}
    }

    if( show_tracked_property_->getBool() )
    {
      alpha = static_cast<rviz::FloatProperty*>( list_tracked_->subProp( "Alpha" ) )->getFloat();
      color = static_cast<rviz::ColorProperty*>( list_tracked_->subProp( "Color" ) )->getOgreColor();
      for( size_t i = 0; i < visuals_.size(); i++ )
	{
	  visuals_[ i ]->setColorTracked( color.r, color.g, color.b, alpha );
	}
    }
}
  
// Set the current scale values for each visual.
void RadarDisplay::updateScale()
{
  float scale;
  if( show_raw_property_->getBool() )
    {
      scale = static_cast<rviz::FloatProperty*>( list_raw_->subProp( "Scale" ) )->getFloat();
      for( size_t i = 0; i < visuals_.size(); i++ )
	{
	  visuals_[ i ]->setScaleRaw( scale );
	}
    }

  if( show_tracked_property_->getBool() )
    {
      scale = static_cast<rviz::FloatProperty*>( list_tracked_->subProp( "Scale" ) )->getFloat();
      for( size_t i = 0; i < visuals_.size(); i++ )
	{
	  visuals_[ i ]->setScaleTracked( scale );
	}
    }
}

// Set the number of past visuals to show.
void RadarDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// Set whether to display raw markers.
void RadarDisplay::updateShowRaw()
{
  show_raw_ = show_raw_property_->getBool();
  if( show_raw_ )
    {
      list_raw_->show();
    }
  else
    {
      list_raw_->hide();
    }
}

// Set whether to display tracked markers.
void RadarDisplay::updateShowTracked()
{
  show_tracked_ = show_tracked_property_->getBool();
  if( show_tracked_ )
    {
      list_tracked_->show();
    }
  else
    {
      list_tracked_->hide();
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
      visual->setMessageRaw( msg );
      alpha = static_cast<rviz::FloatProperty*>( list_raw_->subProp( "Alpha" ) )->getFloat();
      color = static_cast<rviz::ColorProperty*>( list_raw_->subProp( "Color" ) )->getOgreColor();
      visual->setColorRaw( color.r, color.g, color.b, alpha );
      scale = static_cast<rviz::FloatProperty*>( list_raw_->subProp( "Scale" ) )->getFloat();
      visual->setScaleRaw( scale );
    }
  else
    {
      visual->clearMessageRaw();
    }
  if( show_tracked_property_->getBool() )
    {
      visual->setMessageTracked( msg );      
      alpha = static_cast<rviz::FloatProperty*>( list_tracked_->subProp( "Alpha" ) )->getFloat();
      color = static_cast<rviz::ColorProperty*>( list_tracked_->subProp( "Color" ) )->getOgreColor();
      visual->setColorTracked( color.r, color.g, color.b, alpha );
      scale = static_cast<rviz::FloatProperty*>( list_tracked_->subProp( "Scale" ) )->getFloat();
      visual->setScaleTracked( scale );
    }
  else
    {
      visual->clearMessageTracked();
    }
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

} // end namespace rviz_radar_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_radar_plugin::RadarDisplay,rviz::Display )
