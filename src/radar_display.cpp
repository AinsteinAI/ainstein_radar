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

namespace rviz_radar_plugin
{

  // The constructor must have no arguments, so we can't give the
  // constructor the parameters it needs to fully initialize.
  RadarDisplay::RadarDisplay()
  {
    // Options for displaying raw targets:
    show_raw_property_ = new rviz::BoolProperty( "Show Raw Targets", true,
						 "Toggles display of raw target markers.",
						 this, SLOT( updateShowTargets() ) );

    color_raw_ = new rviz::ColorProperty( "Color", QColor( 255, 0, 0 ),
					  "Color to draw the target markers.",
					  this, SLOT( updateColorAndAlpha() ) );
  
    alpha_raw_ = new rviz::FloatProperty( "Alpha", 1.0,
					  "Marker opacity. 0 is fully transparent, 1 is fully opaque.",
					  this, SLOT( updateColorAndAlpha() ) );

    scale_raw_ = new rviz::FloatProperty( "Scale", 0.2,
					  "Marker scale, in meters.",
					  this, SLOT( updateScale() ) );

    shape_raw_property_ = new rviz::EnumProperty( "Shape", "Cube",
						  "Target shape type.",
						  this, SLOT( updateTargetShape() ) );
    shape_raw_property_->addOptionStd( "Cube", 1 );
    shape_raw_property_->addOptionStd( "Sphere", 3 );

    // Options for displaying tracked targets:
    show_tracked_property_ = new rviz::BoolProperty( "Show Tracked Targets", true,
						     "Toggles display of tracked target markers.",
						     this, SLOT( updateShowTargets() ) );
  
    color_tracked_ = new rviz::ColorProperty( "Color", QColor( 255, 0, 0 ),
					      "Color to dtracked the target markers.",
					      this, SLOT( updateColorAndAlpha() ) );
  
    alpha_tracked_ = new rviz::FloatProperty( "Alpha", 1.0,
					      "Marker opacity. 0 is fully transparent, 1 is fully opaque.",
					      this, SLOT( updateColorAndAlpha() ) );

    scale_tracked_ = new rviz::FloatProperty( "Scale", 0.2,
					      "Marker scale, in meters.",
					      this, SLOT( updateScale() ) );

    shape_tracked_property_ = new rviz::EnumProperty( "Shape", "Cube",
						      "Target shape type.",
						      this, SLOT( updateTargetShape() ) );
    shape_tracked_property_->addOptionStd( "Cube", 1 );
    shape_tracked_property_->addOptionStd( "Sphere", 3 );
  
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

    // Determines whether to show the speed arrows:
    show_info_property_ = new rviz::BoolProperty( "Show Info", false,
						  "Toggles display of target info text.",
						  this, SLOT( updateShowTargetInfo() ) );

    info_text_height_property_ = new rviz::FloatProperty( "Info Text Height", 0.05,
							  "Target info text height.",
							  this, SLOT( updateInfoTextHeight() ) );
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

// Set whether to display raw and/or tracked markers.
void RadarDisplay::updateShowTargets()
{
  // Get the visual options settings:
  show_raw_ = show_raw_property_->getBool();
  show_tracked_ = show_tracked_property_->getBool();

  // Raw visual options:
  if( show_raw_ )
    {
      color_raw_->show();
      alpha_raw_->show();
      scale_raw_->show();
      shape_raw_property_->show();
    }
  else
    {
      color_raw_->hide();
      alpha_raw_->hide();
      scale_raw_->hide();
      shape_raw_property_->hide();
      visuals_.clear();
    }

  // Tracked visual options:
  if( show_tracked_ )
    {
      color_tracked_->show();
      alpha_tracked_->show();
      scale_tracked_->show();
      shape_tracked_property_->show();
    }
  else
    {
      color_tracked_->hide();
      alpha_tracked_->hide();
      scale_tracked_->hide();
      shape_tracked_property_->hide();
      visuals_.clear();
    }

  // General visual options:
  if( show_raw_ || show_tracked_ )
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
void RadarDisplay::processMessage( const radar_sensor_msgs::RadarData::ConstPtr& msg )
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

  
  // Set raw target data and visual options:
  if( show_raw_property_->getBool() )
    {
      // First set target shapes:
      visual->setTargetShapeRaw( shape_raw_property_->getOptionInt() );

      // Then set the target data from message:
      visual->setMessageRaw( msg );

      // Set the target visual options:
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

  // Set tracked target data and visual options:
  if( show_tracked_property_->getBool() )
    {
      // First set target shapes:
      visual->setTargetShapeTracked( shape_tracked_property_->getOptionInt() );

      // Then set the target data from message:
      visual->setMessageTracked( msg );

      // Set the target visual options:
      alpha = alpha_tracked_->getFloat();
      color = color_tracked_->getOgreColor();
      visual->setColorTracked( color.r, color.g, color.b, alpha );
      scale = scale_tracked_->getFloat();
      visual->setScaleTracked( scale );
    
    }
  else 
    {
      visual->clearMessageTracked();
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
    for( const auto& v : visuals_ )
      {
	v->setTargetShapeRaw( shape_raw_property_->getOptionInt() );
	v->setTargetShapeTracked( shape_tracked_property_->getOptionInt() );
      }       
  }
} // end namespace rviz_radar_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_radar_plugin::RadarDisplay,rviz::Display )
