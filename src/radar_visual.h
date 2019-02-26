#ifndef RADAR_VISUAL_H
#define RADAR_VISUAL_H

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <radar_sensor_msgs/RadarData.h>

namespace rviz
{
  class Shape;
  class Arrow;
}

namespace rviz_radar_plugin
{

  // Create an aggregate class for different types of basic visual elements
  // to be used in visualizing radar data.
  //
  // Each TargetVisual contains a Shape for the target itself and an Arrow
  // to visualize the target speed (velocity).
  class TargetVisual
  {
  public:
  TargetVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) :
    pos( rviz::Shape::Cube, scene_manager, parent_node ),
      speed( scene_manager, parent_node ),
      info( "test" )
	{
	  parent_node->attachObject( &info );
	  info.setCharacterHeight( 0.1 );
	}      
    ~TargetVisual() {};

    rviz::Shape pos;
    rviz::Arrow speed;
    rviz::MovableText info;
  };
  
// Declare the visual class for this display.
//
// Each instance of RadarVisual represents the visualization of a single
// sensor_msgs::Radar message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class RadarVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  RadarVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~RadarVisual();

  // Configure the visual to show the data in the message.
  void setMessageRaw( const radar_sensor_msgs::RadarData::ConstPtr& msg );
  void clearMessageRaw( void );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way RadarVisual is only
  // responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Radar message.
  void setColorRaw( float r, float g, float b, float a );

  // Set the scale of the visual, which are user-editable
  // parameters and therefore don't come from the Radar message.
  void setScaleRaw( float scale );

  // Update the targets based on new user-set parameters.
  void updateTargets();
  
  // Set the min and max range, which are user-editable
  // parameters and therefore don't come from the Radar message.
  void setMinRange( float min_range );
  void setMaxRange( float max_range );

  // Set whether to render speed arrows:
  void setShowSpeedArrows( bool show_speed_arrows );

  // Set whether to render target info text:
  void setShowTargetInfo( bool show_target_info );
  
private:
  // The object implementing the radar target visuals
  std::vector< boost::shared_ptr<TargetVisual> > radar_target_visuals_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Radar message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  // Minimum and maximum range, user-configurable filtering parameters:
  float min_range_;
  float max_range_;

  // Determines whether speed arrows are rendered with zero length:
  bool show_speed_arrows_;

  // Determines whether target info is rendered:
  bool show_target_info_;
};

} // end namespace rviz_radar_plugin

#endif // RADAR_VISUAL_H
