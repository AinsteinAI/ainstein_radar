#ifndef RADAR_VISUAL_H
#define RADAR_VISUAL_H

#include <radar_sensor_msgs/RadarData.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
  class Shape;
}

namespace rviz_radar_plugin
{

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
  void setMessageTracked( const radar_sensor_msgs::RadarData::ConstPtr& msg );
  void clearMessageRaw( void );
  void clearMessageTracked( void );

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
  void setColorTracked( float r, float g, float b, float a );

  // Set the scale of the visual, which are user-editable
  // parameters and therefore don't come from the Radar message.
  void setScaleRaw( float scale );
  void setScaleTracked( float scale );

  // Update the targets based on new user-set parameters.
  void updateTargets();
  
  // Set the min and max range, which are user-editable
  // parameters and therefore don't come from the Radar message.
  void setMinRange( float min_range );
  void setMaxRange( float max_range );
  
private:
  // The object implementing the raw radar target shapes
  std::vector< boost::shared_ptr<rviz::Shape> > radar_target_shapes_raw_;

  // The object implementing the tracked radar target shapes
  std::vector< boost::shared_ptr<rviz::Shape> > radar_target_shapes_tracked_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Radar message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  // Minimum and maximum range, user-configurable filtering parameters:
  float min_range_;
  float max_range_;
};

} // end namespace rviz_radar_plugin

#endif // RADAR_VISUAL_H
