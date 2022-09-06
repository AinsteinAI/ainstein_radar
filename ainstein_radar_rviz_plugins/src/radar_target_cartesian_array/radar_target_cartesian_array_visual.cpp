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
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/console.h>
#include <geometry_msgs/Vector3.h>

#include "radar_target_cartesian_array_visual.h"
#include "radar_target_cartesian_array_display.h"

namespace ainstein_radar_rviz_plugins
{

  const int RadarTargetCartesianArrayVisual::max_target_visuals = 1000;

  RadarTargetCartesianArrayVisual::RadarTargetCartesianArrayVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
  {
    scene_manager_ = scene_manager;

    // Create a node to store the pose of the Radar's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();

    // Set showing speed arrows to false by default:
    show_speed_arrows_ = false;

    // Set showing target info to false by default:
    show_target_info_ = false;

    // Set the target visuals vector capacities:
    radar_target_cartesian_visuals_.reserve(RadarTargetCartesianArrayVisual::max_target_visuals);
  }

  RadarTargetCartesianArrayVisual::~RadarTargetCartesianArrayVisual()
  {
    // Clear the visual:
    radar_target_cartesian_visuals_.clear();

    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode(frame_node_);
  }

  double target_range = 0;

  void RadarTargetCartesianArrayVisual::setMessage(const ainstein_radar_msgs::RadarTargetCartesianArray::ConstPtr &msg)
  {
    // Resize the target shapes vector:
    clearMessage();

    // Fill the target shapes from RadarTargetArray message:
    for (const auto &target : msg->targets)
    {
      // Create the new object position visual and push it back:
      radar_target_cartesian_visuals_.emplace_back(scene_manager_, frame_node_, shape_type_);

      double range;
      range = std::sqrt(std::pow(target.pose.position.x, 2.0) +
                        std::pow(target.pose.position.y, 2.0) +
                        std::pow(target.pose.position.z, 2.0));

      radar_target_cartesian_visuals_.back().range = range;

      if (range > min_range_ && range < max_range_)
      {
        // Create the new target shape, fill it and push back:
        radar_target_cartesian_visuals_.emplace_back(scene_manager_, frame_node_, shape_type_);

        // Copy the target into the shape:
        radar_target_cartesian_visuals_.back().t = target;

        // Compute the target's Cartesian position:
        radar_target_cartesian_visuals_.back().pos.setPosition(
            Ogre::Vector3(target.pose.position.x, target.pose.position.y, target.pose.position.z));

        radar_target_cartesian_visuals_.back().pos.setOrientation(
            Ogre::Quaternion(target.pose.orientation.w,
                             target.pose.orientation.x,
                             target.pose.orientation.y,
                             target.pose.orientation.z));

        radar_target_cartesian_visuals_.back().pos.setScale(Ogre::Vector3(scale_, scale_, scale_));

        // set the target speed arrow length:
        if (show_speed_arrows_)
        {
          radar_target_cartesian_visuals_.back().speed.set(std::abs(target.speed), // shaft length
                                                           0.05,                   // shaft diameter
                                                           0.1,                    // arrow head length
                                                           0.1);                   // arrow head diameter
        }
        else
        {
          radar_target_cartesian_visuals_.back().speed.set(0.0, 0.0, 0.0, 0.0);
        }

        // Set the target speed arrow position to the target position:
        radar_target_cartesian_visuals_.back().speed.setPosition(radar_target_cartesian_visuals_.back().pos.getPosition());

        // The target speed points toward the radar sensor:
        radar_target_cartesian_visuals_.back().speed.setDirection(radar_target_cartesian_visuals_.back().pos.getPosition() /
                                                                  std::copysign(range, target.speed));

        // Set the info text:
        if (show_target_info_)
        {
          radar_target_cartesian_visuals_.back().info.setLocalTranslation(radar_target_cartesian_visuals_.back().pos.getPosition());
          std::ostringstream ss;
          ss << target;
          radar_target_cartesian_visuals_.back().info.setCaption(ss.str());
        }
        else
        {
          radar_target_cartesian_visuals_.back().info.setColor(Ogre::ColourValue(0.0, 0.0, 0.0, 0.0));
        }
      }
    }
  }

  void RadarTargetCartesianArrayVisual::clearMessage(void)
  {
    radar_target_cartesian_visuals_.clear();
  }

  // Position is passed through to the SceneNode.
  void RadarTargetCartesianArrayVisual::setFramePosition(const Ogre::Vector3 &position)
  {
    frame_node_->setPosition(position);
  }

  // Orientation is passed through to the SceneNode.
  void RadarTargetCartesianArrayVisual::setFrameOrientation(const Ogre::Quaternion &orientation)
  {
    frame_node_->setOrientation(orientation);
  }

  // Color is passed through to the Shape object.
  void RadarTargetCartesianArrayVisual::setColor(int color_method, float r, float g, float b, float a)
  {
    switch (color_method)
    {
    case RadarTargetCartesianArrayDisplay::COLOR_METHOD_FLAT: // Flat coloring
    {
      for (auto &shape : radar_target_cartesian_visuals_)
      {
        shape.pos.setColor(Ogre::ColourValue(r, g, b, a));
        shape.speed.setColor(Ogre::ColourValue(r, g, b, a));
      }
    }
    break;

    case RadarTargetCartesianArrayDisplay::COLOR_METHOD_COLLISION_TIME: // Collision time-based coloring
    {
      // Compute the color based on time to collision:
      Ogre::ColourValue red = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);
      Ogre::ColourValue yellow = Ogre::ColourValue(1.0, 1.0, 0.0, 1.0);
      Ogre::ColourValue green = Ogre::ColourValue(0.0, 1.0, 0.0, 1.0);
      double min_time = 2.0;
      double max_time = 10.0;
      double total_time = max_time - min_time;

      for (auto &shape : radar_target_cartesian_visuals_)
      {
        double collision_time = shape.range / -shape.t.speed;

        // Objects moving away are at an infinite collision time:
        if (collision_time < 0.0)
        {
          collision_time = max_time;
        }

        // Bound the collision time:
        collision_time = std::min(std::max(collision_time, min_time), max_time);

        Ogre::ColourValue color;
        if (collision_time < min_time + 0.5 * total_time)
        {
          color.r = yellow.r + (1.0 - ((collision_time - min_time) / (0.5 * total_time))) * (red.r - yellow.r);
          color.g = yellow.g + (1.0 - ((collision_time - min_time) / (0.5 * total_time))) * (red.g - yellow.g);
          color.b = yellow.b + (1.0 - ((collision_time - min_time) / (0.5 * total_time))) * (red.b - yellow.b);
        }
        else
        {
          color.r = green.r + (1.0 - ((collision_time - min_time) / total_time)) * (yellow.r - green.r);
          color.g = green.g + (1.0 - ((collision_time - min_time) / total_time)) * (yellow.g - green.g);
          color.b = green.b + (1.0 - ((collision_time - min_time) / total_time)) * (yellow.b - green.b);
        }

        shape.pos.setColor(color);
        shape.speed.setColor(color);
      }
    }
    break;

    case RadarTargetCartesianArrayDisplay::COLOR_METHOD_SNR: // SNR-based coloring
    {
      radar_target_cartesian_visuals_.back().pos.setColor(Ogre::ColourValue(r, g, b, a));
      radar_target_cartesian_visuals_.back().speed.setColor(Ogre::ColourValue(r, g, b, a));
    }
    break;

    default:
      ROS_ERROR_STREAM("Invalid color method passed to setColor.");
      break;
    }
  }

  // Scale is passed through to the Shape object.
  void RadarTargetCartesianArrayVisual::setScale(float scale)
  {
    for (auto &shape : radar_target_cartesian_visuals_)
    {
      shape.pos.setScale(Ogre::Vector3(scale, scale, scale));
    }
  }

  void RadarTargetCartesianArrayVisual::setMinRange(float min_range)
  {
    min_range_ = min_range;
  }

  void RadarTargetCartesianArrayVisual::setMaxRange(float max_range)
  {
    max_range_ = max_range;
  }

  void RadarTargetCartesianArrayVisual::setShowSpeedArrows(bool show_speed_arrows)
  {
    // Store the desired state:
    show_speed_arrows_ = show_speed_arrows;

    // Update all the existing arrows if switched off:
    if (!show_speed_arrows_)
    {
      // Easiest way to hide arrows is to scale them down to zero:
      for (auto &t : radar_target_cartesian_visuals_)
      {
        t.speed.set(0.0, 0.0, 0.0, 0.0);
      }
    }
  }

  void RadarTargetCartesianArrayVisual::setShowTargetInfo(bool show_target_info)
  {
    // Store the desired state:
    show_target_info_ = show_target_info;

    // Update all the existing text if switched off:
    if (!show_target_info_)
    {
      for (auto &t : radar_target_cartesian_visuals_)
      {
        t.info.setColor(Ogre::ColourValue(0.0, 0.0, 0.0, 0.0));
      }
    }
  }

  void RadarTargetCartesianArrayVisual::setInfoTextHeight(float info_text_height)
  {
    // Store the desired state:
    info_text_height_ = info_text_height;

    // Update all the existing text if switched off:
    for (auto &t : radar_target_cartesian_visuals_)
    {
      t.info.setCharacterHeight(info_text_height_);
    }
  }

} // namespace ainstein_radar_rviz_plugins
