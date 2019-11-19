^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2019-11-19)
------------------

2.0.1 (2019-11-12)
------------------

2.0.0 (2019-11-12)
------------------
* Update RadarInfo message and fix dependencies
  Updated the RadarInfo message with new fields fully describing the data
  and added comments to explain them.
  Also fixed a small dependency in the RViz plugins package for display
  of the RadarInfo messages, however this plugin is not complete anyway.
* Contributors: Nick Rotella

1.1.0 (2019-10-29)
------------------
* Minor fixes to package XML formatting
  Fixed the package XML file formatting and added missing content to
  conform to the suggested style guidelines.
* Use custom PCL radar point for data converter class
  Switched from using the normal pcl::PointXYZ type to the custom radar
  specific ainstein_radar_filters::PointRadarTarget type in the radar to
  point cloud conversion class. Tested on radar data and verified that it
  allows coloring clouds according to additional radar-specific fields eg
  range, speed, etc. This permits using existing point cloud-based filter
  node/lets to filter based on radar parameters, deprecating eg the range
  filter class in this repo.
  Also removed a debug printout from the rviz plugin class.
* Ensure RadarTargetArray visual cleared on hide
  Fixed bug so that the RadarTargetDisplay visuals get cleared when the
  display for them is hidden. Still fails to work sometimes, maybe there
  is a race condition - return to this later.
* Contributors: Nick Rotella

1.0.3 (2019-10-03)
------------------

1.0.2 (2019-09-25)
------------------

1.0.1 (2019-09-24)
------------------
* Merge branch 'master' of https://github.com/AinsteinAI/ainstein_radar
* Add RadarInfo RViz message filter display, compiles
  Added a message filter display for RadarInfo messages in RViz. It
  compiles and works fine but needs to be modified because the scaling
  of the FOV cone (currently the only supported visualization for this
  message type) does not work intuitively.  The position of the cone
  shape visual is also in the center of the cone, so the shape must
  also be translated. Come back to this later.
* Add collision time-based color to Rviz radar visual
  Added the option to color radar target array messages based on their
  time to collision (range divided by speed, corrected for direction)
  in addition to normal (flat) coloring. This is similar to the Rviz
  color options for PointCloud2 based on any fields in that message.
  Still needs cleaning up so that the non-flat color method hides the
  color selector, as well as adds configurable min/max collision times
  for determining coloring. The colors are currently fixed to green at
  >=10s, red at <=2s and yellow in the middle (with smooth interpolation)
* Refactor RViz plugin naming and clean up code
  Refactored the RViz plugin for the RadarTargetArray message type to
  reflect this name, allowing for other plugins for aggregate radar
  types (such as the deprecated RadarData which contained both raw and
  tracked target arrays).
  Also began code cleanup, removing irrelevant comments, fixing spacing
  etc.
* Fix multiple bugs causing Rviz plugin to crash
  Fixed bugs in the RadarTargetArray Rviz message filter plugin which
  were causing the plugin to crash when loaded. Tested all functionality
  and confirmed working with sample data from uSharp3D.
* Change raw pointers to rviz properties to smart pointers
* Rename ainstein_rviz_plugins to ainstein_radar_rviz_plugins for consistency
* Contributors: Nick Rotella
