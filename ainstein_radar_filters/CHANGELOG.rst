^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2019-10-29)
------------------
* Minor, add radar SNR as laserscan intensities
* Refactor pointcloud and laserscan converters
  Refactored the radar to pointcloud and laserscan conversion class and
  nodes in order to remove deprecated functionality and keep topic names
  consistent between them. The laserscan converter still has filtering
  based on the min/max angle/range parameters which should be removed and
  these parameters should be set from a radar sensor info message similar
  to camera info.
* Minor fixes to package XML formatting
  Fixed the package XML file formatting and added missing content to
  conform to the suggested style guidelines.
* Expose radar target array to point cloud conversion
  Exposed a function from ainstein_radar_filters which converts from the
  RadarTargetArray message type to the custom PCL point cloud type which
  includes radar data by making it static.  This was needed for new
  tools which need easy access to a polar-to-cartesian function. It may
  make more sense to pull out such conversions and put them in utility
  class somewhere else.
  Note that the only change to CMakeLists required to expose the header
  from this package to any package which imports it was to add an
  INCLUDE_DIRS line in the catkin_Package() function of this package.
* Add param for fixed frame to point cloud converter
  Add an optional fixed frame parameter for the radar target array to
  point cloud converted which takes in the name of the fixed frame,
  otherwise defaulting to map. Previously, map was hardcoded.
* Refactor redundant radar to pointcloud class
  Refactored the old, redundant radar to pointcloud converter class and
  associated node/lets to a radar speed filter class, preserving the
  projected speed filtering functionality. Tested on K79 data and working
  with mock zero speed command, should test further with nonzero GPS
  speed.
  There is still deprecated functionality in this class for testing the
  radar rotated which should be removed at some point as this was only
  experimental.
* Use custom PCL radar point for data converter class
  Switched from using the normal pcl::PointXYZ type to the custom radar
  specific ainstein_radar_filters::PointRadarTarget type in the radar to
  point cloud conversion class. Tested on radar data and verified that it
  allows coloring clouds according to additional radar-specific fields eg
  range, speed, etc. This permits using existing point cloud-based filter
  node/lets to filter based on radar parameters, deprecating eg the range
  filter class in this repo.
  Also removed a debug printout from the rviz plugin class.
* Contributors: Nick Rotella

1.0.3 (2019-10-03)
------------------
* Minor bug fix in tracking filter update
  Fixed a small bug in the Kalman Filter state covariance update equation
  which had an extra transpose in it. This likely didn't affect filter
  performance noticeably because it only affected off-diagonal elements.
* Contributors: Nick Rotella

1.0.2 (2019-09-25)
------------------

1.0.1 (2019-09-24)
------------------
* Refactor filters into separate subpkg, fix bug
  Created subpackage ainstein_radar_filters for radar filters and
  conversions, moved all filters from ainstein_radar_drivers into this
  subpkg and tested build and launch on rosbag data.
  Also fixed a small bug in the radar data range filter in which the
  dynamic reconfigure callback was not being registered, preventing the
  filter from working. Now, the filter compiles and works properly.
* Contributors: Nick Rotella
