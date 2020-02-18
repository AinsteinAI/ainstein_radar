^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2020-02-11)
------------------
* Minor, add missing dependency
* Contributors: Nick Rotella

3.0.0 (2020-02-06)
------------------
* Add Cartesian tracking filter tracked pose output
  Added publishing of tracked object poses as geometry_msgs/PoseArray msg
  populated from tracked Cartesian position and using Cartesian velocity
  to determine pose. Forward direction (+x) is defined by tracked 3d
  velocity direction, with other axes determined by completing a right
  handed frame with +z up. Unfortunately, velocities cannot be published
  themselves without defining a custom TwistArray message, but nothing
  similar is supported natively in eg RViz.
* Update Cartesian tracking filter, fix bugs and tune
  Updated the Cartesian tracking filter to use the measured position
  instead of measured unit direction, since we can measure range with
  most radars. Fixed other bugs in implementation and got working.
  Tuned resulting filter parameters to get decent tracking, now working
  similarly to old tracking filter (in spherical coordinates). Still need
  to add full Pose and Twist output; currently converting to RadarTarget
  for backwards-compatibility with other radar filters.
* Minor, remove outdated filter config
* Add Cartesian tracking filter, needs debugging
  Added a Cartesian coordinates tracking filter for radar data which
  stores object state as Cartesian position and velocity rather than
  spherical coordinate range/angles/speed. The advantage is that we can
  track the full Cartesian velocity by using information from the radar
  speed measurement (which is a projection onto the radian direction) and
  consecutive range/angle measurements. This unfortunately makes the KF
  into an EKF because the measurements are now nonlinear in Cartesian
  space.
  The filter compiles and runs but never outputs tracked targets in its
  current state, and thus needs debugging. Since the RadarTarget datatype
  is not capable of storing full 3d velocity anyway, we need to output
  eg Pose and Twist types instead. Since the filter stores uncertainty,
  we can use PoseWithCovarianceStamped and TwistWithCovarianceStamped.
* Add support for point cloud input to radar tracking
  Added the ability for the radar tracking filter to subscribe to either
  a RadarTargetArray or PointCloud2 message. Internally, subscribing to
  a point cloud message converts to radar message type and then calls the
  original radar callback. This was added for portability since radar and
  point cloud messages are used with radar sensors often interchangeably.
* Minor, explicitly copy header in conversion
* Minor, remove outdated nodelet plugin
* Merge branch 'master' of https://github.com/AinsteinAI/ainstein_radar
* Update combine filter example launch and fix bug
  Updated the radar combine filter example launch file in the
  ainstein_radar_filters package to show how to launch three K79 radars
  and run the combine filter.
  Also fixed a bug in which the dynamic configuration was set for all
  possible numbers of radars which caused runtime errors because only one
  of the time sync objects was valid. Now checking the number of topics
  and only setting that object.
* Fix race condition in KF-based tracking filter
  Fixed a race condition in which the KF update loop and the radar data
  callback were in a race condition, with the number of KFs possibly
  changing during processing of new incoming data. Added a mutex to
  synchronize these functions, however the downside is potentially
  slower processing speed due to blocking waits for mutex unlocks in
  the data callback. Should be fine for small numbers of KFs.
* Fix broken radar to point cloud conversion node
* Minor, change passthrough filter default field
* Add example launch for radar combine filter usage
  Added a launch file example for using the radar combine filter to sync
  three RadarTargetArray topics. Note that the topic names must be set
  as an arg and passed in through rosparam due to the parameter being a
  list of strings. Also note that the "slop" factor in message syncing
  has its default set in the cfg/ file and can be loaded using the usual
  dynamic reconfigure loader.
* Add up to 8 synchronized topics in combine filter
  Added the ability to synchronize up to 8 RadarTargetArray topics in the
  combine filter. In theory the approximate time synchronizer in ROS can
  support 9 topics, HOWEVER boost::bind (which is used to register the
  callback) only supports 9 total arguments, the first of which in this
  case must be the "this" pointer because we bind a class member function
  here (see https://www.boost.org/doc/libs/1_45_0/libs/bind/bind.html#NumberOfArguments)
  The ability to synchronize radar topics beyond 3 has not been tested
  with data (only checked for compilation).
* Merge branch 'master' of https://github.com/AinsteinAI/ainstein_radar
* Minor, add dynconfig update to combine filter
* Minor, needed to negate speed for K79-3D using new SNR firmware
* Refactor to clean up combine filter callbacks
  Refactored the combine filter's callbacks to use one central callback
  which works for any number of topics. This cleans up the overloaded
  callbacks a bit before adding the callbacks for >3 topics (to come
  next).
* Minor, refactor combine topic names for consistency
* Refactor combine filter for 2 or 3 topics via param
  Refactored the radar combine filter to approximately sync 2 or 3 topics
  by reading in a list of topic names and calling appropriate templated
  functions for setting up the time synchronizer and registering callback
  functions for each. This is not very clear but since the time sync
  variables are templated on the exact number of topics, the use of pre-
  defined templated variables for each number of topics seems unavoidable
  unfortunately.
  Tested by using rosbag_tools to change the frame ID of sample K79 data
  in different output bagfiles, playing back those bag files, remapping
  the topic name to something unique and defining static tf publishers to
  translate the data around before combining.
  Next, need to consolidate the actual combination callback code and add
  templated variables/functions for all other topic numbers (4-9, since
  ROS in C++ supports syncing up to 9 topics. Under the hood, this also
  uses messy templated code!)
* Fix radar passthrough filter output transform
  Fixed the output radar passthrough filter cloud to transform the data
  properly in all cases.
* Pull out coordinate transform utility functions
  Pulled utility functions for spherical<->Cartesian coordinate
  transforms and made message converstion depend on them.
* Minor, add passthrough output tf to original frame
* Fix radar passthrough filter output frame missing
  The output message published by the passthrough filter needs to be set
  explicitly, since the pcl conversions and passthrough doesn't preserve
  the frame ID (this should be debugged as it's desired for the frame to
  be set at each step). Thus, we explicitly set the output frame before
  publishing.
* Add input/output frame params to radar passthrough
  Added the ability to specify input and output frames for the radar
  passthrough filter which function in the same manner as those in
  the pcl_ros package.
  The input_frame parameter specifies the frame in which the filtering
  should be performed, ie the input message is first converted to this
  frame before filtering. Note that this converts to a PointCloud2 and
  uses doTransform, so at the moment it *does not transform the radar
  fields (range, angles)* and is meant to be used for xyz filtering like
  the pcl_ros version. By default, the input frame is used and no
  transform is performed.
  The output_frame specifies the frame to publish the filtered data in
  after filtering. This is the original frame of the message by default
  but can be anything, including the input_frame.
  This functionality needs testing with hardware to confirm, and may be
  refactored to use a radar-specific doTransform soon.
* Add PointCloud2 conversion to RadarTargetArray
  Added a ROS cloud to radar array conversion using the PCL to radar
  converter as an intermediate step.
* Finish combine filter for two radar topics, tested
  Finished writing and testing the radar combine filter to approximately
  sync two RadarTargetArray topics and combine them via PointCloud2
  combine functionality.
  This was tested with a single radar and the relay node from topic_tools
  package; next test with multiple radars.
* Comment out radar transform function, add later
  Commented out the radar transform function because the pointcloud
  transform function from pcl_ros does not support tf2 in any way, so
  the best way to do this is probably convert to a PointCloud2 message
  and use tf2::doTransform. This function itself is templated on the data
  type and should be specialized to RadarTargetArray to do native tf2
  transforming - add this later.
* Fix pcl to radar conversion, add convert to degrees
* Fix pcl to radar target conversion
  Fixed the pcl point to radar target conversion to calculate the new
  spherical coordinates instead of copying from the extra fiels in the
  PCL struct. This is necessary in case the point has been transformed
  before being converted back to a radar target.
* Add combine filter for two radar topics, needs test
  Added a filter to combine two radar topics using an approximate time
  synchronizer, however this assumes the original messages are both in
  the same frame ID. Next, need to add conversion to the specified output
  frame ID.
  Compiles but needs testing.
* Add changes to fix previous commit
* Modify laser scan conversion to add nodelet
  Added a laser scan conversion nodelet after modifying the laser scan
  conversion class.  Needs testing.
* Minor, add missing nodelet to install
* Remove range filter, deprecated by passthrough
  Removed the old radardata range filter because the new passthrough
  filter based on the PCL library passthrough filter functionally
  deprecates the range filter (passthrough is more general, as it
  applies to any field including range).
* Refactor tracking and nearest target filter naming
  Refactored the tracking filter and nearest target filter to rename them
  according to a new convention for filters, removing the word radar
  since this is evident from the namespace ainstein_radar_filters scope.
* Major refactor, add conversion header and nodelets
  Refactored the conversion utilities to live within a namespace instead
  of the radar to pointcloud class, changed their usage in all dependent
  files.
  Added nodelets for the passthrough and radar to pointcloud filters,
  tested on K79 data. Removed old nodelets which weren't being built
  properly.
* Add generic passthrough filter for radar data
  Added a generic radar target array passthrough filter which functions
  exactly the same as those provided by pcl_ros. Uses dynamic reconfigure
  to allow changing the filter field and limits as well. Tested and works
  as expected. Next, will add a nodelet version.
* Add conversions from point cloud back to radar data
  Added new conversions for PCL point/point cloud types to radar ROS
  message types (opposite of what previously existed).
* Refactor radar to ROS point cloud conversion node
  Refactored the raadr to point cloud conversion node to separate the
  class (which only has static functions that should be moved to a
  utilities library at some point instead) from the node itself so that
  other classs/nodes can use the conversion functionality.
* Minor fixes to radar+camera fusion launch and node
  Fixed the radar+camera fusion launch file to use the updated topic
  names for radar and camera data. Also fixed the fusion class itself to
  prevent crashing when empty bounding box arrays are processed. This
  node is still intended for use with the tracking filter.
* Contributors: Nick Rotella

2.0.2 (2019-11-19)
------------------
* Minor, fix header exports breaking bloom build
* Rename input/output radar topics
  Renamed all instances of radardata_in and radardata_out to radar_in and
  radar_out to conform with other packages.
* Fix laser scan converter params, remove deprecated
  Fixed the min/max range for the laserscan converted to be 0.0 and 100.0
  respectively by default so that the filtering by range doesn't affect
  most radars by default. These paremeters are required by the laserscan
  message and should instead be set from the RadarInfo message, to be
  done soon. Also removed some deprecated code.
* Remove deprecated file and code from pcl converter
  Removed an old file for testing the pointcloud (pcl) converter class
  and removed old code from the pointcloud converted class which was
  previously used to filter targets based on relative speed.
* Contributors: Nick Rotella

2.0.1 (2019-11-12)
------------------

2.0.0 (2019-11-12)
------------------
* Add K79 people tracking filter launch and params
* Add tf2_eigen dependency to build
* Minor, fix jsk messages dependency
* Add bounding box output from radar tracking filter
  Added publication of bounding boxes for the tracked targets of the
  radar target tracking filter, computed to bound all targets used for
  a Kalman Filter update at each step.  This is a sort of "model-based
  clustering" of radar data since the KF itself tracks with the aid of
  a simple motion model.
  Next, plan to add Cartesian pose+covariance output.
* Contributors: Nick Rotella

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
