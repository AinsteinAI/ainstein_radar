^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2019-10-03)
------------------

1.0.2 (2019-09-25)
------------------
* Remove unused includes causing dependency issues
* Contributors: Nick Rotella

1.0.1 (2019-09-24)
------------------
* Refactor filters into separate subpkg, fix bug
  Created subpackage ainstein_radar_filters for radar filters and
  conversions, moved all filters from ainstein_radar_drivers into this
  subpkg and tested build and launch on rosbag data.
  Also fixed a small bug in the radar data range filter in which the
  dynamic reconfigure callback was not being registered, preventing the
  filter from working. Now, the filter compiles and works properly.
* Fix CMake issue and move nodelets to plugins folder
  Added missing dependency to range filter nodelet which was setting up a
  build race condition in which the dynamic reconfigure header sometimes
  was built after the nodelet, causing the build to fail.
  Also created a plugins folder and moved all nodelet XML files there,
  and updated the package.xml to match.
* Build fix, add radardata range filter nodelet xml
  Added the radardata_range_filter nodelet XML file to the package XML
  which fixes a build issue involving missing config for this nodelet.
* Refactor K79 interface to break out pure C++ driver
  Refactored the K79 radar interface to break out a pure C/C++ driver
  class called RadarDriverK79 on which RadarInterfaceK79 now depends.
  Tested and working with K79-2D, would need to do the same for K79-3D
  as well as all other radars which have ROS-dependent drivers.
  The next step would be to move the pure C++ driver outside this pkg
  entirely and create an external dependency, allowing use of radars
  without needing ROS installed.
* Merge branch 'master' of https://github.com/AinsteinAI/ainstein_radar
* Add range filter dynamic reconfigure, needs testing
* Rename all instances of usharp3d to srd_d1
  Renamed all instances of usharp3d to srd_d1 to follow new product
  naming. srd_d1 python node should be tested and cleaned up and drop
  support for old firmware going forward.
* Minor, change topic naming for radar outputs
* Minor, fix timestamps on radar data
* Minor, remove sign flip from filter, was a hack
* Merge branch 'nick-addK79-3D'
* Add K79-3D node and fix azimuth K79-2D parser issue
  Added the K79 3D and verified the data, though additional testing may
  need to be done. Added the interface (which is very similar to K79-2D
  and should be combined later), node and launch file.
  Fixed an issue with azimuth parsing which caused targets to appear
  behind the sensor. This was due to the azimuth angle being read as
  a 16bit signed int when in fact it is only a single byte (unsigned).
  This change was also implemented to K79-3D.
* Small fix to K79 speed so that away is +ve
* Add K79-3D interface class and test node
  Added a new K79-3D interface class and node (and later, nodelet to be
  added).  This is very similar to the K79(-2D) class and node.
* Merge branch 'nick-fixK79Parsing'
* Merge branch 'master' of https://github.com/AinsteinAI/ainstein_radar
* Refactor tracking filter to use dynamic reconfigure
  Refactored the radar target tracking filter to use dynamically
  reconfigurable parameters both for high-level target tracking and
  low-level individual Kalman Filters (all of which share the same
  per-target KF parameters).
* Add native PCL point type for radar data conversion
  Added a new PCL point type based on pcl::PointXYZ with additional
  fields specific to radar data. This allows for visualizing targets
  based on range, speed, snr etc in RViz by color as can normally be
  done for x,y,z coloring which is useful for debugging.
  More importantly, this exposes radar-specific data to all PCL library
  functionality, for instance making easy to use any of the basic PCL
  filters wrapped in pcl_ros (http://wiki.ros.org/pcl_ros#ROS_nodelets)
  without adding any new implementations (though this will require
  nodelets which will be added next).
  This opens the posibility to do much more advanced filtering on radar
  data going forward.
* Fix parsing of K79 data using casts
  Added static_cast's to fix the parsing of K79 data from the char buffer
* Update usharp3d testing launch, fix bug in driver
  Updated the usharp3d testing launch file after outdoor testing.
  Fixed a time bug in the usharp3d python driver by switching to
  Time(0) which gets the latest message regardless of stamped time.
* Add support for new uSharp3D version
  New uSharp3D version firmware was modified to change the data format
  bsed on feedback given to Beijing. The python ROS node was modified
  to support old and new firmware using a ROS param to specify which.
* Add device and frame ID params to uSharp3D node
  The uSharp3D node is still the rough python node (yet to be re-
  written in c++) but optional frame_id and device_id parameters were
  added for modularity.  The defaults are "map" and "/dev/ttyUSB0",
  respectively.
* Modify and tune tracking filter, add test launch
  Modified the tracking filter to only use a measurement for one
  filter, rather than allowing multiple filters to use the same target.
  This was necessary because there were cases in which multiple filters
  would split off from one due to an inconsistent but valid target, and
  then these duplicates would be corrected by future, more accurate
  measurements and ALL remain in the list because they were all able to
  update with the same measurement.
  Tuned the tracking filter on single and two target data from outdoors
  testing of uSharp3D. This radar has particularly "jumpy" returns which
  require setting the confidence level and filter timeout lower to keep
  filters alive through measurement gaps. The process noise was decreased
  to keep the measurement covariance lower and prevent using bad targets.
  The measurement noise was increased to "smooth" jumps due to spotty
  targets, resulting in more of a low-pass tracking.
  Note: tuning was done by first filtering out distance targets using
  the range filter (>10m).
* Add rosconsole configuration file for debugging
  Added a config/ folder containing a rosconsole configuration file
  which enables printing DEBUG-level messages.  To enable this from
  a launch file, load the rosconsole config into the environment with:
  <env name="ROSCONSOLE_CONFIG_FILE" value="$(find ainstein_radar_drivers)/config/debug_rosconsole.conf"/>
* Add range filter for RadarTargetArray
  Added a range filter to remove targets from a RadarTargetArray which
  are outside specified parameter bounds for min/max range.  Added a node
  and nodelet based on this filter, with the intention being that other
  filters (for example, speed filter, angle filter, etc) can be run as
  nodelets and stacked for easy preprocessing of raw detections.
* Rename defines, add val gate thresh param
  Renamed the noise-related KF parameter defines to STDEV to make it
  clear that these are standard deviations, not variances (they get
  squared in the noise matrices).
  Rescoped filter parameters to a /filter namespace and added a
  validation gate threshold to the filter which is set by the user based
  on desired confidence level from a Chi Squared distribution with 4
  DoFs (measurement dimension).  These can be looked up in a table for
  now, eg the table "Lower-tail critical values of chi-square distribution
  with ν degrees of freedom " on the page:
  https://www.itl.nist.gov/div898/handbook/eda/section3/eda3674.htm
  Also changed publisher to publish tracked targets even if list is
  empty (otherwise Rviz plugin always displays last message which makes
  debugging the filter difficult).  Maybe change this back later.
* Move initial covariance from state to constructor
  Moved the initialization of the covariance for a filter from the
  state's constructor to being passed in from the filter. This is a
  step towards reading in KF parameters from rosparam.
* Add launch file for testing tracking with uSharp3D
* Add node for tracking targets based on raw detections
  Added a class which maintains a list of Kalman Filters instantiated
  from raw targets (detections).  The class adds a new filter for each
  detection and integrates the radar data to provide tracked targets at
  a fixed update rate, unlike raw detections which are can be sparse in
  time. Each time a RadarTargetArray is received, a callback passes the
  raw detection information to all tracked target filters for updating.
  Those detections which are unused by all filters cause a new filter to
  be spawned, while filters which have not been updated recently are
  pruned.
  Currently, only tested indoors. A number of low-level KF parameters
  are also fixed as constants, but should be exposed to the used for
  setting as ROS parameters. The node is functional but the interface
  will be improved.
* Add Kalman Filter for tracked radar detections
  Added simple Kalman Filter and nested filter state classed for
  implementing tracked target filtering from raw radar returns, similar
  to the filtering performed on some Ainstein radars in firmware. This
  class will be used by a node which maintains a vector of filters for
  tracked target candidates, matches new detections to tracked targets
  and publishes stable detections.
* Move nearest target filter into namespace
  Moved the nearest target filter class into the ainstein_radar namespace
  following convention, and updated the node.
* Move nearest target filter into namespace
  Moved the nearest target filter class into the ainstein_radar namespace
  following convention, and updated the node.
* Minor change to publisher in usharp3d node
  Changed the publisher for the usharp3d python node to only publish
  when there are valid targets (prevent spamming empty messages).
* Fix radar stamped msg, add nearest target filter
  Fixed the RadarTargetStamped message to use the unstamped RadarTarget
  message rather than duplicating fields.
  Added a nearest target filter which extracts the nearest target (by
  range) within set min/max range bounds and optionally low-pass filters
  it before publishing as both a RadarTargetStamped and as an array with
  one message (called "tracked").  Will remove the array published after
  implementing a proper tracked target filter.
* Add initial uSharp3D python node and launch file
  Added the uSharp3D python node based on Zhenyu's script, to be replaced
  with a C++ node taking parameters, remappings etc as needed.
  Also added a launch file for testing the radar - for now, it's very
  simple since there are no configurable parameters.
* Refactor ainstein_radar_drivers using new message definitions
  The ainstein_radar_drivers subpkg now builds using the new radar msg
  definitions, however ainstein_rviz_plugins does not build.
* Migrate old radar_ros_interface pkg to new ainstein_radar_drivers subpkg
* Contributors: Nick Rotella
