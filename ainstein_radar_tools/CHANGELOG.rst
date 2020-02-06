^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2020-02-06)
------------------
* Major refactor, add conversion header and nodelets
  Refactored the conversion utilities to live within a namespace instead
  of the radar to pointcloud class, changed their usage in all dependent
  files.
  Added nodelets for the passthrough and radar to pointcloud filters,
  tested on K79 data. Removed old nodelets which weren't being built
  properly.
* Fix radar camera fusion output image and launches
  Fixed the name of the radar+camera fusion class' output image topic to
  be scoped within a new private image transport instance, and fixed the
  launch files to use the correct topic name.
* Minor, fix launch for radar camera val
* Minor fixes to radar+camera fusion launch and node
  Fixed the radar+camera fusion launch file to use the updated topic
  names for radar and camera data. Also fixed the fusion class itself to
  prevent crashing when empty bounding box arrays are processed. This
  node is still intended for use with the tracking filter.
* Fix multiple target rendering without SNR alpha
  Fixed rendering multiple target rectangles when the SNR-based alpha is
  not used for blending. Now renders all targets instead of only the
  first one.
  Also fixed a few small issues with other files.
* Contributors: Nick Rotella

2.0.2 (2019-11-19)
------------------
* Use RadarInfo for sizing validation 2d bounding box
  Changed the radar-camera validation node, which draws 2d bounding boxes
  on the input image corresponding to the radar sensor's specifications,
  to use the actual RadarInfo message assumed to be published by any
  radar which publishes data. Needs testing on hardware.
* Separate radar camera validation class from node
* Rename radar camera test node, update launch files
  Renamed the radar camera "test" node to rdara camera "validation" and
  updated launch files for T79 and added one for K79. Testing again with
  K79 to verify this still works and get screenshots for wiki tutorials.
  In the future, should separate radar camera validation class from the
  node for portability, same as radar camera fusion class/node setup.
* Contributors: Nick Rotella

2.0.1 (2019-11-12)
------------------
* Add vision_msgs as ainstein_radar_tools dependency
* Contributors: Nick Rotella

2.0.0 (2019-11-12)
------------------
* Add changelog for new subpkg ainstein_radar_tools
* Add 3d bounding box output from radar camera fusion
  Added 3d bounding box publishing from the radar camera fusion class
  which uses the radar tracking filter 2d bounding box (assuming it is
  published) to get the width and depth of the object and uses the object
  height from the object detector (optionally also uses the object
  detector reported width instead of radar data). This is done by
  projecting the 2d image bounding box into 3d space at the distance
  of the tracked target.
* Add working radar/camera fusion using TensorFlow
  Added a working radar/camera fusion or "cross-validation" class which
  annotates objects detected from a camera image using a pre-trained
  TensorFlow-based 2d object detector with radar information for all
  detected objects which overlap with radar data.
  Functionality only has "runtime dependencies" on the TensorFlow object
  detector in the sense that fusion is driven by radar, camera, and
  detected object callbacks.  The fusion node is also prevented from
  running until the object detector node advertises a service indicating
  that it's ready. Finally, the label map from object index to string
  name is expected to be set in the parameter server as a dictionary by
  the object detector. The object detector itself could therefore be
  anything which outputs vision_msgs/Detection2DArray messages,
  advertises an "is ready" service and sets the label "database" (map)
  in the parameter server.
  For an example on how to use T79 with a RealSense d435 (RGB camera
  only) and set the correct topic/service/parameter mappings, see the
  launch file added in this commit.
* Developing radar+camera cross-validation "fusion"
  Testing a new node for radar+camera cross-validation using pre-trained
  TensorFlow models for 2d object detection combined with radar data to
  display bounding boxes associated with radar detections. WIP.
* Add new ainstein_radar_tools subpkg
  Added a new ainstein_radar_tools subpackage to ainstein_radar which is
  meant to store tools and utilities based on the other subpackages but
  not core to development, for example sensor fusion and SLAM nodes using
  radar data among other sensors.  This could arguably be broken out into
  its own package and will be if necessary, however the intent is for
  these tools to aid in development for anyone using Ainstein radars.
  The first and only tool in this subpackage is a simple replacement for
  the "CapApp" radar/camera sensor fusion application which draws boxes
  over the image to indicate targets. This requires a calibrated camera
  publishing CameraInfo messages (a RealSense d435i was used for the
  development). A sample launch file for using this with T79 is in the
  launch folder and will probably be replaced with a ROS wiki tutorial.
* Contributors: Nick Rotella

* Add 3d bounding box output from radar camera fusion
  Added 3d bounding box publishing from the radar camera fusion class
  which uses the radar tracking filter 2d bounding box (assuming it is
  published) to get the width and depth of the object and uses the object
  height from the object detector (optionally also uses the object
  detector reported width instead of radar data). This is done by
  projecting the 2d image bounding box into 3d space at the distance
  of the tracked target.
* Add working radar/camera fusion using TensorFlow
  Added a working radar/camera fusion or "cross-validation" class which
  annotates objects detected from a camera image using a pre-trained
  TensorFlow-based 2d object detector with radar information for all
  detected objects which overlap with radar data.
  Functionality only has "runtime dependencies" on the TensorFlow object
  detector in the sense that fusion is driven by radar, camera, and
  detected object callbacks.  The fusion node is also prevented from
  running until the object detector node advertises a service indicating
  that it's ready. Finally, the label map from object index to string
  name is expected to be set in the parameter server as a dictionary by
  the object detector. The object detector itself could therefore be
  anything which outputs vision_msgs/Detection2DArray messages,
  advertises an "is ready" service and sets the label "database" (map)
  in the parameter server.
  For an example on how to use T79 with a RealSense d435 (RGB camera
  only) and set the correct topic/service/parameter mappings, see the
  launch file added in this commit.
* Developing radar+camera cross-validation "fusion"
  Testing a new node for radar+camera cross-validation using pre-trained
  TensorFlow models for 2d object detection combined with radar data to
  display bounding boxes associated with radar detections. WIP.
* Add new ainstein_radar_tools subpkg
  Added a new ainstein_radar_tools subpackage to ainstein_radar which is
  meant to store tools and utilities based on the other subpackages but
  not core to development, for example sensor fusion and SLAM nodes using
  radar data among other sensors.  This could arguably be broken out into
  its own package and will be if necessary, however the intent is for
  these tools to aid in development for anyone using Ainstein radars.
  The first and only tool in this subpackage is a simple replacement for
  the "CapApp" radar/camera sensor fusion application which draws boxes
  over the image to indicate targets. This requires a calibrated camera
  publishing CameraInfo messages (a RealSense d435i was used for the
  development). A sample launch file for using this with T79 is in the
  launch folder and will probably be replaced with a ROS wiki tutorial.
* Contributors: Nick Rotella

1.1.0 (2019-10-29)
------------------

1.0.3 (2019-10-03)
------------------

1.0.2 (2019-09-25)
------------------

1.0.1 (2019-09-24)
------------------
