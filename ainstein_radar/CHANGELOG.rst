^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2020-02-11)
------------------

3.0.0 (2020-02-06)
------------------

2.0.2 (2019-11-19)
------------------

2.0.1 (2019-11-12)
------------------

2.0.0 (2019-11-12)
------------------
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
* Minor fixes to package XML formatting
  Fixed the package XML file formatting and added missing content to
  conform to the suggested style guidelines.
* Contributors: Nick Rotella

1.0.3 (2019-10-03)
------------------
* Minor corrections to metapackage package XML
* Contributors: Nick Rotella

1.0.2 (2019-09-25)
------------------

1.0.1 (2019-09-24)
------------------
* Migrate old radar_sensor_msgs pkg to new ainstein_radar_msgs subpkg
* New metapackage for Ainstein radar ROS support
* Contributors: Nick Rotella
