^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
