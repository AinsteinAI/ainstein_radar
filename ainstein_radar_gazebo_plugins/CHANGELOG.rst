^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2019-10-03)
------------------

1.0.2 (2019-09-25)
------------------

1.0.1 (2019-09-24)
------------------
* Merge branch 'master' of https://github.com/AinsteinAI/ainstein_radar
* Minor fix to SDF and change default radar frame
  Updated the SDF file to use the correct dynamic library (was using old
  package) and changed the default radar sensor frame to base_link.
* Refactor Gazebo plugins subpkg as part of metapkg
* Migrate old gazebo_radar_sensor_plugin to new ainstein_radar_gazebo_plugins subpkg
  Currently will not compile, requires modifcation to use new
  ainstein_radar package.
* Contributors: Nick Rotella
