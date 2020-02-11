^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

1.1.0 (2019-10-29)
------------------
* Minor fixes to package XML formatting
  Fixed the package XML file formatting and added missing content to
  conform to the suggested style guidelines.
* Contributors: Nick Rotella

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
