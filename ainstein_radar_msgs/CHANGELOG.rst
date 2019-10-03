^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2019-10-03)
------------------

1.0.2 (2019-09-25)
------------------

1.0.1 (2019-09-24)
------------------
* Minor, remove unnecessary install target for radar_msgs
* Merge branch 'master' of https://github.com/AinsteinAI/ainstein_radar
* Add ainstein_radar_msgs/RadarInfo msg definition
  Added a new message type to ainstein_radar_msgs to store information
  about a radar sensor's properties, configuration, etc. Currently
  contains physical limits for range, speed and angles.
* Fix radar stamped msg, add nearest target filter
  Fixed the RadarTargetStamped message to use the unstamped RadarTarget
  message rather than duplicating fields.
  Added a nearest target filter which extracts the nearest target (by
  range) within set min/max range bounds and optionally low-pass filters
  it before publishing as both a RadarTargetStamped and as an array with
  one message (called "tracked").  Will remove the array published after
  implementing a proper tracked target filter.
* Refactor radar message types, other ainstein_radar subpkgs WILL NOT BUILD
  Refactored the message types defined for radars, breaking all other subpkgs
  in the ainstein_radar metapkg. Now refactoring all other pkgs to use the
  new message definitions.
* Migrate old radar_sensor_msgs pkg to new ainstein_radar_msgs subpkg
* Contributors: Nick Rotella
