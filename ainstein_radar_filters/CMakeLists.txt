cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(ainstein_radar_filters)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  pcl_ros
  tf2
  tf2_ros
  tf2_eigen
  ainstein_radar_msgs
  dynamic_reconfigure
  jsk_recognition_msgs
)

# Must be called after catkin_python_setup() and before catkin_package():
generate_dynamic_reconfigure_options(
  cfg/TrackingFilter.cfg
  cfg/RangeFilter.cfg
  )

catkin_package(INCLUDE_DIRS include)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 )

add_executable(radar_target_array_speed_filter_node src/radar_target_array_speed_filter_node.cpp src/radar_target_array_speed_filter.cpp)
add_dependencies(radar_target_array_speed_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_target_array_speed_filter_node ${catkin_LIBRARIES})

add_library(radar_target_array_speed_filter_nodelet src/radar_target_array_speed_filter_nodelet.cpp src/radar_target_array_speed_filter.cpp)
target_link_libraries(radar_target_array_speed_filter_nodelet ${catkin_LIBRARIES})

add_executable(radar_target_array_to_point_cloud_node src/radar_target_array_to_point_cloud_node.cpp src/radar_target_array_to_point_cloud.cpp)
add_dependencies(radar_target_array_to_point_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_target_array_to_point_cloud_node ${catkin_LIBRARIES})

add_executable(radar_target_array_to_laser_scan_node src/radar_target_array_to_laser_scan_node.cpp src/radar_target_array_to_laser_scan.cpp)
add_dependencies(radar_target_array_to_laser_scan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_target_array_to_laser_scan_node ${catkin_LIBRARIES})

add_executable(radardata_to_nearest_target_node src/radardata_to_nearest_target_node.cpp src/radardata_to_nearest_target.cpp)
add_dependencies(radardata_to_nearest_target_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radardata_to_nearest_target_node ${catkin_LIBRARIES})

add_executable(radardata_to_tracked_targets_node src/radardata_to_tracked_targets_node.cpp src/radardata_to_tracked_targets.cpp src/radar_target_kf.cpp)
add_dependencies(radardata_to_tracked_targets_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(radardata_to_tracked_targets_node ${catkin_LIBRARIES})

add_executable(radardata_range_filter_node src/radardata_range_filter_node.cpp src/radardata_range_filter.cpp)
add_dependencies(radardata_range_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radardata_range_filter_node ${catkin_LIBRARIES})

add_library(radardata_range_filter_nodelet src/radardata_range_filter_nodelet.cpp src/radardata_range_filter.cpp)
add_dependencies(radardata_range_filter_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radardata_range_filter_nodelet ${catkin_LIBRARIES})

install(TARGETS
  radar_target_array_speed_filter_node
  radar_target_array_speed_filter_nodelet
  radar_target_array_to_point_cloud_node
  radar_target_array_to_laser_scan_node
  radardata_to_nearest_target_node
  radardata_to_tracked_targets_node
  radardata_range_filter_node
  radardata_range_filter_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Install project namespaced headers
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE)
