cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(ainstein_radar_drivers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  can_msgs
  socketcan_bridge
  pcl_ros
  tf2
  tf2_ros
  ainstein_radar_msgs
  dynamic_reconfigure
)

# Must be called after catkin_python_setup() and before catkin_package():
generate_dynamic_reconfigure_options(
  cfg/TrackingFilter.cfg
  )

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 )

add_executable(k79_node src/k79_node.cpp src/radar_interface_k79.cpp)
add_dependencies(k79_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(k79_node ${catkin_LIBRARIES})

add_library(k79_nodelet src/k79_nodelet.cpp src/radar_interface_k79.cpp)
target_link_libraries(k79_nodelet ${catkin_LIBRARIES})

add_executable(t79_bsd_node src/t79_bsd_node.cpp src/radar_interface_t79_bsd.cpp)
add_dependencies(t79_bsd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(t79_bsd_node ${catkin_LIBRARIES})

add_library(t79_bsd_nodelet src/t79_bsd_nodelet.cpp src/radar_interface_t79_bsd.cpp)
target_link_libraries(t79_bsd_nodelet ${catkin_LIBRARIES})

add_executable(radardata_to_pointcloud_node src/radardata_to_pointcloud_node.cpp src/radardata_to_pointcloud.cpp)
add_dependencies(radardata_to_pointcloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radardata_to_pointcloud_node ${catkin_LIBRARIES})

add_library(radardata_to_pointcloud_nodelet src/radardata_to_pointcloud_nodelet.cpp src/radardata_to_pointcloud.cpp)
target_link_libraries(radardata_to_pointcloud_nodelet ${catkin_LIBRARIES})

add_executable(radardata_to_laserscan_node src/radardata_to_laserscan_node.cpp src/radardata_to_laserscan.cpp)
add_dependencies(radardata_to_laserscan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radardata_to_laserscan_node ${catkin_LIBRARIES})

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
target_link_libraries(radardata_range_filter_nodelet ${catkin_LIBRARIES})

install(TARGETS
  k79_node
  k79_nodelet
  t79_bsd_node
  t79_bsd_nodelet
  radardata_to_pointcloud_node
  radardata_to_pointcloud_nodelet
  radardata_to_laserscan_node
  radardata_to_nearest_target_node
  radardata_to_tracked_targets_node
  radardata_range_filter_node
  radardata_range_filter_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE)
