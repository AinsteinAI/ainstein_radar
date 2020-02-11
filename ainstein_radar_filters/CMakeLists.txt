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
  tf2_sensor_msgs
  ainstein_radar_msgs
  dynamic_reconfigure
  jsk_recognition_msgs
)

# Must be called after catkin_python_setup() and before catkin_package():
generate_dynamic_reconfigure_options(
  cfg/TrackingFilter.cfg
  cfg/TrackingFilterCartesian.cfg
  cfg/PassthroughFilter.cfg
  cfg/CombineFilter.cfg
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

add_library(radar_target_array_to_point_cloud_nodelet src/radar_target_array_to_point_cloud_nodelet.cpp src/radar_target_array_to_point_cloud.cpp)
add_dependencies(radar_target_array_to_point_cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_target_array_to_point_cloud_nodelet ${catkin_LIBRARIES})

add_executable(radar_target_array_to_laser_scan_node src/radar_target_array_to_laser_scan_node.cpp src/radar_target_array_to_laser_scan.cpp)
add_dependencies(radar_target_array_to_laser_scan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_target_array_to_laser_scan_node ${catkin_LIBRARIES})

add_library(radar_target_array_to_laser_scan_nodelet src/radar_target_array_to_laser_scan_nodelet.cpp)
add_dependencies(radar_target_array_to_laser_scan_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_target_array_to_laser_scan_nodelet ${catkin_LIBRARIES})

add_executable(nearest_target_filter_node src/nearest_target_filter_node.cpp src/nearest_target_filter.cpp)
add_dependencies(nearest_target_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nearest_target_filter_node ${catkin_LIBRARIES})

add_executable(tracking_filter_node src/tracking_filter_node.cpp src/tracking_filter.cpp src/radar_target_kf.cpp)
add_dependencies(tracking_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(tracking_filter_node ${catkin_LIBRARIES})

add_executable(tracking_filter_cartesian_node src/tracking_filter_cartesian_node.cpp src/tracking_filter_cartesian.cpp src/radar_target_cartesian_kf.cpp)
add_dependencies(tracking_filter_cartesian_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(tracking_filter_cartesian_node ${catkin_LIBRARIES})

add_executable(radar_passthrough_filter_node src/radar_passthrough_filter_node.cpp src/radar_passthrough_filter.cpp)
add_dependencies(radar_passthrough_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(radar_passthrough_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(radar_passthrough_filter_nodelet src/radar_passthrough_filter_nodelet.cpp src/radar_passthrough_filter.cpp)
add_dependencies(radar_passthrough_filter_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(radar_passthrough_filter_nodelet ${catkin_LIBRARIES})

add_executable(radar_combine_filter_node src/radar_combine_filter_node.cpp src/radar_combine_filter.cpp)
add_dependencies(radar_combine_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(radar_combine_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
  radar_target_array_speed_filter_node
  radar_target_array_speed_filter_nodelet
  radar_target_array_to_point_cloud_node
  radar_target_array_to_point_cloud_nodelet
  radar_target_array_to_laser_scan_node
  radar_target_array_to_laser_scan_nodelet
  nearest_target_filter_node
  tracking_filter_node
  tracking_filter_cartesian_node
  radar_passthrough_filter_node
  radar_passthrough_filter_nodelet
  radar_combine_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
