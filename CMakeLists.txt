cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(radar_ros_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
  visualization_msgs
  pcl_ros
  tf2
  tf2_ros
  radar_sensor_msgs
)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 )

add_executable(radar_data_viz_node src/radar_data_viz_node.cpp)
add_dependencies(radar_data_viz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_data_viz_node ${catkin_LIBRARIES})

add_executable(radar_data_viz_point_cloud_node src/radar_data_viz_point_cloud_node.cpp src/RadarDataVizPointCloud.cpp)
add_dependencies(radar_data_viz_point_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_data_viz_point_cloud_node ${catkin_LIBRARIES})

add_executable(radar_data_to_laser_scan_node src/radar_data_to_laser_scan_node.cpp src/RadarDataToLaserScan.cpp)
add_dependencies(radar_data_to_laser_scan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_data_to_laser_scan_node ${catkin_LIBRARIES})

add_executable(car_demo_test_node src/car_demo_test_node.cpp src/RadarNodeT79BSD.cpp)
add_dependencies(car_demo_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_demo_test_node ${catkin_LIBRARIES})

add_executable(radar_node_k79 src/radar_node_k79.cpp src/RadarNodeK79.cpp)
add_dependencies(radar_node_k79 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_node_k79 ${catkin_LIBRARIES})

add_executable(radar_node_k79_gps src/radar_node_k79_gps.cpp src/RadarNodeK79GPS.cpp)
add_dependencies(radar_node_k79_gps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_node_k79_gps ${catkin_LIBRARIES})

install(TARGETS radar_data_viz_node radar_data_viz_point_cloud_node radar_data_to_laser_scan_node car_demo_test_node radar_node_k79 radar_node_k79_gps
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
