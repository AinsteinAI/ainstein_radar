cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(radar_ros_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
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

add_executable(k79_node src/k79_node.cpp src/radar_interface_k79.cpp)
add_dependencies(k79_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(k79_node ${catkin_LIBRARIES})

add_library(k79_nodelet src/k79_nodelet.cpp src/radar_interface_k79.cpp)
target_link_libraries(k79_nodelet ${catkin_LIBRARIES})

add_executable(t79_bsd_node src/t79_bsd_node.cpp src/radar_interface_t79_bsd.cpp)
add_dependencies(t79_bsd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(t79_bsd_node ${catkin_LIBRARIES})

add_executable(radardata_to_pointcloud_node src/radardata_to_pointcloud_node.cpp src/radardata_to_pointcloud.cpp)
add_dependencies(radardata_to_pointcloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radardata_to_pointcloud_node ${catkin_LIBRARIES})

add_library(radardata_to_pointcloud_nodelet src/radardata_to_pointcloud_nodelet.cpp src/radardata_to_pointcloud.cpp)
target_link_libraries(radardata_to_pointcloud_nodelet ${catkin_LIBRARIES})

add_executable(radardata_to_laserscan_node src/radardata_to_laserscan_node.cpp src/radardata_to_laserscan.cpp)
add_dependencies(radardata_to_laserscan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radardata_to_laserscan_node ${catkin_LIBRARIES})

install(TARGETS k79_node k79_nodelet t79_bsd_node radardata_to_pointcloud_node radardata_to_pointcloud_nodelet radardata_to_laserscan_node
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
