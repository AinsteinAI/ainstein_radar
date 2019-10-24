cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(ainstein_radar_drivers)

find_package(PCL REQUIRED)

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
  cfg/ZoneOfInterestT79.cfg
  )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

add_executable(k79_node src/k79_node.cpp src/radar_interface_k79.cpp src/radar_driver_k79.cpp)
add_dependencies(k79_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(k79_node ${catkin_LIBRARIES})

add_library(k79_nodelet src/k79_nodelet.cpp src/radar_interface_k79.cpp src/radar_driver_k79.cpp)
target_link_libraries(k79_nodelet ${catkin_LIBRARIES})

add_executable(k79_3d_node src/k79_3d_node.cpp src/radar_interface_k79_3d.cpp)
add_dependencies(k79_3d_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(k79_3d_node ${catkin_LIBRARIES})

add_library(k79_3d_nodelet src/k79_3d_nodelet.cpp src/radar_interface_k79_3d.cpp)
target_link_libraries(k79_3d_nodelet ${catkin_LIBRARIES})

add_executable(t79_node src/t79_node.cpp src/radar_interface_t79.cpp)
add_dependencies(t79_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(t79_node ${catkin_LIBRARIES})

add_executable(t79_bsd_node src/t79_bsd_node.cpp src/radar_interface_t79_bsd.cpp)
add_dependencies(t79_bsd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(t79_bsd_node ${catkin_LIBRARIES})

install(TARGETS
  k79_node
  k79_nodelet
  k79_3d_node
  k79_3d_nodelet
  t79_node
  t79_bsd_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Install project namespaced headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
