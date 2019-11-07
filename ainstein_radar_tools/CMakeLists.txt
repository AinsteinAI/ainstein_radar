cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(ainstein_radar_tools)

find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
cv_bridge
pcl_ros
roscpp
std_msgs
image_transport
image_geometry 
ainstein_radar_msgs
ainstein_radar_filters
vision_msgs
)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 )

add_executable(radar_camera_test_node src/radar_camera_test_node.cpp)
add_dependencies(radar_camera_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_camera_test_node ${catkin_LIBRARIES})

add_executable(radar_camera_fusion_node src/radar_camera_fusion_node.cpp src/radar_camera_fusion.cpp)
add_dependencies(radar_camera_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_camera_fusion_node ${catkin_LIBRARIES})

install(TARGETS
  radar_camera_test_node radar_camera_fusion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Install project namespaced headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE)
