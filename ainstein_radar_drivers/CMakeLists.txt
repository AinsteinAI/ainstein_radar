cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 14)

project(ainstein_radar_drivers)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  can_msgs
  socketcan_bridge
  tf2
  tf2_ros
  ainstein_radar_msgs
  ainstein_radar_filters
  dynamic_reconfigure
  message_generation
  rviz_visual_tools
  )

add_message_files(
  FILES
  O79NetworkParams.msg
  O79DeviceParams.msg
  O79TrackingParams.msg
  WayvAirConfiguration.msg
  WayvAirStatus.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Must be called after catkin_python_setup() and before catkin_package():
generate_dynamic_reconfigure_options(
  cfg/ZoneOfInterestT79.cfg
  cfg/O79Params.cfg
  )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(o79_can_node src/o79_can_node.cpp src/radar_interface_o79_can.cpp src/utilities.cpp)
add_dependencies(o79_can_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(o79_can_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(o79_udp_node src/o79_udp_node.cpp src/radar_interface_o79_udp.cpp src/radar_driver_o79_udp.cpp src/utilities.cpp src/radar_device_alarms.cpp)
add_dependencies(o79_udp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(o79_udp_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(t79_node src/t79_node.cpp src/radar_interface_t79.cpp src/utilities.cpp)
add_dependencies(t79_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(t79_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(t79_bsd_node src/t79_bsd_node.cpp src/radar_interface_t79_bsd.cpp src/utilities.cpp)
add_dependencies(t79_bsd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(t79_bsd_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS
  nodes/wayv_node
  nodes/o79_config_node
  nodes/srd_d1_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS
  o79_can_node
  o79_udp_node
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
