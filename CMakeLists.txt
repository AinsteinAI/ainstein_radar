cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(radar_ros_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
  visualization_msgs
  message_generation
  tf2
  tf2_ros
)

add_message_files(
  FILES
  RadarData.msg
  RadarTarget.msg
  RadarTargetRaw.msg
  RadarTargetTracked.msg
  RadarAlarm.msg
  K79TestPacket.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 )
 
add_executable(radar_ros_interface_test_node src/radar_ros_interface_test_node.cpp src/RadarNodeT79BSD.cpp)
add_dependencies(radar_ros_interface_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_ros_interface_test_node ${catkin_LIBRARIES})

add_executable(k79_test_node src/k79_test_node.cpp src/RadarNodeK79.cpp)
add_dependencies(k79_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(k79_test_node ${catkin_LIBRARIES})

install(TARGETS radar_ros_interface_test_node k79_test_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
