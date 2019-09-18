cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(ainstein_radar_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  RadarInfo.msg
  RadarAlarm.msg
  RadarAlarmArray.msg
  RadarAlarmStamped.msg
  RadarTarget.msg
  RadarTargetArray.msg
  RadarTargetStamped.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

include_directories(
 ${catkin_INCLUDE_DIRS}
 )
 
install(TARGETS
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
