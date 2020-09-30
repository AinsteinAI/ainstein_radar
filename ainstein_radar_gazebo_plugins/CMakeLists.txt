cmake_minimum_required(VERSION 3.0.0)
project(ainstein_radar_gazebo_plugins)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  ainstein_radar_msgs
  )

find_package (gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp
  )

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} src/radar_plugin.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin )
