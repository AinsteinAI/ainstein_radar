cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 11)

project(ainstein_radar_rviz_plugins)
find_package(catkin REQUIRED COMPONENTS
  rviz
  ainstein_radar_msgs
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

link_directories(
  ${catkin_LIBRARY_DIRS}
  )

## Set Qt's "MOC" generation to happen automatically:
set(CMAKE_AUTOMOC ON)

## Use the version of Qt used by RViz:
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## Prefer the Qt signals and slots to avoid defining "emit", "slots", etc because they
## can conflict with boost signals, so define QT_NO_KEYWORDS here:
add_definitions(-DQT_NO_KEYWORDS)

## List of source files. The generated MOC files are included automatically as headers:
set(SRC_FILES
  src/radar_target_array_display.cpp
  src/radar_target_array_visual.cpp
  src/radar_info_display.cpp
  src/radar_info_visual.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
