cmake_minimum_required(VERSION 3.2.0)
project(mrs_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  gazebo_ros
  std_msgs
  mrs_msgs
  )

find_package(mavros_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  LIBRARIES mrs_simulation
  DEPENDS mavros_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  )

add_library(mrs_simulation
  src/main.cpp
  )

add_dependencies(mrs_simulation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(mrs_simulation
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS mrs_simulation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  )

install(DIRECTORY ROMFS/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ROMFS
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
