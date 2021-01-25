cmake_minimum_required(VERSION 3.2.0)
project(mrs_simulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  gazebo_ros
  std_msgs
)

find_package(mavros_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_message_files(DIRECTORY msg FILES
  SpawnerDiagnostics.msg  
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  LIBRARIES mrs_simulation
  DEPENDS mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
)

add_library(mrs_simulation src/main.cpp)

add_dependencies(mrs_simulation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mrs_simulation
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
)
