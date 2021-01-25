cmake_minimum_required(VERSION 3.2.0)
project(mrs_simulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  gazebo_ros
  std_msgs
  cmake_modules
)

find_package(mavros_msgs REQUIRED)

add_message_files(DIRECTORY msg FILES
  SpawnerDiagnostics.msg  
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mavros_msgs message_runtime
  LIBRARIES mrs_simulation
  DEPENDS mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
)

add_library(mrs_simulation src/main.cpp)

add_dependencies(mrs_simulation
  ${mrs_simulation_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mrs_simulation
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
)
