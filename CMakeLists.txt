cmake_minimum_required(VERSION 3.0.2)
project(myastar)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  nav_core
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(myastar
  src/myastar.cpp
  src/astar.cpp
)


add_dependencies(myastar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(myastar
  ${catkin_LIBRARIES}
)
