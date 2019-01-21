cmake_minimum_required(VERSION 2.8.3)

project(robot_sim)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DLINUX)

find_package(orocos_kdl)
find_package(catkin REQUIRED
  orocos_kdl
  kdl_parser
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS orocos_kdl
)

SET(SRC_LIST
  src/utils.cpp
  src/robot_chain.cpp
  src/robot_arm.cpp
  src/robot_hand.cpp
  src/lwr4p_robot.cpp
  src/bh282_robot.cpp
)

include_directories(
  include
  usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_LIST}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  pthread
  orocos-kdl
  armadillo
  ${catkin_LIBRARIES}
)
