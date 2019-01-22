cmake_minimum_required(VERSION 2.8.3)

project(robot_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DLINUX)

find_package(catkin REQUIRED
  roscpp
  roslib
  robot_sim
  bhand
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

# ===================================
# ======   LWR4+ robot sim   ========
# ===================================

add_executable(lwr4p_robot_sim
  src/lwr4p_robot_sim.cpp
)

add_dependencies(lwr4p_robot_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lwr4p_robot_sim
  armadillo
  robot_sim
  ${catkin_LIBRARIES}
)

# ===================================
# ======   BH282 robot sim   ========
# ===================================

add_executable(bh282_robot_test
  src/bh282_robot_test.cpp
)

add_dependencies(bh282_robot_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(bh282_robot_test
  armadillo
  bhand
  ${catkin_LIBRARIES}
)

# +++++++++++++++++++++++++++++++++++++++
add_executable(stop_barrett
  src/stop_barrett.cpp
)

add_dependencies(stop_barrett
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(stop_barrett
  armadillo
  bhand
  ${catkin_LIBRARIES}
)

# ===================================
# ===================================
