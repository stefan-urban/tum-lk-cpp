cmake_minimum_required(VERSION 2.8.3)
project(driver)

# C++11 compiler flag
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# ------------------------------------------------------------------------------
# ROS configuration

# Dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  pathfinder
)

# Package configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES driver
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------
# Build configuration

include_directories(${catkin_INCLUDE_DIRS})

# Driver node
add_executable(driver
               src/main.cpp
               src/TurtleBot.cpp
               src/StateManager.cpp
               src/Driver.cpp
               src/StateIdle.cpp
               src/StateMovingTwist.cpp
               src/StateMovingLocation.cpp
               src/StateRotating.cpp
               src/StateBumperhit.cpp
               src/StateRandomWalk.cpp)

add_dependencies(driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(driver ${catkin_LIBRARIES} )
