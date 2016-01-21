cmake_minimum_required(VERSION 2.8.3)
project(pathfinder)

# C++11 compiler flag
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# ------------------------------------------------------------------------------
# ROS configuration

# Dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  nav_msgs
  aruco_msgs
)

# Messages files
add_message_files( FILES Path.msg )
generate_messages( DEPENDENCIES std_msgs nav_msgs )

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

# Pathfinder node
add_executable(pathfinder src/main.cpp src/TargetDetermination.cpp src/Planner.cpp)
add_dependencies(pathfinder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pathfinder ${catkin_LIBRARIES} )
