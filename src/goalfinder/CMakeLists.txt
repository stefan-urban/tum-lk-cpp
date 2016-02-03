cmake_minimum_required(VERSION 2.8.3)
project(goalfinder)

# C++11 compiler flag
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# ------------------------------------------------------------------------------
# ROS configuration

# Dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  aruco_msgs
  std_msgs
  nav_msgs
)

# Messages files
add_message_files( FILES Path.msg Goal.msg Goals.msg )
generate_messages( DEPENDENCIES std_msgs nav_msgs geometry_msgs )

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

# Goalfinder node
add_executable(goalfinder src/main.cpp src/TargetDetermination.cpp)
add_dependencies(goalfinder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goalfinder ${catkin_LIBRARIES} )
