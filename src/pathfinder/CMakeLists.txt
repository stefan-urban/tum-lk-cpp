cmake_minimum_required(VERSION 2.8.3)
project(pathfinder)

# C++11 compiler flag
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# ------------------------------------------------------------------------------
# ROS configuration

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  goalfinder
  tf
)

# Messages files
add_message_files( FILES Path.msg )
generate_messages( DEPENDENCIES std_msgs nav_msgs )

catkin_package(
  CATKIN_DEPENDS goalfinder
)

# ------------------------------------------------------------------------------
# Build configuration

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pathfinder_node src/main.cpp src/Planner.cpp)
add_dependencies(pathfinder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pathfinder_node ${catkin_LIBRARIES})
