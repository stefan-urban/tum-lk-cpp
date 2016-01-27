cmake_minimum_required(VERSION 2.8.3)
project(amcl_tf_broadcaster)

# C++11 compiler flag
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# ------------------------------------------------------------------------------
# ROS configuration
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)


catkin_package()


# ------------------------------------------------------------------------------
# Build configuration

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(amcl_tf_broadcaster_node src/main.cpp)
add_dependencies(amcl_tf_broadcaster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(amcl_tf_broadcaster_node
  ${catkin_LIBRARIES}
)
