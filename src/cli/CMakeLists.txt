cmake_minimum_required(VERSION 2.8.3)
project(cli)

# C++11 compiler flag
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


# ------------------------------------------------------------------------------
# ROS configuration

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------
# Build configuration

add_executable(cli src/main.cpp src/CommandLine.cpp)
add_dependencies(cli ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cli
  ${catkin_LIBRARIES}
)
