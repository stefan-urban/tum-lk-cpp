cmake_minimum_required(VERSION 2.8.3)
project(aruco_tf_broadcaster)

# C++11 compiler flag
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  aruco
  aruco_msgs
  aruco_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(broadcaster src/main.cpp)
add_dependencies(broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(broadcaster
 ${catkin_LIBRARIES}
)
