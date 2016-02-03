cmake_minimum_required(VERSION 2.8.3)
project(amcl_pose_tf_broadcaster)

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

add_executable(amcl_pose_tf_broadcaster src/main.cpp)
add_dependencies(amcl_pose_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(amcl_pose_tf_broadcaster
 ${catkin_LIBRARIES}
)
