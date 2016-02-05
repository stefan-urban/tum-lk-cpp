cmake_minimum_required(VERSION 2.8.3)
project(driver_old)

# C++11 compiler flag
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# ------------------------------------------------------------------------------
# ROS configuration

# Dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  goalfinder
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

# Drive to one single goal
add_executable(single_goal
               src/single_goal.cpp)

add_dependencies(single_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(single_goal ${catkin_LIBRARIES} )

# Drive to codes 0-7
add_executable(multiple_goals
               src/multiple_goals.cpp)

add_dependencies(multiple_goals ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(multiple_goals ${catkin_LIBRARIES} )
