cmake_minimum_required(VERSION 3.0.2)
project(rosbag_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosbag_filter_node src/rosbag_filter.cpp)

target_link_libraries(rosbag_filter_node
  ${catkin_LIBRARIES}
)

add_dependencies(rosbag_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
