cmake_minimum_required(VERSION 3.0.2)
project(rosbag_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  nav_msgs
  sensor_msgs
)
find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)
catkin_package()
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(rosbag_filter_node src/rosbag_filter.cpp)

set_target_properties(rosbag_filter_node PROPERTIES OUTPUT_NAME rosbag_filter_node PREFIX "")

add_dependencies(rosbag_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rosbag_filter_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
