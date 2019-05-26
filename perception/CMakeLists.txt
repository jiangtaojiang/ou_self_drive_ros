cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  pcl_ros
  yolo
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#Compile a node written in C++
add_executable(perception
  src/Point.cpp
  src/PointMap.cpp
  src/PointMapCell.cpp
  src/Perception.cpp
  src/perception_node.cpp
)
add_dependencies(perception
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(perception
  ${catkin_LIBRARIES}
)
