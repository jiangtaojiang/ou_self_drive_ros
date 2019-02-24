cmake_minimum_required(VERSION 2.8.3)

project(yolo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#Compile a node written in C++
add_executable(YOLONode
	src/YOLONode.cpp
	src/yolo_node.cpp
)

target_link_libraries(YOLONode
	${catkin_LIBRARIES}
	${CMAKE_SOURCE_DIR}/darknet/libdarknet.a
)

