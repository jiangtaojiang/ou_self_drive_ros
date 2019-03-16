cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_BUILD_TYPE Debug)

project(yolo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  xmlrpcpp
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

#Compile a node written in C++
add_executable(YOLONode
	src/YOLONode.cpp
	src/yolo_node.cpp
	src/YOLO.cpp
)

target_link_libraries(YOLONode
	${catkin_LIBRARIES}
	m
    pthread
    stdc++
	cudart
    cuda    
    cublas
	curand
)

#change this to your darknet path!!!
target_link_libraries(YOLONode
	~/selfdrive/darknet/libdarknet.so
)


