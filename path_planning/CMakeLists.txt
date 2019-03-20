cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#Compile a node written in C++
add_executable(path_planning
  src/PathPlanning.cpp
  src/path_planning.cpp
)
add_dependencies(path_planning
  ${catkin_EXPORTED_TARGETS}
)

link_directories(${CMAKE_SOURCE_DIR}/path_planning/3rd_party_libraries)
include_directories(${CMAKE_SOURCE_DIR}/path_planning/3rd_party_libraries)

target_link_libraries(path_planning
  ${catkin_LIBRARIES}
  -L${CMAKE_SOURCE_DIR}/path_planning/3rd_party_libraries/libtinyspline.a
)
