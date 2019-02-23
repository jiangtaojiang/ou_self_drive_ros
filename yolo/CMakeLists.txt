cmake_minimum_required(VERSION 2.8.3)

project(new_package_name)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Compile a node written in C++
# add_executable(name_of_node
#   src/source1.cpp
#   src/source2.cpp
# )
# add_dependencies(name_of_node
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(name_of_node
#   ${catkin_LIBRARIES}
# )
