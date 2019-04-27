cmake_minimum_required(VERSION 2.8.3)
project(radar)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  radar_detection.msg
  radar_detections.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


# add_dependencies(name_of_node
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(name_of_node
#   ${catkin_LIBRARIES}
# )
