cmake_minimum_required(VERSION 2.8.3)
project(gem_speed_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/SpeedControl.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/gem_speed_control.cpp
  src/GemSpeedControl.cpp
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
