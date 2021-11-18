cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

add_message_files(
  FILES
  PathArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs nav_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/trajectory_generator_node.cpp src/trajectory_generator.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
