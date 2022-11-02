cmake_minimum_required(VERSION 2.8.3)
project(serial_parser)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_parser_node src/etri_com_protocol.cpp)

target_link_libraries(serial_parser_node
  ${catkin_LIBRARIES}
)

