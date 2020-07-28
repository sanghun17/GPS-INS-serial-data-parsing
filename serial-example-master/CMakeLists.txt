cmake_minimum_required(VERSION 2.8.3)
project(serial_example)

find_package(catkin REQUIRED COMPONENTS
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

add_executable(serial_cho_node src/serial_cho_node.cpp)
target_link_libraries(serial_cho_node ${catkin_LIBRARIES})

add_executable(serial_example_node src/serial_example_node.cpp)
target_link_libraries(serial_example_node ${catkin_LIBRARIES})

add_executable(serial_hex_node src/serial_hex_node.cpp)
target_link_libraries(serial_hex_node ${catkin_LIBRARIES})


