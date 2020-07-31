cmake_minimum_required(VERSION 3.0.2)
project(hexserial)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
  geometry_msgs 
  sensor_msgs
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  LIBRARIES hexserial 
  CATKIN_DEPENDS message_runtime roscpp std_msgs serial geometry_msgs sensor_msgs
  DEPENDS system_lib

)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
) 	


add_executable(gps_data_pub src/serial_hex_node.cpp)
target_link_libraries(gps_data_pub ${catkin_LIBRARIES})
add_dependencies(gps_data_pub hexserial_generate_messages_cpp)

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/ros/melodic")
