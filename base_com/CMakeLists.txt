cmake_minimum_required(VERSION 2.8.3)
project(base_com)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  riki_msgs
  std_msgs
  tf
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rikirobot
#  CATKIN_DEPENDS geometry_msgs ros_arduino_msgs roscpp rospy rosserial_python sensor_msgs std_msgs tf tf
#  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_com_node src/riki_base_node.cpp)
target_link_libraries(base_com_node ${catkin_LIBRARIES})
