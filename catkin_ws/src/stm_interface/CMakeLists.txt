cmake_minimum_required(VERSION 3.0.2)
project(stm_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  RelayControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS  std_msgs message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
