cmake_minimum_required(VERSION 3.1.3)
project(tactile_msgs)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs message_generation message_runtime
)

add_message_files(DIRECTORY msg FILES
  TactileState.msg
  TactileContact.msg
  TactileContacts.msg)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs message_runtime
)
