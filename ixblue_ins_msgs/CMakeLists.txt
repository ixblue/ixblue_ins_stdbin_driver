cmake_minimum_required(VERSION 2.8.3)
project(ixblue_ins_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  ins.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
)
