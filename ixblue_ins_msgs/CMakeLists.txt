cmake_minimum_required(VERSION 3.8)

project(ixblue_ins_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(MSG_FILES
  "msg/Ins.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
  std_msgs
  geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()


# cmake_minimum_required(VERSION 3.0.2)
# project(ixblue_ins_msgs)

# find_package(catkin REQUIRED COMPONENTS
#   geometry_msgs
#   std_msgs
#   message_generation
# )

# ## Generate messages in the 'msg' folder
# add_message_files(
#   DIRECTORY msg
#   FILES
#   Ins.msg
# )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
# )

# ###################################
# ## catkin specific configuration ##
# ###################################
# catkin_package(
#   CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
# )
