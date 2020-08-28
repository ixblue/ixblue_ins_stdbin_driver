cmake_minimum_required(VERSION 2.8.3)
project(ixblue_ins_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_DEPS roscpp nav_msgs sensor_msgs ixblue_ins_msgs)
find_package(catkin REQUIRED ${CATKIN_DEPS})

## Specify libraries to link a library or executable target against
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

## System dependencies are found with CMake's conventions

# ixblue_stdbin_decoder must be set before Boost otherwise components are
# overridden (ixblue_stdbin_decoder only require system while we need
# program_options to build the packets replayer)
find_package(ixblue_stdbin_decoder REQUIRED)

# PCAP and Boost program_options are used to build the packets replayer
# that is only used for development so build and requise it as dependency
# if tests are built
if(CATKIN_ENABLE_TESTING)
   find_package(PCAP REQUIRED)
   find_package(Boost REQUIRED COMPONENTS system program_options)
else()
   find_package(Boost REQUIRED COMPONENTS system)
endif()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS ${CATKIN_DEPS}
   DEPENDS Boost ixblue_stdbin_decoder
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
aux_source_directory(src NODE_SRCS)
add_executable(${PROJECT_NAME}_node ${NODE_SRCS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ixblue_stdbin_decoder::ixblue_stdbin_decoder)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
   add_executable(packets_replayer tools/packets_replayer.cpp)
   target_link_libraries(packets_replayer ${PCAP_LIBRARY} ${Boost_LIBRARIES})

   ## Add gtest based cpp test target and link libraries
   catkin_add_gtest(${PROJECT_NAME}-test
     test/test_ixblue_ins_driver.cpp
     src/ros_publisher.cpp
   )
   if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ixblue_stdbin_decoder::ixblue_stdbin_decoder)
   endif()
endif()
