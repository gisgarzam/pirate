cmake_minimum_required(VERSION 2.8.3)
project(pirate)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED  actionlib actionlib_msgs roscpp rospy std_msgs genmsg geometry_msgs turtlesim sensor_msgs ram_input_msgs pirate_msgs pirate_srv)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
 DIRECTORY action
  FILES 
	Pirate.action
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs sensor_msgs)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapping_midi
  CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#add_executable(pirateserver_node src/pirate_server_node.cpp src/pirate_server.cpp)
#target_link_libraries(pirateserver_node ${catkin_LIBRARIES})
#add_dependencies(pirateserver_node ${pirate_EXPORTED_TARGETS})

add_executable(pirate_manager_node src/pirate_manager_node.cpp src/pirate_manager.cpp src/pirate_server.cpp src/motion_primitives.cpp src/control.cpp src/setpoint.cpp src/clamp_straight.cpp)
target_link_libraries(pirate_manager_node ${catkin_LIBRARIES})
add_dependencies(pirate_manager_node ${pirate_EXPORTED_TARGETS})

# FOR SERVICES - REMOVE LATER
add_executable(jointsrv_server src/joint_srv_server.cpp)
target_link_libraries(jointsrv_server ${catkin_LIBRARIES})
add_dependencies(jointsrv_server ${pirate_EXPORTED_TARGETS})

add_executable(jointsrv_client src/joint_srv_client.cpp)
target_link_libraries(jointsrv_client ${catkin_LIBRARIES})
add_dependencies(jointsrv_client ${pirate_EXPORTED_TARGETS})

add_executable(mapper src/mapper.cpp)
target_link_libraries(mapper ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-Wall -std=c++11")

