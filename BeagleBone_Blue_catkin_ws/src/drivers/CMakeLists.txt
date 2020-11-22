cmake_minimum_required(VERSION 2.8.3)
project(drivers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	roscpp
	rospy
	std_msgs
	sensor_msgs
	message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs sensor_msgs roscpp rospy std_msgs message_runtime)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
)

link_directories (
/usr/lib
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/cpptests.cpp
# )

## Declare a C++ library
# add_library(Bar30
#   src/Bar30.cpp
# )

add_executable(dtt src/dtt.cpp)
target_link_libraries(dtt roboticscape ${catkin_LIBRARIES})

add_executable(vbs src/vbs.cpp)
target_link_libraries(vbs roboticscape ${catkin_LIBRARIES})

add_executable(pwm_outs src/pwm_outs.cpp)
target_link_libraries(pwm_outs roboticscape ${catkin_LIBRARIES})

add_executable(heartbeat src/mauv_heartbeat.cpp)
target_link_libraries(heartbeat roboticscape ${catkin_LIBRARIES})

add_executable(leaksensors src/leaksensors.cpp)
target_link_libraries(leaksensors roboticscape ${catkin_LIBRARIES})

add_executable(moving_mass src/moving_mass.cpp)
target_link_libraries(moving_mass roboticscape ${catkin_LIBRARIES})



#############
## Install ##
#############


#############
## Testing ##
#############
