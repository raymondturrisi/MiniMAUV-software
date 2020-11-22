cmake_minimum_required(VERSION 2.8.3)
project(navigation)

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

#add_executable(mauv_brain src/mauv_brain.cpp)
#target_link_libraries(mauv_brain roboticscape ${catkin_LIBRARIES})

#add_executable(path_planning src/path_planning.cpp)
#target_link_libraries(path_planning roboticscape ${catkin_LIBRARIES})

add_executable(prototype_mission_1 src/prototype_mission_1.cpp)
target_link_libraries(prototype_mission_1 roboticscape ${catkin_LIBRARIES})

add_executable(prototype_mission_2 src/prototype_mission_2.cpp)
target_link_libraries(prototype_mission_2 roboticscape ${catkin_LIBRARIES})

add_executable(prototype_mission_3 src/prototype_mission_3.cpp)
target_link_libraries(prototype_mission_3 roboticscape ${catkin_LIBRARIES})

add_executable(prototype_mission_4_1 src/prototype_mission_4_1.cpp)
target_link_libraries(prototype_mission_4_1 roboticscape ${catkin_LIBRARIES})

add_executable(prototype_mission_4_2 src/prototype_mission_4_2.cpp)
target_link_libraries(prototype_mission_4_2 roboticscape ${catkin_LIBRARIES})

add_executable(prototype_mission_4_3 src/prototype_mission_4_3.cpp)
target_link_libraries(prototype_mission_4_3 roboticscape ${catkin_LIBRARIES})

add_executable(prototype_mission_5 src/prototype_mission_5.cpp)
target_link_libraries(prototype_mission_5 roboticscape ${catkin_LIBRARIES})

add_executable(prototype_mission_6 src/prototype_mission_6.cpp)
target_link_libraries(prototype_mission_6 roboticscape ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############
