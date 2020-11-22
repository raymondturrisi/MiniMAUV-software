cmake_minimum_required(VERSION 2.8.3)
project(data)

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
 add_library(Bar30
   src/Bar30.cpp
 )

add_executable(pt_sensors src/pt_sensors.cpp)
target_link_libraries(pt_sensors Bar30 ${catkin_LIBRARIES})
target_link_libraries(pt_sensors roboticscape ${catkin_LIBRARIES})

add_executable(imu_mpu src/imu_mpu.cpp)
target_link_libraries(imu_mpu roboticscape ${catkin_LIBRARIES})

#add_executable(vi_sensors src/vi_sensors.cpp)
#target_link_libraries(vi_sensors roboticscape ${catkin_LIBRARIES})

#add_executable(filtering src/filtering.cpp)
#target_link_libraries(filtering roboticscape ${catkin_LIBRARIES})

#add_executable(recording src/recording.cpp)
#target_link_libraries(recording roboticscape ${catkin_LIBRARIES})


##Jame Strawson's C calibration programs
add_executable(calAccel src/rc_calibrate_accel.cpp)
target_link_libraries(calAccel roboticscape ${catkin_LIBRARIES})

add_executable(calMag src/rc_calibrate_mag.cpp)
target_link_libraries(calMag roboticscape ${catkin_LIBRARIES})

add_executable(calGyro src/rc_calibrate_gyro.cpp)
target_link_libraries(calGyro roboticscape ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############
