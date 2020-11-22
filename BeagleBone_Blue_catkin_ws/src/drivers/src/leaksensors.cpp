/*
* @file leaksensors.cpp
* @Node name: mauv_heartbeat
* @Publishing to:
*  <std_msgs::Bool>/heartbeat/leaking

* @Subscribing to:
*   none
*
* @description
* 	BR leak sensor
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	10/30/2020, rt: Created leaksensors.cpp, code confirmed
*
*/

#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <signal.h>
#include <chrono>
#include <unistd.h>
#include <cmath>

extern "C" {
  #include "getopt.h"
  #include "roboticscape.h"
  #include "rc/time.h"
  #include "rc/servo.h"
}
#include "ros/ros.h"

//include ros message types
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

//pin literals
#define gpio_ls_ChipNum 1
#define leak_Pin 17

//shuts down if ctrl^c
void MySigintHandler(int sig)
{
  ROS_INFO("Shutting down: leak sensors");
  ros::shutdown();
  return;
}

int main(int argc, char **argv) {

  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "leak_sensors");
  // create a handle of the node (name used in the file)
  ros::NodeHandle ls;
  signal(SIGINT, MySigintHandler);

  ros::Publisher ls_pub = ls.advertise<std_msgs::Bool>("heartbeat/leaking", 1);
  ros::Rate loop_rate(1);
  std_msgs::Bool leaking;
  leaking.data = false;
  int leaking_sensor = 0;
  rc_gpio_init(gpio_ls_ChipNum, leak_Pin, GPIOHANDLE_REQUEST_INPUT);

  while(ros::ok()) {

    leaking_sensor = rc_gpio_get_value(gpio_ls_ChipNum,leak_Pin);
    //if gpio low, not leaking, else - leaking
    if(leaking_sensor == 1) {
      leaking.data = true;
    } else {
      leaking.data = false;
    }
    //ROS_INFO("leaking: %i", leaking_sensor);
    ls_pub.publish(leaking);
    //passes to ros
    ros::spinOnce();
    //sleeps for 1 second
    loop_rate.sleep();
  }
  return 0;
}
