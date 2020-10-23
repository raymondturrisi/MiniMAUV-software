/*
* @file mauv_heartbeat.cpp
* @Node name: mauv_heartbeat
* @Publishing to:
*  All

* @Subscribing to:
*   none
*
* @description
* 	Prototype mission 1 with open loop controls
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	9/24/2020, rt: Created mauv_heartbeat.cpp
*
*
*
*/

#include <sstream>
#include <thread>
#include <chrono>
#include <ctime>
#include <ratio>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

//shuts down if ctrl^c
void MySigintHandler(int sig)
{
  ROS_INFO("Shutting down: prototype_mission_1");
  ros::shutdown();
}


int main(int arc, char **argv[]) {
  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "prototype_mission_1");
  // create a handle of the node (name used in the file)
  ros::NodeHandle pm1;
  signal(SIGINT, MySigintHandler);

  ros::Publisher hfleft_ang_pub = pm1.advertise<std_msgs::Float32>("dtt/hfleft_ang", 2);
  ros::Publisher hfright_ang_pub = pm1.advertise<std_msgs::Float32>("dtt/hfright_ang", 2);
  ros::Publisher escleft_pub = pm1.advertise<std_msgs::Float32>("dtt/escleft", 2);
  ros::Publisher escright_pub = pm1.advertise<std_msgs::Float32>("dtt/escright", 2);
  ros::Publisher vbs_desired_dist_pub = pm1.advertise<std_msgs::Float32>("vbs/vbs_desired_dist", 2);
  ros::Publisher mm_ang_pub = pm1.advertise<std_msgs::Float32>("mm/mm_ang", 2);

  ros::Rate loop_rate(100);

  while(ros::ok()) {

    ROS_INFO("Go-mode..");
    for(int i = 0; i < 10; i++) {
      ROS_INFO("%i", i);
      ros::Duration(1).sleep();
    }

    ROS_INFO("Starting");

    //hold steady
    for(int i = 0; i < 10; i++) {
      hfleft_ang_pub.publish(180);
      hfright_ang_pub.publish(180);
      escleft_pub.publish(1500);
      escright_pub.publish(1500);
      vbs_desired_dist_pub(3);
      mm_ang_pub.publish(90);
      ros::spinOnce();
      ros::Duration(1).sleep();
    }
    //move foils up
    for(int i = 0; i < 10; i++) {
      hfleft_ang_pub.publish(270);
      hfright_ang_pub.publish(270);
      escleft_pub.publish(1550);
      escright_pub.publish(1550);
      vbs_desired_dist_pub(3);
      mm_ang_pub.publish(90);
      ros::spinOnce();
      ros::Duration(1).sleep();
    }
    //move foils down
    for(int i = 0; i < 10; i++) {
      hfleft_ang_pub.publish(90);
      hfright_ang_pub.publish(90);
      escleft_pub.publish(1450);
      escright_pub.publish(1450);
      vbs_desired_dist_pub(3);
      mm_ang_pub.publish(90);
      ros::spinOnce();
      ros::Duration(1).sleep();
    }
    //center foils and thrust forward
    for(int i = 0; i < 10; i++) {
      hfleft_ang_pub.publish(180);
      hfright_ang_pub.publish(180);
      escleft_pub.publish(1600);
      escright_pub.publish(1600);
      vbs_desired_dist_pub(3);
      mm_ang_pub.publish(90);
      ros::spinOnce();
      ros::Duration(1).sleep();
    }
    //rotate battery and retract vbs
    for(int i = 0; i < 10; i++) {
      hfleft_ang_pub.publish(180);
      hfright_ang_pub.publish(180);
      escleft_pub.publish(1600);
      escright_pub.publish(1600);
      vbs_desired_dist_pub(1);
      mm_ang_pub.publish(270);
      ros::spinOnce();
      ros::Duration(1).sleep();
    }
    //passes to ros
    ros::spinOnce();
  }
  return 0;
}
