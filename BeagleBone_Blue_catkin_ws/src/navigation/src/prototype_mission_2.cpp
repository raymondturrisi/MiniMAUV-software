/*
* @file prototype_mission_2.cpp
* @Node name: prototype_mission_2
* @Publishing to:
*  All

* @Subscribing to:
*   none
*
* @description
* 	Prototype mission 2 with open loop controls
* Holds MAUV in horizontal cruising configuration for 1 minute before closing node
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	11/14/2020: Created prototype_mission_2
*
*
*/

#include <sstream>
#include <thread>
#include <chrono>
#include <ctime>
#include <cmath>
#include <signal.h>
#include <ratio>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

//shuts down if ctrl^c
void MySigintHandler(int sig) {
  ROS_INFO("Shutting down: prototype_mission_2");
  ros::shutdown();
  return;
}

//subscriber variables
float vbs_cdist = 20, beat = 0;
bool vbs_calibrated = false, leaking = false;

//subscriber callbacks
void vbs_cdistCallback(const std_msgs::Float32::ConstPtr& msg) {
  vbs_cdist = msg->data;
}
void vbs_calibratedCallback(const std_msgs::Bool::ConstPtr& msg) {
  vbs_calibrated = msg->data;
}
void beatCallback(const std_msgs::Float32::ConstPtr& msg) {
  beat = msg->data;
}

void leakingCallback(const std_msgs::Float32::ConstPtr& msg) {
  leaking = msg->data;
}

int main(int argc, char **argv) {
  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "prototype_mission_6");
  // create a handle of the node (name used in the file)
  ros::NodeHandle pm2;
  signal(SIGINT, MySigintHandler);

  //subscribers: vbs_dist, vbs_calibrated, beat
  ros::Subscriber vbs_cdist_sub = pm2.subscribe("vbs/vbs_current_dist", 1, vbs_cdistCallback);
  ros::Subscriber vbs_calibrated_sub = pm2.subscribe("vbs/vbs_calibrated", 1, vbs_calibratedCallback);
  ros::Subscriber heartbeat_sub = pm2.subscribe("heartbeat/beat", 1, beatCallback);
  ros::Subscriber leaking_sub = pm2.subscribe("heartbeat/leaking", 1, leakingCallback);

  //publishers
  ros::Publisher hfleft_ang_pub = pm2.advertise<std_msgs::Float32>("dtt/hfleft_ang", 1);
  ros::Publisher hfright_ang_pub = pm2.advertise<std_msgs::Float32>("dtt/hfright_ang", 1);
  ros::Publisher escleft_pub = pm2.advertise<std_msgs::Float32>("dtt/escleft", 1);
  ros::Publisher escright_pub = pm2.advertise<std_msgs::Float32>("dtt/escright", 1);
  ros::Publisher vbs_desired_dist_pub = pm2.advertise<std_msgs::Float32>("vbs/vbs_desired_dist", 1);
  ros::Publisher mm_ang_pub = pm2.advertise<std_msgs::Float32>("mm/mm_ang", 1);
  ros::Publisher vbs_calibrated_pub = pm2.advertise<std_msgs::Bool>("vbs/vbs_calibrated", 1);

  //publishing variables
  std_msgs::Float32 hfleft_ang;
  std_msgs::Float32 hfright_ang;
  std_msgs::Float32 escleft;
  std_msgs::Float32 escright;
  std_msgs::Float32 vbs_desired_dist;
  std_msgs::Bool vbs_calibrated_pub_msg;
  std_msgs::Float32 mm_ang;

  ros::Rate loop_rate(10);

  while(ros::ok()) {

    ROS_INFO("T-10..\n");
    for(int i = 10; i >= 0; i--) {
      ROS_INFO("%i\n", i);
      ros::Duration(1).sleep();
    }

    ROS_INFO("Starting\n");


    //1a) Get into vertical diving configuration
    hfleft_ang.data = 180;
    hfright_ang.data = 180;
    escleft.data = 1500;
    escright.data = 1500;
    vbs_desired_dist.data = 10;
    mm_ang.data = 270;

    while((std::abs(vbs_desired_dist.data - vbs_cdist) > 5) && !leaking) {
      ROS_INFO("1a) dist err %f\n", std::abs(vbs_desired_dist.data - vbs_cdist));
      hfleft_ang_pub.publish(hfleft_ang);
      hfright_ang_pub.publish(hfright_ang);
      escleft_pub.publish(escleft);
      escright_pub.publish(escright);
      vbs_desired_dist_pub.publish(vbs_desired_dist);
      mm_ang_pub.publish(mm_ang);
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }


    //Hold in vertical diving configuration for 60 seconds >> for ~~
    hfleft_ang.data = 180;
    hfright_ang.data = 180;
    escleft.data = 1500;
    escright.data = 1500;
    vbs_desired_dist.data = 10;
    mm_ang.data = 270;

    for(int i = 0; i < 60; i++) {
      if(leaking) {
        break;
      }
      ROS_INFO("1) i %i\n", i);
      hfleft_ang_pub.publish(hfleft_ang);
      hfright_ang_pub.publish(hfright_ang);
      escleft_pub.publish(escleft);
      escright_pub.publish(escright);
      vbs_desired_dist_pub.publish(vbs_desired_dist);
      mm_ang_pub.publish(mm_ang);
      ros::spinOnce();
      ros::Duration(1).sleep();
    }

    ros::shutdown();
  }
  return 0;
}
