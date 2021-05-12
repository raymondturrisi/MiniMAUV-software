/*
* @file prototype_mission_1.cpp
* @Node name: prototype_mission_1
* @Publishing to:
*  All

* @Subscribing to:
*   none
*
* @description
* 	Prototype mission 1 with open loop controls
* 1) Center foils, calibrate vbs, center moving mass
* 2) move vbs to 50 mm
* 3) thrust forward for 10 seconds
* 4) Stop thrusters, rotate mass, retract vbs to 10 mm
* 5) when transitioned, canter foils at opposite 45 degrees and slowly thrust to spiral dive down for 3 seconds
* 6) fix foils at 180 degrees, adjust mass, go to max vbs distance
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	10/25/2020: Created prototype_mission_6, code confirmed
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
  ROS_INFO("Shutting down: prototype_mission_1");
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

void leakingCallback(const std_msgs::Bool::ConstPtr& msg) {
  leaking = msg->data;
}

int main(int argc, char **argv) {
  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "prototype_mission_6");
  // create a handle of the node (name used in the file)
  ros::NodeHandle pm6;
  signal(SIGINT, MySigintHandler);

  //subscribers: vbs_dist, vbs_calibrated, beat
  ros::Subscriber vbs_cdist_sub = pm6.subscribe("vbs/vbs_current_dist", 1, vbs_cdistCallback);
  ros::Subscriber vbs_calibrated_sub = pm6.subscribe("vbs/vbs_calibrated", 1, vbs_calibratedCallback);
  ros::Subscriber heartbeat_sub = pm6.subscribe("heartbeat/beat", 1, beatCallback);
  ros::Subscriber leaking_sub = pm6.subscribe("heartbeat/leaking", 1, leakingCallback);

  //publishers
  ros::Publisher hfleft_ang_pub = pm6.advertise<std_msgs::Float32>("dtt/hfleft_ang", 1);
  ros::Publisher hfright_ang_pub = pm6.advertise<std_msgs::Float32>("dtt/hfright_ang", 1);
  ros::Publisher escleft_pub = pm6.advertise<std_msgs::Float32>("dtt/escleft", 1);
  ros::Publisher escright_pub = pm6.advertise<std_msgs::Float32>("dtt/escright", 1);
  ros::Publisher vbs_desired_dist_pub = pm6.advertise<std_msgs::Float32>("vbs/vbs_desired_dist", 1);
  ros::Publisher mm_ang_pub = pm6.advertise<std_msgs::Float32>("mm/mm_ang", 1);
  ros::Publisher vbs_calibrated_pub = pm6.advertise<std_msgs::Bool>("vbs/vbs_calibrated", 1);

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

    //1) Center foils, calibrate vbs, center moving mass >> while not calibrated
    hfleft_ang.data = 180;
    hfright_ang.data = 180;
    escleft.data = 1500;
    escright.data = 1500;
    vbs_desired_dist.data = 20;
    mm_ang.data = 90;
    vbs_calibrated_pub_msg.data = false;
    vbs_calibrated_pub.publish(vbs_calibrated_pub_msg);


    while(!vbs_calibrated && !leaking) {
      ROS_INFO("1) calibrated %b\n", vbs_calibrated);
      hfleft_ang_pub.publish(hfleft_ang);
      hfright_ang_pub.publish(hfright_ang);
      escleft_pub.publish(escleft);
      escright_pub.publish(escright);
      vbs_desired_dist_pub.publish(vbs_desired_dist);
      mm_ang_pub.publish(mm_ang);
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }
    //2) move vbs to 50 mm >> while abs(cdist-ddist) > 5mm

    hfleft_ang.data = 180;
    hfright_ang.data = 180;
    escleft.data = 1500;
    escright.data = 1500;
    vbs_desired_dist.data = 80;
    mm_ang.data = 90;

    while((std::abs(vbs_desired_dist.data - vbs_cdist) > 5) && !leaking) {
      ROS_INFO("2) dist err %f\n", std::abs(vbs_desired_dist.data - vbs_cdist));
      hfleft_ang_pub.publish(hfleft_ang);
      hfright_ang_pub.publish(hfright_ang);
      escleft_pub.publish(escleft);
      escright_pub.publish(escright);
      vbs_desired_dist_pub.publish(vbs_desired_dist);
      mm_ang_pub.publish(mm_ang);
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }

    //3) thrust forward for 10 seconds >> for ~~
    hfleft_ang.data = 180;
    hfright_ang.data = 180;
    escleft.data = 1600;
    escright.data = 1600;
    vbs_desired_dist.data = 80;
    mm_ang.data = 90;

    for(int i = 0; i < 10; i++) {
      if(leaking) {
        break;
      }
      ROS_INFO("3) i %i\n", i);
      hfleft_ang_pub.publish(hfleft_ang);
      hfright_ang_pub.publish(hfright_ang);
      escleft_pub.publish(escleft);
      escright_pub.publish(escright);
      vbs_desired_dist_pub.publish(vbs_desired_dist);
      mm_ang_pub.publish(mm_ang);
      ros::spinOnce();
      ros::Duration(1).sleep();
    }
    //4) rotate mass, retract vbs to 10 mm >> while abs(cdist-ddist) > 5mm
    hfleft_ang.data = 180;
    hfright_ang.data = 180;
    escleft.data = 1500;
    escright.data = 1500;
    vbs_desired_dist.data = 10;
    mm_ang.data = 90;

    for(int i = 0; i < 10; i++) {
      if(leaking) {
        break;
      }
      ROS_INFO("4a) hard break %f\n", escleft.data);
      escleft.data-=20;
      escright.data-=20;
      escleft_pub.publish(escleft);
      escright_pub.publish(escright);
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }

    hfleft_ang.data = 180;
    hfright_ang.data = 180;
    escleft.data = 1500;
    escright.data = 1500;
    vbs_desired_dist.data = 10;
    mm_ang.data = 270;

    while((std::abs(vbs_desired_dist.data - vbs_cdist) > 5) && !leaking) {
      ROS_INFO("4b) dist err %f\n", std::abs(vbs_desired_dist.data - vbs_cdist));
      hfleft_ang_pub.publish(hfleft_ang);
      hfright_ang_pub.publish(hfright_ang);
      escleft_pub.publish(escleft);
      escright_pub.publish(escright);
      vbs_desired_dist_pub.publish(vbs_desired_dist);
      mm_ang_pub.publish(mm_ang);
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }
    //5) when transitioned, canter foils at opposite 45 degrees and slowly thrust to spiral dive down for 10 seconds >> for ~~
    hfleft_ang.data = 75;
    hfright_ang.data = 285;
    escleft.data = 1550;
    escright.data = 1550;
    vbs_desired_dist.data = 10;
    mm_ang.data = 270;

    for(int i = 0; i < 10; i++) {
      if(leaking) {
        break;
      }
      ROS_INFO("5) spiraling %i\n", i);
      hfleft_ang_pub.publish(hfleft_ang);
      hfright_ang_pub.publish(hfright_ang);
      escleft_pub.publish(escleft);
      escright_pub.publish(escright);
      vbs_desired_dist_pub.publish(vbs_desired_dist);
      mm_ang_pub.publish(mm_ang);
      ros::spinOnce();
      ros::Duration(1).sleep();
    }

    //6) fix foils at 180 degrees, adjust mass, go to max vbs distance >> while abs(cdist-ddist) > 5mm
    //passes to ros
    hfleft_ang.data = 180;
    hfright_ang.data = 180;
    escleft.data = 1500;
    escright.data = 1500;
    vbs_desired_dist.data = 80;
    mm_ang.data = 90;

    while(std::abs(vbs_desired_dist.data - vbs_cdist) > 5) {
      ROS_INFO("6) dist err %f\n", std::abs(vbs_desired_dist.data - vbs_cdist));
      hfleft_ang_pub.publish(hfleft_ang);
      hfright_ang_pub.publish(hfright_ang);
      escleft_pub.publish(escleft);
      escright_pub.publish(escright);
      vbs_desired_dist_pub.publish(vbs_desired_dist);
      mm_ang_pub.publish(mm_ang);
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }

    ros::shutdown();
  }
  return 0;
}
