/*
* @file moving_mass.cpp
* @Node name: moving_mass    [for moving mass]
* @Publishing to:
*   std_msgs::Float32 mm/mm_pwm
* @Subscribing to:
*   std_msgs::Float32 mm/mm_ang
*
* @description
* 	Translates commands to desired PWM
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	9/13/2020: Made moving_mass.cpp
*
*
*
*/
#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <signal.h>
#include <thread>
#include <chrono>
#include <unistd.h>

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

#define mmPin 4
//shuts down if ctrl^c
void MySigintHandler(int sig) {
  ROS_INFO("Shutting down: moving mass");
  ros::shutdown();
  return;
}

//Respective node variables
float mm_ang = 90;
bool moving = false;
//Callback function which deposits subscription variables into respective node variables
void mmCallback(const std_msgs::Float32::ConstPtr& msg) {
  mm_ang = msg->data;
	moving = true;
}

int main(int argc, char **argv) {

  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "moving_mass");
  // create a handle of the node (name used in the file)
  ros::NodeHandle mm;
  signal(SIGINT, MySigintHandler);

  //creates subscriber
  ros::Subscriber mm_sub = mm.subscribe("mm/mm_ang", 1, mmCallback);

  //creates publisher
  ros::Publisher mm_pub = mm.advertise<std_msgs::Float32>("mm/mm_pwm", 1);

  //ROS loop rate, 10 Hz, 0.1 seconds
  ros::Rate loop_rate(10);

  //unmodified servos: [500 - 2500 pwm, centered at 1500 pwm (135 degrees)] rotates ccw in increasing pwm
  //we want 0 (2500 pwm) to the right, clockwise to 270 (500 pwm) to the top
  //neutral at 90 (1833 pwm)

  //we want a travelling rate max of 180deg/5sec, 32deg/sec, 3.2deg/0.1sec

  float mm_pwm = 1833.0, mm_pwm_hold = 1833.0;

  std_msgs::Float32 mm_pwm_pub;
  mm_pwm_pub.data = 1833.0;
	mm_pub.publish(mm_pwm_pub);
	float mm_pwm_f = 1833.0;
  while(ros::ok()) {
    //conversion for bottom angle
    mm_pwm = 2500.0 - 2000.0*(mm_ang/270.0);
		//ROS_INFO("PWM %f, ANG %f", mm_pwm, mm_ang);
    //sends pwm for corresponding moving mass angle
    if(moving) {
			//if desired pwm is less than current pwm, increase
			if(mm_pwm > mm_pwm_hold) {
		      for(float i = mm_pwm_hold; i <= mm_pwm; i+=3.2) {
		          mm_pwm_pub.data = i;
		          mm_pub.publish(mm_pwm_pub);
							//ROS_INFO("less than %f\n", i);
		        //waits for 0.100 seconds
		        //loop_rate.sleep();
						ros::Duration(0.05).sleep();
		      }
					mm_pwm_f = mm_pwm;
			}
			//if desired pwm is greater than current pwm, decrease
			if(mm_pwm < mm_pwm_hold) {
		      for(float i = mm_pwm_hold; i >= mm_pwm; i-=3.2) {
		          mm_pwm_pub.data = i;
		          mm_pub.publish(mm_pwm_pub);
							//ROS_INFO("greater than than %f\n", i);
		        //waits for 0.100 seconds
		        //loop_rate.sleep();
						ros::Duration(0.05).sleep();
		      }
					mm_pwm_f = mm_pwm;
			}
			moving = false;
    }
    //finishes last call in loop and sends and holds battery in position
    mm_pwm_pub.data = mm_pwm_f;
    mm_pub.publish(mm_pwm_pub);
    //passes to ros
    ros::spinOnce();
    //saves last value
    mm_pwm_hold = mm_pwm;
  }
  return 0;
}
