/*
* @file pwm_outs.cpp
* @Node name: pwm_outs
* @Publishing to:
*   none
*
* @Subscribing to:
*   std_msgs::Float32 (all) dtt/hfLeft_pwm, dtt/hfRight_pwm, dtt/escLeft_pwm,
*   dtt/escRight_pwm, vbs/vbs_pwm, mm/mm_ang
*
* @description
* 	Reads desired PWM from various drivers and outputs PWM at the required
* frequency of 50Hz.
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	9/13/2020, rt: Made pwm_outs.cpp
*
*
*
*/

#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <signal.h>
#include <chrono>
#include <unistd.h>

extern "C" {
	#include "getopt.h"
	#include "roboticscape.h"
	#include "rc/time.h"
	#include "rc/adc.h"
	#include "rc/servo.h"
	}
#include "ros/ros.h"

//include ros message types
#include "std_msgs/Float32.h"

//pin literals
#define hf_Left_Pin 1
#define hf_Right_Pin 2
#define vbs_Pin 3
#define mm_Pin 4
#define esc_Left_Pin 5
#define esc_Right_Pin 6


//shuts down if ctrl^c
void MySigintHandler(int sig) {
  ROS_INFO("Shutting down: pwm_outs");
  //neutral pwms for all servos and esc's
  rc_servo_send_pulse_us(hf_Left_Pin,1475);
	rc_servo_send_pulse_us(hf_Right_Pin,1475);
	rc_servo_send_pulse_us(vbs_Pin,1475);
	//rc_servo_send_pulse_us(mm_Pin,1833); removed because it would cause mm to jerk
	rc_servo_send_pulse_us(esc_Left_Pin,1500);
	rc_servo_send_pulse_us(esc_Right_Pin,1500);

  //powers off bbbl servo rail
	rc_servo_power_rail_en(0);
	rc_adc_cleanup();
  ros::shutdown();
  return;
}

//Respective node variables
float hfLeft_pwm = 1475;
float hfRight_pwm = 1475;
float vbs_pwm = 1475;
float mm_pwm = 1833.0;
float escLeft_pwm = 1500;
float escRight_pwm = 1500;

//Callback functions which deposits subscription variables into respective node variables
void hfleftCallback(const std_msgs::Float32::ConstPtr& msg) {
  hfLeft_pwm = msg->data;
}
void hfrightCallback(const std_msgs::Float32::ConstPtr& msg) {
  hfRight_pwm = msg->data;
}
void vbsCallback(const std_msgs::Float32::ConstPtr& msg) {
  vbs_pwm = msg->data;
}
void mmCallback(const std_msgs::Float32::ConstPtr& msg) {
  mm_pwm = msg->data;
}
void escleftCallback(const std_msgs::Float32::ConstPtr& msg) {
  escLeft_pwm = msg->data;
}
void escrightCallback(const std_msgs::Float32::ConstPtr& msg) {
  escRight_pwm = msg->data;
}

int main(int argc, char **argv) {
  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "pwm_outs");
  // create a handle of the node (name used in the file)
  ros::NodeHandle pwms;
  signal(SIGINT, MySigintHandler);

	//creates subscribers
  ros::Subscriber hf_Left_sub = pwms.subscribe("dtt/hfleft_pwm", 1,hfleftCallback);
  ros::Subscriber hf_Right_sub = pwms.subscribe("dtt/hfright_pwm", 1,hfrightCallback);
  ros::Subscriber vbs_sub = pwms.subscribe("vbs/vbs_pwm", 1,vbsCallback);
  ros::Subscriber mm_sub = pwms.subscribe("mm/mm_pwm", 1,mmCallback);
  ros::Subscriber esc_Left_sub = pwms.subscribe("dtt/escleft", 1,escleftCallback);
  ros::Subscriber esc_Right_sub = pwms.subscribe("dtt/escright", 1,escrightCallback);

	if(rc_servo_init()) {
		ROS_INFO("Servo bus initialized\n");
	} else {
		ROS_INFO("Servo bus initialization failed\n");
	}
	if(rc_adc_init()) {
		ROS_INFO("ADC bus initialized\n");
	} else {
		ROS_INFO("ADC bus initialization failed\n");
	}


  //50 hz, standard for servos and esc's
  ros::Rate loop_rate(50);

  /*
    //calibrates BlueRobotics ESC's
    //sends low
    rc_servo_send_pulse_us(esc_Left_Pin,1100);
  	rc_servo_send_pulse_us(esc_Right_Pin,1100);
    ROS_INFO("- Sent low . . . -\r\n");
    ros::Duration(1).sleep();

    //sends high
    rc_servo_send_pulse_us(esc_Left_Pin,1900);
  	rc_servo_send_pulse_us(esc_Right_Pin,1900);
    ros::Duration(2).sleep();
    */

    rc_servo_send_pulse_us(esc_Left_Pin,1500);
  	rc_servo_send_pulse_us(esc_Right_Pin,1500);
    //ROS_INFO("- - Arming phase complete - - \r\n\n\n");

    //ros::Duration(1).sleep();
    /*
		ROS_INFO("- - Entering loop : 3 - - \r\n");
    ros::Duration(1).sleep();
    ROS_INFO("- - Entering loop : 2 - - \r\n");
    ros::Duration(1).sleep();
    ROS_INFO("- - Entering loop : 1 - - \r\n");
    ros::Duration(1).sleep();
	*/

  while(ros::ok()) {

    //maintains the pwms published by respective nodes
    rc_servo_send_pulse_us(hf_Left_Pin,hfLeft_pwm);
  	rc_servo_send_pulse_us(hf_Right_Pin,hfRight_pwm);
  	rc_servo_send_pulse_us(vbs_Pin,vbs_pwm);
  	rc_servo_send_pulse_us(mm_Pin,mm_pwm);
  	rc_servo_send_pulse_us(esc_Left_Pin,escLeft_pwm);
  	rc_servo_send_pulse_us(esc_Right_Pin,escRight_pwm);
		//ROS_INFO("hfLeft: %f, hfRight: %f, escLeft: %f, esc_Right: %f\n", hfLeft_pwm, hfRight_pwm, escLeft_pwm, escRight_pwm);
		//ROS_INFO("vbs: %f, mm: %f\n", vbs_pwm, mm_pwm);
		//ROS_INFO("battery: %f\n", rc_adc_dc_jack());
    //passes to ros
    ros::spinOnce();

    //sleeps for 20 milliseconds
    loop_rate.sleep();
  }
  return 0;
}
