/*
* @file vbs.cpp
* @Node name: vbs
* @Publishing to:
*   /vbs/<Float32>vbs_revs, /vbs/<Float32>vbs_pwm, /vbs/<Float32>vbs_dist
* @Subscribing to:
*   /vbs/<Float32>vbs_desired_dist, /vbs/<bool>vbs_calibrated
*
* @description
* 	Performs calcuations and publishes data related to the
* variable buoyancy system. Will read desired distance, and use empirically found
* coefficients of [distance/#ofrotations and distance/(pwm-seconds)] for closed
* and open loop approximations for current change in buoyancy of the vehicle.
* On startup, performs the calibration of the vbs - trips the button
* TWICE to be pulled low and moves the VBS to the neutral position.
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	9/14/2020: Adopted rudementary vbs code controlling angular velocity and
*            implemented std filter from dtt | rt
*   10/2/2020: Rewrote entire code to fit needs of MiniMAUV control| rt
* Need to:
*       - test code
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
#define vbs_adcPin 3

#define gpio_button_ChipNum 1
#define button_Pin 25

//shuts down if ctrl^c
void MySigintHandler(int sig) {
  ROS_INFO("Shutting down: variable buoyancy system");
  ros::shutdown();
  return;
}

//Respective node variables
const float vbs_max_distance = 100; //millimeters, fixed limit
const int sampleSize = 5;
const float maxVolt = 1.475;

int button_StateInit = -2; //inital value nonreturning from RCLibrary
int adder = 0; //to modify desired distance if in P controller it is too close to either end of potentiometer
float adcMean = 0; //averaged readings over (sampleSize) samples
float err = 0.00; //err for P controller

//float gearRatio = 1.4 mm/rev;
float gearRatio = .46;
float vbs_desired_dist = 50; //millimeters, sub'd
float vbs_olddesired_dist = 50; //millimeters, sub'd
float vbs_current_dist = 10; //millimeters, pub'd
bool vbs_calibrated = false; //pub'd and sub'd
float vbs_revs = 0; //local
float vbs_residual_rev = 0.000;
float vbs_pwm = 1475; //pub'd
int button_State = 0; //local
bool flagged = false;
float threshold = 0.4;//1.4mm per rev, to escape cusp of pot bounds, turn about ~90 degrees (arbitrary), take threshold as 0.4
float gain = 0; //min to move is 30 pwm, settle for 4mm deviation in buoyancy
float pconst = 10;

//Callback function which deposits subscription variables into respective node variables
void vbs_dist_Callback(const std_msgs::Float32::ConstPtr& msg) {
  vbs_desired_dist = msg->data;
}

void vbs_calibrated_Callback(const std_msgs::Bool::ConstPtr& msg) {
  vbs_calibrated = msg->data;
}

int main(int argc, char **argv) {

  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "vbs");

  // create a handle of the node (name used in the file)
  ros::NodeHandle vbs;
  signal(SIGINT, MySigintHandler);

  //creates subscriber
  ros::Subscriber vbs_dist_sub = vbs.subscribe("vbs/vbs_desired_dist",1,vbs_dist_Callback);
  ros::Subscriber vbs_calibrated_sub = vbs.subscribe("vbs/vbs_calibrated",1,vbs_calibrated_Callback);

  //creates publisher
  ros::Publisher vbs_current_dist_pub = vbs.advertise<std_msgs::Float32>("vbs/vbs_current_dist", 1);
  ros::Publisher vbs_pwm_pub = vbs.advertise<std_msgs::Float32>("vbs/vbs_pwm", 1);
  ros::Publisher vbs_revs_pub = vbs.advertise<std_msgs::Float32>("vbs/vbs_revs", 1);
  ros::Publisher vbs_calibrated_pub = vbs.advertise<std_msgs::Bool>("vbs/vbs_calibrated", 1);

  //pub vars
  std_msgs::Float32 cdist_msg;
  std_msgs::Float32 pwm_msg;
  std_msgs::Float32 revs_msg;
  std_msgs::Bool cal_msg;

  //ROS loop rate, 50 Hz
  ros::Rate loop_rate(50);

  //initializes pwm pins (again..?)
  if(rc_adc_init()) {
		ROS_INFO("ADC bus initialized\n");
	} else {
		ROS_INFO("ADC bus initialization failed\n");
	}

  rc_gpio_init(gpio_button_ChipNum, button_Pin, GPIOHANDLE_REQUEST_INPUT);

      while(ros::ok()) {
        button_StateInit = rc_gpio_get_value(gpio_button_ChipNum,button_Pin);
        //while rosok && not calibrated
        while(ros::ok() && !vbs_calibrated) {
          ROS_INFO("CDist: %f, DDist: %f, ODDist: %f, Revs: %f, F: %i \n", vbs_current_dist, vbs_desired_dist, vbs_olddesired_dist, vbs_revs, (int)flagged);
          //vbs pwm out = 1475-w;
          pwm_msg.data = 1415;
          vbs_pwm_pub.publish(pwm_msg);
          //if button val changes
          button_State = rc_gpio_get_value(gpio_button_ChipNum,button_Pin);
          if(button_State != button_StateInit) {
            //for five seconds
              //vbs pwm out = 1475-100;
              //pauses publishing from node for 5 seconds, forces vbs to push past when button switches
              //ros::Duration(5).sleep();
            //calibrated = true;
            vbs_calibrated = true;
            vbs_current_dist = 0.00;
            vbs_revs = 0;
            cal_msg.data = vbs_calibrated;
            cdist_msg.data = vbs_current_dist;
            vbs_calibrated_pub.publish(cal_msg);
            vbs_current_dist_pub.publish(cdist_msg);
            ROS_INFO("vbs: calibrated");
            //break
            break;
          }
          ros::spinOnce();
          loop_rate.sleep();
        //end while
        }

        //while ros ok and calibrated
        while(ros::ok() && vbs_calibrated) {
          ROS_INFO("CDist: %f, DDist: %f, ODDist: %f, Revs: %f, F: %i , err: %f, gain: %f, adcMean: %f\n", vbs_current_dist, vbs_desired_dist, vbs_olddesired_dist, vbs_revs, (int)flagged, err, gain, adcMean);

          //if current desired distance != old desired distance
            //adder = 0;
          if(vbs_desired_dist != vbs_olddesired_dist) {
            adder = 0;
          }
          //if desired dist is greater than max distance
            //publish to desired distance "max distance"
          if(vbs_desired_dist >= vbs_max_distance) {
            vbs_desired_dist = vbs_max_distance;
            ROS_INFO("vbs: desired distance greater than max distance");
          }

          //reads adc with minimal filtering, currently using mean of five readings
          for(int i = 0; i < sampleSize; i++) {
            adcMean+= rc_adc_read_volt(2)/maxVolt;
            ros::Duration(0.010/((float)sampleSize)).sleep(); //takes max of 10 milliseconds of the 20 millisecond total program runtime
          }
          adcMean/=sampleSize; //averaged readings over 10 ms, returning percentage of full rotation
          //calculates error, desired dist - current dist + adder
          err = vbs_desired_dist - vbs_current_dist;
          //if(err is positive && adc is greater than 95 percent && not flagged)
            //revs++
            //flagged
          if(err >= 0 && adcMean >=0.95 && !flagged) {
            vbs_revs++;
            flagged = true;
          }
          //if(err is negative && adc is less than 5 percent && not flagged)
            //revs--
            //flagged
          if(err <= 0 && adcMean <= 0.05 && !flagged) {
            vbs_revs--;
            flagged = true;
          }
          //if(flagged && adc > 5 percent && adc < 95 percent)
            //unflag
          if(flagged && adcMean >0.05 && adcMean < 0.95) {
            flagged = !flagged;
          }
          //if(err is less than threshold && adc < 10 percent && adc > 90 percent)
            //use local adder variable above or below the desired distance to get way from bounds of potentiometer
              //if err is positive, adder = positive threshold
              //if err is negative, adder = negative threshold
          if(err < threshold && adcMean < 0.1 && adcMean > 0.9) {
            if(err > 0) adder = threshold;
            if(err <= 0) adder = -threshold;
          }

          gain = pconst*(err+adder);
          //PWM out = 1475 + gain * (err+adder)
          if(gain >= 200) {
            gain = 200;
          }

          if(gain <= -200) {
            gain = -200;
          }

          vbs_residual_rev = adcMean;

          //vbs_pwm = 1475 + gain * (err + adder);
          vbs_pwm = 1475 + gain;
          vbs_current_dist = (vbs_revs+vbs_residual_rev)*gearRatio;
          //publishes pwm out
          pwm_msg.data = vbs_pwm;
          cdist_msg.data = vbs_current_dist;
          revs_msg.data = vbs_revs;
          vbs_pwm_pub.publish(pwm_msg);
          vbs_current_dist_pub.publish(cdist_msg);
          vbs_revs_pub.publish(revs_msg);
          //records old published value in order to reset adder at top of loop
          vbs_olddesired_dist = vbs_desired_dist; //when they don't equal eachother, it means it's updated and adder can be set to zero

          ros::spinOnce();
          loop_rate.sleep();
        //end while
        }
      }
    ros::spinOnce();
    return 0;
  }
