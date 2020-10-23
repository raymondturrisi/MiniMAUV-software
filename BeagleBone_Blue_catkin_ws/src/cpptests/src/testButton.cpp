//########## INCLUDE DIRECTORIES ##############
//Control servomotor in BBB using RCLibrary
#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <signal.h>
#include <chrono>
#include <unistd.h>
//#include <ifstream>
//#include <ofstream>

extern "C" {
	#include "getopt.h"
	#include "roboticscape.h"
	#include "rc/time.h"
	#include "rc/adc.h"
	#include "rc/dsm.h"
	#include "rc/servo.h"
	#include "rc/mpu.h"
	#include "rc/gpio.h"
	}
#include "ros/ros.h"
#include "std_msgs/Int32.h"

#define chip_Num 1
#define button_Pin 25

void MySigintHandler(int sig) {
    for(int i = 0; i < 250; i++) {
	    rc_servo_send_pulse_us(3,1475);
	    ros::Duration(0.02).sleep();
    }
	//rc_mpu_power_off();
	//rc_servo_cleanup();
	return;
}

int gain = 0; //positive drives forward

void gainCallback(const std_msgs::Int32::ConstPtr& msg) {
  gain = msg->data;
}

int main(int argc, char** argv) {
    
    signal(SIGINT, MySigintHandler);
    ros::init(argc,argv,"testButton");
    
    ros::NodeHandle nh;
    
    rc_servo_cleanup();
    rc_servo_power_rail_en(1);
    ROS_INFO("SERVO: %i",rc_servo_init());
    ROS_INFO("ADC: %i",rc_adc_init());
    ros::Duration(2).sleep();
    
    //creates subscriber
    ros::Subscriber gain_sub = nh.subscribe("vbs/gain",2, gainCallback);
  
    if(rc_gpio_init(chip_Num, button_Pin, GPIOHANDLE_REQUEST_INPUT)) {
        printf("error in initializing chip %i pin %i\n", chip_Num, button_Pin);
    } else {
        printf("initialized chip %i pin %i\n", chip_Num, button_Pin);
    }
    
    int val = 0;
    float vbsang = 0.00, vbsadcmax = 0.000;
    bool bounced = false;
    while(ros::ok()) {
        val = rc_gpio_get_value(chip_Num, button_Pin);
        vbsang = rc_adc_read_volt(3);
        if(vbsang > vbsadcmax) vbsadcmax = vbsang;
        //if(vbsang >= 300)
        //printf("val %f \n", val);
        
        ros::spinOnce();
        if(val >= 0.7 && !bounced) {
            printf("Bounced %i \n", val);
            bounced = true;
            gain*=(-1);
        }
        
        printf("vbsang %f, button pressed %i, gain %i, bounced %i\n", vbsang, val, gain, (int)bounced);
        printf("max voltage: %f", vbsadcmax);
        rc_servo_send_pulse_us(3,1475+gain);
        ros::Duration(0.02).sleep();
    }
    
    rc_gpio_cleanup(chip_Num, button_Pin);
    
    return 0;
}
