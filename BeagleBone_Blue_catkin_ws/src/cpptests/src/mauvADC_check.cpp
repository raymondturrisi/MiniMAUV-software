//########## INCLUDE DIRECTORIES ##############
//Control servomotor in BBB using RCLibrary
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
	#include "rc/dsm.h"
	#include "rc/servo.h"
	#include "rc/mpu.h"
	}
#include "ros/ros.h"

//include ros message types
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <tf/tf.h>
//########## INCLUDE DIRECTORIES END ##############

//########## PROGRAM DEF TYPES ##############

#define servoLeft 1
#define servoRight 2
#define vbs 3
#define battery 4
#define escleft 5
#define escright 6


/*
typedef enum g_mode_t{
        G_MODE_RAD,
        G_MODE_DEG,
        G_MODE_RAW
} g_mode_t;
typedef enum a_mode_t{
        A_MODE_MS2,
        A_MODE_G,
        A_MODE_RAW
} a_mode_t;
*/
//########## PROGRAM DEF TYPES END ##############


int running = 1;

//for control C to turn off node
void MySigintHandler(int sig) {
	//shut things down
	//neutralizes the respective pwm lines
	rc_servo_send_pulse_us(servoLeft,1475);
	rc_servo_send_pulse_us(servoRight,1475);
	rc_servo_send_pulse_us(vbs,1475);
	rc_servo_send_pulse_us(escleft,1500);
	rc_servo_send_pulse_us(escright,1500);
	rc_adc_cleanup();
	//rc_mpu_power_off();
	rc_servo_power_rail_en(0);
	//rc_servo_cleanup();
	fflush(stdout);
	running=0;
	return;
}

//Main program
int main(int argc, char** argv) {
	//initialize ros node with passed arguments
	ros::init(argc,argv,"adcCheck");

	//Creates local node handle (object)
	ros::NodeHandle nh;

	std::cout << "started\n";

	double hz = 50.0000; //0.020 seconds that the servos are expecting a new pwm sig
	ros::Rate loop_rate(hz);

	//battery rotator parameters
	int pwmMin = 1000, pwmMax = 2000; //servo params
	int range = (pwmMax-pwmMin);
	int pwm = pwmMin, inc = range/20;
	unsigned long int count = 0;
	int subcount = 0;
	int vbsgain = 10;

	//initializes adc
	rc_adc_init();
	double hfleft, hfright, vbsang, batvolt;

	while(ros::ok && running) {

		//prints analog readings from hydrofoils and VBS
		hfleft = rc_adc_read_volt(1);
		hfright = rc_adc_read_volt(2);
		vbsang = rc_adc_read_volt(3);
		batvolt = rc_adc_dc_jack();

		printf("hf left = %f\n", hfleft);
		printf("hf right = %f\n\n", hfright);
		printf("vbs = %f\n\n", vbsang);
		printf("VIN = %f\n\n", batvolt);

		signal(SIGINT, MySigintHandler);
	
		ros::spinOnce();
		loop_rate.sleep();
	}
}
