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
	ros::init(argc,argv,"rosIMUServotest");
	/*
	//Configures IMU with RC_mpu, code specific
	rc_mpu_data_t data;
	rc_mpu_config_t conf = rc_mpu_default_config();
	g_mode_t g_mode = G_MODE_DEG; // gyro default to degree mode. OR 0
	a_mode_t a_mode = A_MODE_MS2; // accel default to m/s^2 OR 0
	static int enable_magnetometer = 1;
	static int enable_thermometer = 1;
	static int enable_warnings = 1;
	static int running = 1;

	#define I2C_BUS 2 //for BBBl
	conf.i2c_bus = I2C_BUS;
	conf.enable_magnetometer = enable_magnetometer;
	conf.show_warnings = enable_warnings;
	*/
	//Initializes IMU
	//rc_mpu_initialize(&data, conf);

	//Creates local node handle (object)
	ros::NodeHandle nh;
	//Creates publishers and subscribers objects
	//ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 2);
	//ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 2);
	//ros::Publisher rpy_pub = nh.advertise<geometry_msgs::Vector3>("/imu/rpy", 2);
	//ros::Subscriber imu_sub;

	/*
	//Sets up ROS messages
	sensor_msgs::Imu imu_data;
	sensor_msgs::MagneticField imu_mag;
	geometry_msgs::Vector3 rpy_data;
	std_msgs::Float64 heading_data;
	*/

	//imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 2);
	//mag_pub = nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 2);
	//rpy_pub = nh.advertise<geometry_msgs::Vector3>("/imu/rpy", 2);

	//int a = rc_servo_init();
	//int b = rc_servo_power_rail_en(1);

	//Initializes servos and ESCs
	rc_servo_cleanup();
	sleep(1);
	std::cout << rc_servo_init() << '\n'; //returns -1 if failure, or 0 if successful
	std::cout << rc_servo_power_rail_en(1) << '\n'; //turns on power rail
	sleep(1);

	std::cout << "started\n";

	double hz = 50.0000; //0.020 seconds that the servos are expecting a new pwm sig
	ros::Rate loop_rate(hz);

	//esc parameters
	printf("- - Arming . . . - - \r\n");
	//Sets 0 angular velocity
	double delay = 3.0000;
	for(double i = 0.0000; i < delay; i+=(1.0000/hz)) {
		rc_servo_send_pulse_us(escleft,1500);
		rc_servo_send_pulse_us(escright,1500);
		loop_rate.sleep();
	}

	printf("- Foils off - \r\n");
	//sleep(3);

	printf("- Calibrating motors . . . -\r\n");

	/*
	//calibrates BlueRobotics ESC's
	printf("- Sent low . . . -\r\n");
  delay = 2.0000;
	for(double i = 0.0000; i < delay; i+=(1.0000/hz)) {
		rc_servo_send_pulse_us(escleft,1100);
		rc_servo_send_pulse_us(escright,1100);
		loop_rate.sleep();
	}

	//sleep(2);
	printf("- Sent high . . . -\r\n");
	delay = 2.0000;
	for(double i = 0.0000; i < delay; i+=(1.0000/hz)) {
		rc_servo_send_pulse_us(escleft,1900);
		rc_servo_send_pulse_us(escright,1900);
		loop_rate.sleep();
	}
	*/

	//resends to 0 angular velocity
	printf("waits 5 seconds\n");
	delay = 5.0000;
	for(double i = 0.0000; i < delay; i+=(1.0000/hz)) {
		rc_servo_send_pulse_us(escleft,1500);
		rc_servo_send_pulse_us(escright,1500);
		loop_rate.sleep();
	}

	printf("- Sent high . . . -\r\n");
	//HF params
	printf("wiggles hydrofoils\n");
	delay = 1.000;
	int center = 1475, gain = 10;

	for(double i = 0.0000; i < delay; i+=(1.0000/hz)) {
		rc_servo_send_pulse_us(servoLeft,center+gain);
		rc_servo_send_pulse_us(servoRight,center-gain);
		loop_rate.sleep();
	}
	for(double i = 0.0000; i < delay; i+=(1.0000/hz)) {
		rc_servo_send_pulse_us(servoLeft,center-gain);
		rc_servo_send_pulse_us(servoRight,center+gain);
		loop_rate.sleep();
	}
	printf("laxes hydrofoils\n");
	for(double i = 0.0000; i < delay; i+=(1.0000/hz)) {
		rc_servo_send_pulse_us(servoLeft,center);
		rc_servo_send_pulse_us(servoRight,center);
		loop_rate.sleep();
	}

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

		//Flips battery servo back and forth
		if(pwm > pwmMax || pwm < pwmMin) {
			inc*=(-1);
		}
		rc_servo_send_pulse_us(battery,pwm);
		pwm+=inc;
		//Moves VBS forward and backwards with 10 second increments
		rc_servo_send_pulse_us(vbs,center+vbsgain*subcount);

		//Wiggles hydrofoils for 5 secoond increments
		rc_servo_send_pulse_us(servoLeft,center+gain);
		rc_servo_send_pulse_us(servoRight,center-gain);
		rc_servo_send_pulse_us(escleft,center+gain);
		rc_servo_send_pulse_us(escright,center+gain);

		count++; // if count%500 == 0, 5 seconds
		subcount++;
		if(count%500 == 0) { //every 5 seconds takes an action
			if(subcount >= 2) {
				subcount = 0;
				vbsgain*=(-1);
			}
			gain*=(-1);
		}
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
		/*
		//Measures accelerometer and prints data
		rc_mpu_read_accel(&data);
		rc_mpu_read_gyro(&data);
		rc_mpu_read_mag(&data);
		rc_mpu_read_temp(&data);

		imu_data.linear_acceleration.x = data.accel[0];
		imu_data.linear_acceleration.y = data.accel[1];
		imu_data.linear_acceleration.z = data.accel[2];
		imu_data.angular_velocity.x = data.gyro[0];
		imu_data.angular_velocity.y = data.gyro[1];
		imu_data.angular_velocity.z = data.gyro[2];
		imu_data.header.stamp =  ros::Time::now();
		imu_mag.magnetic_field.x = data.mag[0];
		imu_mag.magnetic_field.y = data.mag[1];
		imu_mag.magnetic_field.z = data.mag[2];
		imu_mag.header.stamp =  ros::Time::now();
		//publish the data
		imu_pub.publish(imu_data);
		mag_pub.publish(imu_mag);
		*/
		ros::spinOnce();
		loop_rate.sleep();
	}
}
