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

int running = 1;

void MySigintHandler(int sig) {
	//shut things down
	rc_mpu_power_off();
	rc_servo_cleanup();
	fflush(stdout);
	running=0;
	return;
}

//updated
int main(int argc, char** argv) {

ros::init(argc,argv,"rosIMUtest");

//Configures IMU with RC_mpu
rc_mpu_data_t data;
rc_mpu_config_t conf = rc_mpu_default_config();
g_mode_t g_mode = G_MODE_DEG; // gyro default to degree mode. OR 0
a_mode_t a_mode = A_MODE_MS2; // accel default to m/s^2 OR 0
static int enable_magnetometer = 1;
static int enable_thermometer = 1;
static int enable_warnings = 1;
static int running = 1;

#define I2C_BUS 2
conf.i2c_bus = I2C_BUS;
conf.enable_magnetometer = enable_magnetometer;
conf.show_warnings = enable_warnings;

//Initializes IMU
rc_mpu_initialize(&data, conf);

//Sets up ROS messages
sensor_msgs::Imu imu_data;
sensor_msgs::MagneticField imu_mag;
geometry_msgs::Vector3 rpy_data;
std_msgs::Float64 heading_data;
double hz=50;

ros::NodeHandle nh;
ros::Publisher imu_pub;
ros::Publisher mag_pub;
ros::Publisher rpy_pub;
ros::Subscriber imu_sub;

imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 2);
mag_pub = nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 2);
rpy_pub = nh.advertise<geometry_msgs::Vector3>("/imu/rpy", 2);

ros::Rate loop_rate(hz);


while(ros::ok && running) {

	rc_servo_send_pulse_us(servo,pwm);
	pwm+=inc;

	signal(SIGINT, MySigintHandler);
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
	ros::spinOnce();
  loop_rate.sleep();
}
}
