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

#define servo 1

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

int main() {

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


int pwmMin = 1000, pwmMax = 2000;
int range = (pwmMax-pwmMin);
rc_servo_cleanup();
sleep(1);
int a = rc_servo_init();
int b = rc_servo_power_rail_en(1);

std::cout << a << '\n';
std::cout << b << '\n';

sleep(1);
std::cout << '\n';

std::cout << "started"  << '\n';
int pwm = pwmMin, inc = range/20;
	while(ros::ok) {
	//Servo control, flips back and forth
	if(pwm >= pwmMax || pwm <= pwmMin) {
		inc*=(-1);
	}
	rc_servo_send_pulse_us(servo,pwm);
	pwm+=inc;
	sleep(1);

	//Measures accelerometer and prints data
	rc_mpu_read_accel(&data);
	rc_mpu_read_gyro(&data);
	rc_mpu_read_mag(&data);
	rc_mpu_read_temp(&data);
	
	printf("A[XYZ]: %6.2f %6.2f %6.2f \n",   data.accel[0],\
                                                        data.accel[1],\
                                                        data.accel[2]);
	printf("Gy[PRY]: %6.1f %6.1f %6.1f \n",   data.gyro[0]*DEG_TO_RAD,\
                                                        data.gyro[1]*DEG_TO_RAD,\
                                                        data.gyro[2]*DEG_TO_RAD);
	printf("M[?]: %6.1f %6.1f %6.1f \n",   data.mag[0],\
                                                        data.mag[1],\
                                                        data.mag[2]);
        printf("T[C]: %4.1f \n ", data.temp);
	}
}
