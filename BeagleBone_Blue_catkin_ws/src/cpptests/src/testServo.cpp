//Control servomotor in BBB using RCLibrary
#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <signal.h>
#include <chrono>
#include <unistd.h>

extern "C" {
	#include "roboticscape.h"
	#include "rc/time.h"
	#include "rc/adc.h"
	#include "rc/dsm.h"
	#include "rc/servo.h"
	}

#define servo 1


int main() {
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

	while(1) {
		for(int pwm = pwmMin; pwm < pwmMax; pwm+= range/100) {
			rc_servo_send_pulse_us(servo,pwm);
			/*if(rc_servo_send_pulse_us(servo,pwm) == -1) {
				std::cout << "failed at: " << pwm << std::endl;
			}*/
			sleep(0.2);
  		}
	}
}
