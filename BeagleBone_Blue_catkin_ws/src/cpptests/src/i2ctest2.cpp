/**
 * testing the bar sensor with i2c base code in ROS node
 */

#include <sstream>
#include "ros/ros.h"
extern "C"  // required when building C code
{
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
#include <rc/i2c.h>
#include <rc/time.h>
}
#include <std_msgs/Float64.h>
#include "Bar30.h"

float freshwater = 997.00;
float seawater = 1027.00;



void MySigintHandler(int sig)
{
  //shuts down if ctrl ^c
  ROS_INFO("shutting down!");
  rc_i2c_close(1);
  ros::shutdown();
}

MS5837 sensor;

int main(int argc, char** argv) {
  //  uint8_t re[17];
  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "Bar30");
  // create a handle of the node (name used in the file)
  ros::NodeHandle bar30;
  signal(SIGINT, MySigintHandler);
  // create a Publisher and publish a string topic named heading
  sensor.init();
 // sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(freshwater); // kg/m^3 (freshwater, seawater)

  ros::Publisher pressure_pub = bar30.advertise<std_msgs::Float64>("pressure", 100);
  ros::Publisher temperature_pub = bar30.advertise<std_msgs::Float64>("temperature", 100);
  // set the frequency. It should be conbined with spinOnce().
  ros::Rate loop_rate(10);

  // set the serial number of the output data
  int count = 0;
	//std_msgs::Float64 voltage, current,power;
	std_msgs::Float64 pressure, temperature;
  while (ros::ok()) {

	  //voltage.data = 1.25*int((vo[0]<<8)|(vo[1]))/1000; //combine the high bits and the low bits;
	  //current.data = 1.25*int((cu[0]<<8)|(cu[1]))/1000;
    //power.data=10*int((po[0]<<8)|(po[1]))/1000;
	//sensor.readTestCase();
	sensor.read_pressure();
	pressure.data = sensor.pressure(1.00f);
	temperature.data = sensor.temperature();

    ROS_INFO("pressure:%f\n", pressure.data);
    ROS_INFO("temperature:%f\n", temperature.data);
    pressure_pub.publish(pressure);
    temperature_pub.publish(pressure);
    //// get the voltage and current data and publish it by the publisher
    // circulate at the set rate
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
