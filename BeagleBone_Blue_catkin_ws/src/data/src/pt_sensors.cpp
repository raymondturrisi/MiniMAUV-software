/*
* @file pt_sensors.cpp
* @Node name: pt_sensors
* @Publishing to:
*
* @Subscribing to:
*
*
* @description
* 	Reports external pressure and temperature with the Bar30 sensor, and the
* internal pressure and temperature with the built in barometer on the BBBl
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	- James Strawson | js (adopted his original code for the internal pt sensor)
* 	-
* @log
* 	9/12/2020: Adopted test code from cpptests/src/i2ctest2.cpp and implemented
* code used in RCLibrary/rc_test_bmp.c by James Strawson | rt
*
*
*
*/

#include <sstream>
#include "ros/ros.h"
extern "C"  // required when building C code
{
  #include <stdio.h>
  #include <getopt.h>
  #include <signal.h>
  #include <stdlib.h>  // for atoi() and exit()
  #include <rc/math/filter.h>
  #include <rc/i2c.h>
  #include <rc/time.h>
  #include <rc/bmp.h>
}
#include <std_msgs/Float32.h>
#include "Bar30.h"

//********Definitions used in rc_test_bmp.c
// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in rc/bmp.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_16
// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER BMP_FILTER_OFF
// our own low pass filter
#define ORDER           2
#define CUTOFF_FREQ     2.0f    // 2rad/s, about 0.3hz
#define BMP_CHECK_HZ    25
#define DT              1.0f/BMP_CHECK_HZ
uint8_t MS5837_30BA = 0;
uint8_t MS5837_02BA = 1;
uint8_t model = MS5837_02BA;

void MySigintHandler(int sig)
{
  //shuts down if ctrl ^c
  ROS_INFO("Shutting down: pt_sensors");
  rc_i2c_close(1);
  rc_bmp_power_off();
  ros::shutdown();
}

//Initializes the MS5837 chip in the Bar30
MS5837 sensor;

int main(int argc, char** argv) {
  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "pt_sensors");

  // create a handle of the node (name used in the file)
  ros::NodeHandle pt_sensors;

  //vars for water densities, kg/m^3
  float freshwater = 997.00;
  float seawater = 1027.00;

  rc_bmp_data_t data, data_last;
  double filtered_alt;
  rc_filter_t lowpass = RC_FILTER_INITIALIZER;

  // create the lowpass filter
  if(rc_filter_butterworth_lowpass(&lowpass,ORDER, DT, CUTOFF_FREQ)) return -1;
  // init barometer and read in first data
  if(rc_bmp_init(OVERSAMPLE, INTERNAL_FILTER))                       return -1;
  if(rc_bmp_read(&data))                                             return -1;

  // prefill low pass filter
  rc_filter_prefill_inputs(&lowpass, data.alt_m);
  rc_filter_prefill_outputs(&lowpass, data.alt_m);


  //forward declares the signal handler
  //signal(SIGINT, MySigintHandler);

  sensor.init();
  sensor.setModel(model);
  sensor.setFluidDensity(freshwater); // kg/m^3 (freshwater, seawater)

  //Creates publishers
  ros::Publisher epressure_pub = pt_sensors.advertise<std_msgs::Float32>("sensors/epressure", 2);
  ros::Publisher etemperature_pub = pt_sensors.advertise<std_msgs::Float32>("sensors/etemperature", 2);
  ros::Publisher ipressure_pub = pt_sensors.advertise<std_msgs::Float32>("sensors/ipressure", 2);
  ros::Publisher itemperature_pub = pt_sensors.advertise<std_msgs::Float32>("sensors/itemperature", 2);

  // set the frequency. It should be conbined with spinOnce(). 25 Hz for BMP on BBBl
  ros::Rate loop_rate(25);

  //std_msgs::Float32 voltage, current,power;
  std_msgs::Float32 epressure, etemperature, ipressure, itemperature;

  while (ros::ok()) {

    ////for troubleshooting Bar30
    //sensor.readTestCase();

    //See include/bar30.h and bar30.cpp for more details (reads pressure AND temperature of bar30)
    sensor.read_pressure();
    epressure.data = sensor.pressure(100.0f); //conversion for Pa
    etemperature.data = sensor.temperature();

    //Reads internal pressure and temperature, if bad read, uses last read [returns 0 on good read, -1 on bad read]
    if(rc_bmp_read(&data)) {
      data = data_last;
    }

    ipressure.data = data.pressure_pa/1000;
    itemperature.data = data.temp_c;

    //ROS_INFO("epressure:%f\n", epressure.data);
    //ROS_INFO("etemperature:%f\n", etemperature.data);
    //ROS_INFO("ipressure:%f\n", ipressure.data);
    //ROS_INFO("itemperature:%f\n", itemperature.data);

    //publishes all data
    epressure_pub.publish(epressure);
    etemperature_pub.publish(etemperature);
    ipressure_pub.publish(ipressure);
    itemperature_pub.publish(itemperature);

    //saves recent bmp data readings
    data_last = data;

    //passes to ros and pauses
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
