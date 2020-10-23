/*
* @file vi_sensors.cpp
* @Node name: VIsensors
* @Publishing to:
*   std_msgs::float32 sensors/voltage1, std_msgs::float32 sensors/current1,
*   std_msgs::float32 sensors/voltage2
* @Subscribing to:
*   none
*
* @description
* 	Reports battery voltage and current every 0.1 seconds. Uses Adafruit's CV
* sensor for voltage 1 and current 1, and BBBl's barrel jack ADC for voltage 2
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	- Jianguang Shi | js (Adopted from his original code)
* 	-
* @log
* 	9/12/2020: Adopted Adafruit Current/Voltage sensor code initially written
* by js for MiniMAUV applications. Implemented BBB onboard barrel jack voltage
* sensor | rt
*
*
*
*/

#include <sstream>
#include "ros/ros.h"
extern "C" {
  #include <stdio.h>
  #include <getopt.h>
  #include <signal.h>
  #include <stdlib.h>  // for atoi() and exit()
  #include <rc/i2c.h>
  #include <rc/time.h>
}
#include <std_msgs/Float64.h>

//i2c registers for adafruits cv sensor
#define INA260_I2CADDR_DEFAULT  0x40  ///< INA260 default i2c address
#define INA260_REG_CURRENT      0x01  ///< Current measurement register (signed) in mA
#define INA260_REG_BUSVOLTAGE   0x02  ///< Bus voltage measurement register in mV
#define INA260_REG_POWER        0x03  ///< Power calculation register in mW
#define INA260_REG_MASK_ENABLE  0x06  ///< Interrupt/Alert setting and checking register
#define INA260_REG_ALERT_LIMIT  0x07  ///< Alert limit value register
#define INA260_REG_MFG_UID      0xFE  ///< Manufacturer ID Register
#define INA260_REG_DIE_UID      0xFF  ///< Die ID and Revision Register

//shuts down if ctrl^c
void MySigintHandler(int sig)
{
  ROS_INFO("Shutting down: vi_sensors");
  rc_i2c_close(1);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  //  uint8_t re[17];
  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "VIsensors");
  // create a handle of the node (name used in the file)
  ros::NodeHandle vi;
  signal(SIGINT, MySigintHandler);
  // create a Publisher and publish a string topic named heading
  ros::Publisher voltage1_pub = vi.advertise<std_msgs::Float32>("sensors/voltage1", 2);
  ros::Publisher current1_pub = vi.advertise<std_msgs::Float32>("sensors/current1", 2);
  ros::Publisher voltage2_pub = vi.advertise<std_msgs::Float32>("sensors/voltage2", 2);
  // set the frequency. It should be conbined with spinOnce().
  ros::Rate loop_rate(10);
  ////init i2c
  rc_i2c_init(1, INA260_I2CADDR_DEFAULT);

  //Init adc converter on bbbl for barrel jack voltage reading
  rc_adc_init();

  //declares messages and variables
  std_msgs::Float64 voltage1, voltage2, current;
  uint8_t cu[2], vo[2];

  while (ros::ok()) {

    //reads current/voltage from i2c
    rc_i2c_read_bytes(1, INA260_REG_CURRENT, 2, cu);
    rc_i2c_read_bytes(1, INA260_REG_BUSVOLTAGE, 2, vo);

    //reads voltage from barrel jack
    voltage2.data = rc_adc_dc_jack();

    //converts i2c readings
    voltage1.data = 1.25*int((vo[0]<<8)|(vo[1]))/1000; //combine the high bits and the low bits;
    current1.data = 1.25*int((cu[0]<<8)|(cu[1]))/1000;

    //ROS_INFO("voltage1:%f\n", voltage1.data);
    //ROS_INFO("current1:%f\n", current1.data);

    //publishes data
    voltage1_pub.publish(voltage1);
    voltage2_pub.publish(voltage2);
    current1_pub.publish(current1);
    //passes to ros
    ros::spinOnce();

    //pauses for set rate accounting for elapsed time
    loop_rate.sleep();
  }
  return 0;
}
