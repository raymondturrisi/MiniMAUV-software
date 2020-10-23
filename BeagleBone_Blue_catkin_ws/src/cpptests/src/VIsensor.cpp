/**
 * 该例程将发布chatter话题，消息类型String
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
#define INA260_I2CADDR_DEFAULT 0x40  ///< INA260 default i2c address
#define INA260_REG_CURRENT 0x01      ///< Current measurement register (signed) in mA
#define INA260_REG_BUSVOLTAGE 0x02   ///< Bus voltage measurement register in mV
#define INA260_REG_POWER        0x03 ///< Power calculation register in mW
#define INA260_REG_MASK_ENABLE 0x06  ///< Interrupt/Alert setting and checking register
#define INA260_REG_ALERT_LIMIT 0x07  ///< Alert limit value register
#define INA260_REG_MFG_UID 0xFE      ///< Manufacturer ID Register
#define INA260_REG_DIE_UID 0xFF      ///< Die ID and Revision Register
void MySigintHandler(int sig)
{
  //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
  ROS_INFO("shutting down!");
  rc_i2c_close(1);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  //  uint8_t re[17];
  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "VIsensor");
  // create a handle of the node (name used in the file)
  ros::NodeHandle vi;
  signal(SIGINT, MySigintHandler);
  // create a Publisher and publish a string topic named heading
  ros::Publisher voltage_pub = vi.advertise<std_msgs::Float64>("voltage", 1000);
  ros::Publisher current_pub = vi.advertise<std_msgs::Float64>("current", 1000);
  ros::Publisher power_pub = vi.advertise<std_msgs::Float64>("power", 1000);
  // set the frequency. It should be conbined with spinOnce().
  ros::Rate loop_rate(50);
  ////init i2c
  rc_i2c_init(1, INA260_I2CADDR_DEFAULT);

  //// get the voltage and current data and publish it by the publisher
  /* 	rc_i2c_read_bytes(1, INA260_REG_DIE_UID, 16, re);
      rc_i2c_read_bytes(1, INA260_REG_MFG_UID, 16, re);
        rc_i2c_read_bytes(1, INA260_REG_MASK_ENABLE, 16, re);
          rc_i2c_read_bytes(1, INA260_REG_ALERT_LIMIT, 16, re); */
  // set the serial number of the output data
  int count = 0;
  while (ros::ok())
  {
    uint8_t cu[2], vo[2], po[2];
    //uint8_t t;

    //// get the voltage and current data and publish it by the publisher
    // rc_i2c_set_device_address(1, INA260_I2CADDR_DEFAULT);
    //rc_i2c_read_byte(1, INA260_REG_BUSVOLTAGE, &t);
    //ROS_INFO("byte:%d\n", t);
    rc_i2c_read_bytes(1, INA260_REG_CURRENT, 2, cu);
    // rc_i2c_set_device_address(1, INA260_I2CADDR_DEFAULT);
    rc_i2c_read_bytes(1, INA260_REG_BUSVOLTAGE, 2, vo);
    rc_i2c_read_bytes(1, INA260_REG_POWER, 2, po);
	//ROS_INFO("voltage0:%d\n voltage1:%d\n", int(vo[0]),int(vo[1]));
    std_msgs::Float64 voltage, current,power;
	  voltage.data = 1.25*int((vo[0]<<8)|(vo[1]))/1000; //combine the high bits and the low bits;
	  current.data = 1.25*int((cu[0]<<8)|(cu[1]))/1000;
    power.data=10*int((po[0]<<8)|(po[1]))/1000;
    ROS_INFO("current:%f\n", current.data);
    ROS_INFO("voltage:%f\n", voltage.data);
    voltage_pub.publish(voltage);
    current_pub.publish(current);
    power_pub.publish(power);
    //// get the voltage and current data and publish it by the publisher
    // circulate at the set rate
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
