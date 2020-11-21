/*
* @file imu_mpu.cpp
* @Node name: imu_mpu
* @Publishing to:
*   /sensors/<sensor_msgs>imu/data_raw, /<sensor_msgs>imu/mag, 	/<geometry_msgs>imu/rpy
*
* @Subscribing to:
*
*
* @description
* 	Reads and publishes the onboard IMU/MPU data within the BeagleBone Blue
*
* @author | initials
* 	Mingxi Zhou, mzhou@uri.edu | mz
* @contributors | initials
* 	- Raymond Turrisi, raymond.turrisi@gmail.com | rt
* 	-
* @log
* 	11/20/2020: mz's code from low_cost_usv2020/usv_sensor/src/imu.cpp was adopted by rt
*
*
*
*/

extern "C" {
  #include "roboticscape.h"
}

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>  // for atoi() and exit()
//#include <rc/mpu.h>
//#include <rc/time.h>
#include <sstream>

#include "ros/ros.h"
//include ros message types
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <tf/tf.h>
//define I2C port for the IMU
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21
static int running = 0;

void MySigintHandler(int sig) {
  //shut things down
  rc_mpu_power_off();
  fflush(stdout);
  running=0;
  return;
}

class imu_driver {
  rc_mpu_data_t data;
  rc_mpu_config_t conf = rc_mpu_default_config();
  sensor_msgs::Imu imu_data;   //IMU data
  sensor_msgs::MagneticField imu_mag;  //IMU Mage
  geometry_msgs::Vector3 rpy_data; //roll pitch yaw data
  std_msgs::Float64 heading_data;  //heading data
  double hz=50;
public:
  imu_driver() {
    //double hz=20;
    imu_pub = nh.advertise<sensor_msgs::Imu>("/sensors/imu/data_raw", 2);
    mag_pub = nh.advertise<sensor_msgs::MagneticField>("/sensors/imu/mag", 2);
    rpy_pub = nh.advertise<geometry_msgs::Vector3>("/sensors/imu/rpy", 2);
    imu_sub = nh.subscribe("/sensor/imu/data",10,&imu_driver::imu_callback,this);
    ////configuration of the imu
    conf.i2c_bus = I2C_BUS;
    conf.enable_magnetometer = 1;	//enable magnetometer, set to 0 for no magnetometer
    conf.show_warnings =1;		//enable warning for debug

    if (rc_mpu_initialize(&data, conf)) {
      printf("rc_initialize_imu_failed\n");
      return;
    }
    //if ctrl-c we shut down things
    signal(SIGINT, MySigintHandler);
    running = 1;
    ROS_INFO("Node started");

  }

  void imu_sample() {
    ros::Rate loop_rate(hz);

    while (ros::ok() & running) {
      //read data
      if(rc_mpu_read_accel(&data)<0){
        printf("read accel data failed\n");
      }
      if(rc_mpu_read_gyro(&data)<0){
        printf("read gyro data failed\n");
      }
      if(rc_mpu_read_mag(&data)<0){
        printf("read mag data failed\n");
      }
      if(rc_mpu_read_temp(&data)<0){
        printf("read imu thermometer failed\n");
      }
      //project to the msgs
      imu_data.linear_acceleration.x = data.accel[0];
      imu_data.linear_acceleration.y = data.accel[1];
      imu_data.linear_acceleration.z = data.accel[2];
      imu_data.angular_velocity.x = data.gyro[0]*DEG_TO_RAD;
      imu_data.angular_velocity.y = data.gyro[1]*DEG_TO_RAD;
      imu_data.angular_velocity.z = data.gyro[2]*DEG_TO_RAD;
      imu_data.header.stamp =  ros::Time::now();
      imu_data.header.frame_id="imu";
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

  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf::Quaternion q(msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
      tf::Matrix3x3 m(q);
      m.getRPY(rpy_data.x, rpy_data.y, rpy_data.z); ///RPY in rad
      rpy_data.x=rpy_data.x*RAD_TO_DEG;
      rpy_data.y=rpy_data.y*RAD_TO_DEG;
      rpy_data.z=rpy_data.z*RAD_TO_DEG;
      //wrap around 0 to 360
      if(rpy_data.z<0) rpy_data.z = rpy_data.z+360;
      if(rpy_data.x<0) rpy_data.x = rpy_data.x+360;
      if(rpy_data.y<0) rpy_data.y = rpy_data.y+360;
      rpy_pub.publish(rpy_data);
    }

  private:
    ros::NodeHandle nh;
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher rpy_pub;
    ros::Subscriber imu_sub;
  };

  int main(int argc, char** argv) {
    // initialize a ros node (name used in ros context)
    ros::init(argc, argv, "imu");
    imu_driver imu;
    imu.imu_sample();
    return 0;
  }
