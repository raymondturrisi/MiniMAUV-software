/*
* @file dtt.cpp
* @Node name: dtt
* @Publishing to:
*
* @Subscribing to:
*
*
* @description
* 	Performs calcuations and publishes data related to the
* Differential Tilting Thruster.
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	9/14/2020, rt: Adopted thruster pid control from mbed to control hydrofoils
* In future code will implement a feature which publishes esc values from
* current status to desired status slowly over a span of time to allow charge up
* and minimize vehile twist
*
*
*/

#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <signal.h>
#include <chrono>
#include <unistd.h>
#include <cmath>

extern "C" {
  #include "getopt.h"
  #include "roboticscape.h"
  #include "rc/time.h"
  #include "rc/servo.h"
}
#include "ros/ros.h"

//include ros message types
#include "std_msgs/Float32.h"

//pin literals
#define hf_Left_adcPin 1
#define hf_Right_adcPin 2


//shuts down if ctrl^c
void MySigintHandler(int sig) {
  ROS_INFO("Shutting down: differential tilting thruster");
  ros::shutdown();
  return;
}


//Respective node variables
std_msgs::Float32 pwmp_msg;
std_msgs::Float32 pwms_msg;
std_msgs::Float32 angmp_msg;
std_msgs::Float32 angms_msg;
std_msgs::Float32 ep_msg;
std_msgs::Float32 es_msg;

float hfleft_ang = 180;
float hfright_ang = 180;

//Callback function which deposits subscription variables into respective node variables
void hfleftCallback(const std_msgs::Float32::ConstPtr& msg) {
  hfleft_ang = msg->data;
}

void hfrightCallback(const std_msgs::Float32::ConstPtr& msg) {
  hfright_ang = msg->data;
}

/////////////PID callbacks
float pconst = 0.00, iconst = 0.00, dconst = 0.00;
void pCallback(const std_msgs::Float32::ConstPtr& msg) {
  pconst = msg->data;
}

void iCallback(const std_msgs::Float32::ConstPtr& msg) {
  iconst = msg->data;
}

void dCallback(const std_msgs::Float32::ConstPtr& msg) {
  dconst = msg->data;
}

int main(int argc, char **argv) {

  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "dtt");
  // create a handle of the node (name used in the file)
  ros::NodeHandle dtt;

  signal(SIGINT, MySigintHandler);

  //creates subscriber
  ros::Subscriber hfleft_ang_sub = dtt.subscribe("dtt/hfleft_ang", 1, hfleftCallback);
  ros::Subscriber hfright_ang_sub = dtt.subscribe("dtt/hfright_ang", 1, hfrightCallback);

  ros::Subscriber pconst_sub = dtt.subscribe("dtt/pconst", 1, pCallback);
  ros::Subscriber iconst_sub = dtt.subscribe("dtt/iconst", 1, iCallback);
  ros::Subscriber dconst_sub = dtt.subscribe("dtt/dconst", 1, dCallback);
  //creates publisher
  //currently does not control motor pwm, 9/14/2020 rt
  //ros::Publisher escleft_pwm_pub = dtt.advertise<std_msgs::float32>("dtt/escleft_pwm", 1);
  //ros::Publisher escright_pwm_pub = dtt.advertise<std_msgs::float32>("dtt/escright_pwm", 1);
  ros::Publisher hfleft_pwm_pub = dtt.advertise<std_msgs::Float32>("dtt/hfleft_pwm", 1);
  ros::Publisher hfright_pwm_pub = dtt.advertise<std_msgs::Float32>("dtt/hfright_pwm", 1);
  ros::Publisher hfleft_ang_read = dtt.advertise<std_msgs::Float32>("dtt/hfleft_ang_read", 1);
  ros::Publisher hfright_ang_read = dtt.advertise<std_msgs::Float32>("dtt/hfright_ang_read", 1);
  ros::Publisher hfleft_err = dtt.advertise<std_msgs::Float32>("dtt/hfleft_ang_read", 1);
  ros::Publisher hfright_err = dtt.advertise<std_msgs::Float32>("dtt/hfright_ang_read", 1);



  //ROS loop rate, 10 Hz
  ros::Rate loop_rate(200);

  //initializes local variables
  //char inputSerial[26];
  float center = 1475.0000, potErrorScale = 1.0036496;

  //float pconst = 1.00, iconst = 2.50, dconst = 0.50, gainmax = 0.00; //kv = 880.0000
  float gainmax = 0.00;
  float angsp = 180, angss = 180; //, wp = 1500, ws = 1500; currently does not control motor pwm, 9/14/2020 rt

  float pwmp = 0.0000, angmp = 0.0000;
  float pwms = 0.0000, angms = 0.0000;

  float Pp = 0.0000, Ip = 0.0000, itrglp = 0.0000, Dp = 0.0000, drvp = 0.0000, ep = 0.0000, eLastp = 0.0000,gainp = 0.0000;
  float Ps = 0.0000, Is = 0.0000, itrgls = 0.0000, Ds = 0.0000, drvs = 0.0000, es = 0.0000, eLasts = 0.0000,gains = 0.0000;

  float potavgp = 0.0000,potavg2p = 0.0000,varsump = 0.0000,stdp = 0.00000,tolp = 0.0000;
  float potavgs = 0.0000,potavg2s = 0.0000,varsums = 0.0000,stds = 0.00000,tols = 0.0000;
  int rs = 0,rp = 0;

  float potrp[10], potrs[10];
  float varp[10], vars[10];

  //fills with zeros
  for(int i = 0; i < 10; i++) {
    potrp[i] = 0; potrs[i] = 0;
    varp[i] = 0; vars[i] = 0;
  }

  //Foil alignment mechanical offsets per assembly
  float Offsetp = 0.00, Offsets = 0.00;

  float servoPot_out_vmax = 1.154286; //maximum output of the servos : ~25% volts supplied to servos

  if(rc_adc_init()) {
		ROS_INFO("ADC bus initialized\n");
	} else {
		ROS_INFO("ADC bus initialization failed\n");
	}

  while(ros::ok()) {
    //Zeros running parameters which get used throughout program
    potavgp = 0; potavgs = 0;
    potavg2p = 0; potavg2s = 0;
    varsump = 0; varsums = 0;
    rp = 0; rs = 0;

    angsp = hfleft_ang;
    angss = hfright_ang;

    for (int i = 0; i < 10; i++) {
      potrp[i] = 0.000000;
      potrs[i] = 0.000000;
      varp[i] = 0.000000;
      vars[i] = 0.000000;
    }

    //Hydrofoil PID control:
    //Takes series of readings and applies filter within specified tolerances in order to reject values before computing PID

    //Fills potentiometer read array, if a read is greater than the std tolerance, fill spot with average
    //Collects 10 reads of a specified increment, converts to angle with scale
    for (int i = 0; i < 10; i++) {
      potrp[i] = (rc_adc_read_volt(hf_Left_adcPin)*360.000*potErrorScale/servoPot_out_vmax);
      potavgp += potrp[i];

      potrs[i] = (360.00-rc_adc_read_volt(hf_Right_adcPin)*360.000*potErrorScale/servoPot_out_vmax);
      potavgs += potrs[i];

      //sleeps for 5 milliseconds
      ros::Duration(0.005).sleep();
    }

    //takes average of readings
    potavgp = potavgp/10; potavgs = potavgs/10;

    //takes variance for standard deviations
    for (int i = 0;i<10;i++) {
      varp[i] = pow((float)potrp[i] - (float)potavgp,(float)2);
      varsump += varp[i];

      vars[i] = pow((float)potrs[i] - (float)potavgs,(float)2);
      varsums += vars[i];
    }

    //computes standard deviations
    stdp = sqrt(varsump/10); stds = sqrt(varsums/10);

    //Scans values and replaces values our outside of a given tolerance, should be proportional to sample standard deviations
    tolp = stdp; tols = stds;
    for (int i = 0; i < 10;i++) {
      if ((potrp[i] >= (potavgp+tolp)) || (potrp[i] <= (potavgp-(tolp)))) {
        potrp[i] = potavgp;
        rp += 1;
      }

      if ((potrs[i] >= (potavgs+tols)) || (potrs[i] <= (potavgs-(tols)))) {
        potrs[i] = potavgs;
        rs += 1;
      }

      potavg2p += potrp[i];

      potavg2s += potrs[i];
    }

    //measured angle is corrected average
    angmp = potavg2p/10 + Offsetp; angms = potavg2s/10 + Offsets;

    //make PID class?
    ep = (angsp-angmp); es = (angss-angms);

    //P
    Pp = pconst*ep; Ps = pconst*es;

    //I
    itrglp = itrglp + ep; itrgls = itrgls + es;
    Ip = iconst*itrglp; Is = iconst*itrgls;

    //D
    drvp = ep-eLastp; drvs = es-eLasts;
    Dp = dconst*drvp; Ds = dconst*drvs;

    gainp = Pp+Ip+Dp; gains = (Ps+Is+Ds)*(-1);

    //Corrects to user specified maximum angular velocity (from observation)
    gainmax = 45.00000;
    //gain_correction(gainp, gains, gainmax)
    if (gainp >= gainmax) {
      gainp = gainmax;
    }

    if (gains >= gainmax) {
      gains = gainmax;
    }

    if (gainp <= -gainmax) {
      gainp = -gainmax;
    }

    if (gains <= -gainmax) {
      gains = -gainmax;
    }

    //sets interal equal to zero to mitigate late oscillations
    if ((angmp >= angsp+0.1) || (angmp <= angsp-0.1)) {
      itrglp = 0.000;
    }
    if ((angms >= angss+0.1) || (angms <= angss-0.1)) {
      itrgls = 0.000;
    }

    ////////////////end of filter and pid computations

    //publishes to respective topics (from mbed: fires actuators)
    pwmp = center+gainp; pwms = center+gains;

    //escleft_pwm_pub.publish(wp);
    //escright_pwm_pub.publish(ws);
    pwmp_msg.data = pwmp;
    pwms_msg.data = pwms;
    angmp_msg.data = angmp;
    angms_msg.data = angms;
    ep_msg.data = ep;
    es_msg.data = es;

    hfleft_pwm_pub.publish(pwmp_msg);
    hfright_pwm_pub.publish(pwms_msg);
    hfleft_ang_read.publish(angmp_msg);
    hfright_ang_read.publish(angms_msg);
    hfleft_err.publish(ep_msg);
    hfright_err.publish(es_msg);

    ROS_INFO("DAng_L = %f, MAng_L = %f, Gain_L = %f, pwm_L = %f\n", angsp, angmp, gainp, pwmp);
    ROS_INFO("DAng_R = %f, MAng_R = %f, Gain_R = %f, pwm_R = %f\n", angss, angms, gains, pwms);
    ROS_INFO("p = %f, i = %f, d = %f\n", pconst, iconst, dconst);
    //records current error for D computations
    eLastp = ep; eLasts = es;
    //passes to ros
    ros::spinOnce();
  }
  return 0;
}
