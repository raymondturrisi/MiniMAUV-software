/*
* @file mauv_heartbeat.cpp
* @Node name: mauv_heartbeat
* @Publishing to:
*  std_msgs::int32 heartbeat/beat

* @Subscribing to:
*   none
*
* @description
* 	Maintains timing for MAUV vehicle. Cycles at 0.001 s (1 ms) so beats of
* the vehicle serves as a global timer for various processes.
* i.e. beats%500 = every 0.5 s, beats%1000 = every 1 s
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	9/13/2020, rt: Created mauv_heartbeat.cpp
*
*
*
*/

#include <sstream>
#include <signal.h>
#include "ros/ros.h"
#include <std_msgs/Float32.h>

//shuts down if ctrl^c
void MySigintHandler(int sig)
{
  ROS_INFO("Shutting down: mauv_heartbeat");
  ros::shutdown();
  return;
}

int main(int argc, char **argv) {

  // initialize a ros node (name used in ros context)
  ros::init(argc, argv, "mauv_heartbeat");
  // create a handle of the node (name used in the file)
  ros::NodeHandle heartbeat;
  signal(SIGINT, MySigintHandler);

  ros::Publisher heartbeat_pub = heartbeat.advertise<std_msgs::Float32>("heartbeat/beat", 1);
  ros::Rate loop_rate(1);

  std_msgs::Float32 beat;
  beat.data = 0.000;

  while(ros::ok()) {
    //starts at 0 beats
    heartbeat_pub.publish(beat);

    //every milliseconds increments the number of beats
    beat.data+=1;

    //passes to ros
    ros::spinOnce();

    //sleeps for 1 millisecond
    loop_rate.sleep();
  }
  return 0;
}
