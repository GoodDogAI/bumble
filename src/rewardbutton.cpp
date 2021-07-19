#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <fstream>
#include <random>
#include <math.h>


/**
 * This code listens to a bluetooth device such as a fob, or camera shutter, and 
 * sends a message on the reward topic when it is triggered.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rewardbutton");


  ros::NodeHandle n;
  ros::NodeHandle nhPriv("~");

  ros::Publisher reward_pub = n.advertise<std_msgs::Float32>("reward_button", 2);
  
  ros::Rate loop_rate(2);


    
 
  while (ros::ok())
  {
    // Publish the latest value of the reward button
    std_msgs::Float32 reward;
    reward.data = 0.0;
    reward_pub.publish(reward);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}