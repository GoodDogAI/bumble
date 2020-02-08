#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <fstream>

#include <math.h>

#define MAX_SPEED 50.0
static volatile float vel_left, vel_right;


/**
 * This code just drive the robot around randomly, for the purposes of initializing reinforcement learning training.
 */
int main(int argc, char **argv)
{
  /**
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "randomwalk");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);


  ros::Rate loop_rate(20);



  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 2 * MAX_SPEED * double(rand())/double(RAND_MAX) - MAX_SPEED;
    msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;

    cmd_vel_pub.publish(msg);

  ROS_INFO("Sending random direction %f, %f, %f,  Ang %f, %f, %f",
    msg.linear.x, msg.linear.y, msg.linear.z,
    msg.angular.x, msg.angular.y, msg.angular.z);



    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}