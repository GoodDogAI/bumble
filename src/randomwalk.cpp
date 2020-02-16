#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include <iostream>
#include <fstream>
#include <random>
#include <math.h>

#define MAX_SPEED 0.50

#define TILT_ID 10
#define PAN_ID 11

std::uniform_real_distribution<> FORWARD_SPEED_DIST(-0.50, 0.50);
std::uniform_real_distribution<> ANGULAR_RATE_DIST(-0.50, 0.50);

std::uniform_real_distribution<> PAN_DIST(350, 700);
std::uniform_real_distribution<> TILT_DIST(475, 725);

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
  ros::ServiceClient pan_tilt_client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

  ros::Rate loop_rate(20);

  ros::Time recalc_means = ros::Time::now();

  float forward_mean = 0, angular_mean = 0;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());

  while (ros::ok())
  {
    //Every so often, pick a new mean/variance for the movement parameters
      if (ros::Time::now() - recalc_means > ros::Duration(1)) {
      forward_mean = FORWARD_SPEED_DIST(gen);
      angular_mean = ANGULAR_RATE_DIST(gen);
      recalc_means = ros::Time::now();

      dynamixel_workbench_msgs::DynamixelCommand panMsg;
      panMsg.request.id = PAN_ID;
      panMsg.request.addr_name = "Goal_Position";
      panMsg.request.value = PAN_DIST(gen);
      pan_tilt_client.call(panMsg);

      dynamixel_workbench_msgs::DynamixelCommand tiltMsg;
      tiltMsg.request.id = TILT_ID;
      tiltMsg.request.addr_name = "Goal_Position";
      tiltMsg.request.value = TILT_DIST(gen);
      pan_tilt_client.call(tiltMsg);
    } 


    geometry_msgs::Twist msg;

    msg.linear.x = forward_mean;
    msg.angular.z = angular_mean;

    cmd_vel_pub.publish(msg);

  ROS_INFO("Sending random direction %f, %f, %f,  Ang %f, %f, %f",
    msg.linear.x, msg.linear.y, msg.linear.z,
    msg.angular.x, msg.angular.y, msg.angular.z);


    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}