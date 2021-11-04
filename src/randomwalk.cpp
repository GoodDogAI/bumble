#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "bumble/HeadCommand.h"

#include <iostream>
#include <fstream>
#include <random>
#include <math.h>


float external_reward = 0.0f;


// Called when you receive a reward message from an external controller
void rewardButtonCallback(const std_msgs::Float32& rew)
{
  if (rew.data != external_reward)
    ROS_INFO("Received reward button message: %f", rew.data);

  external_reward = rew.data;
}


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
  ros::NodeHandle nhPriv("~");

  std::uniform_real_distribution<> FORWARD_SPEED_DIST(nhPriv.param<float>("forward_speed_min", -0.50),
                                                      nhPriv.param<float>("forward_speed_max", 0.50));

  std::uniform_real_distribution<> ANGULAR_RATE_DIST(nhPriv.param<float>("angular_speed_min", -0.50),
                                                     nhPriv.param<float>("angular_speed_max", 0.50));

  std::uniform_real_distribution<> PITCH_DIST(nhPriv.param<float>("pitch_angle_min", -30.0),
                                              nhPriv.param<float>("pitch_angle_max", 30.0));

  std::uniform_real_distribution<> YAW_DIST(nhPriv.param<float>("yaw_angle_min", -30.0),
                                            nhPriv.param<float>("yaw_angle_max", 30.0));


  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher head_pub = n.advertise<geometry_msgs::Twist>("head_cmd", 1);

  ros::Subscriber sub = n.subscribe("/reward_button", 1, rewardButtonCallback);


  ros::Rate loop_rate(20);

  ros::Time recalc_means = ros::Time::now();
  ros::Duration recalc_freq = ros::Duration(nhPriv.param<float>("change_frequency_secs", 1.0));

  float forward_mean = 0, angular_mean = 0;
  float head_pitch = 0, head_yaw = 0;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());

  while (ros::ok())
  {
    //Every so often, pick a new mean/variance for the movement parameters
    if (ros::Time::now() - recalc_means > recalc_freq && external_reward >= 0.0f) {
      forward_mean = FORWARD_SPEED_DIST(gen);
      angular_mean = ANGULAR_RATE_DIST(gen);
      head_pitch = PITCH_DIST(gen);
      head_yaw = YAW_DIST(gen);

      recalc_means = ros::Time::now();
    } 
    else if (external_reward < 0.0f) {
      // Stop external motion if you got a negative reward
      forward_mean = 0.0f;
      angular_mean = 0.0f;

      head_pitch = 0.0f;
      head_yaw = 0.0f;

      recalc_means = ros::Time::now();
    }

    geometry_msgs::Twist msg;
    msg.linear.x = forward_mean;
    msg.angular.z = angular_mean;
    cmd_vel_pub.publish(msg);

    bumble::HeadCommand head_msg;
    head_msg.cmd_angle_pitch = head_pitch;
    head_msg.cmd_angle_yaw = head_yaw;
    head_pub.publish(head_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}