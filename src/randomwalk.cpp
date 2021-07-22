#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "mainbot/HeadFeedback.h"

#include <iostream>
#include <fstream>
#include <random>
#include <math.h>

#define DEFAULT_STEPS_PER_DEGREE (1024/300.0)

float external_reward = 0.0f;
int pan_id = 0;
int tilt_id = 0;

// Called when you receive a reward message from an external controller
void rewardButtonCallback(const std_msgs::Float32& rew)
{
  if (rew.data != external_reward)
    ROS_INFO("Received reward button message: %f", rew.data);

  external_reward = rew.data;
}

void sendDynamixelCommands(ros::ServiceClient &pan_tilt_client, float head_pan, float head_tilt){ 
   dynamixel_workbench_msgs::DynamixelCommand panMsg;
    panMsg.request.id = pan_id;
    panMsg.request.addr_name = "Goal_Position";
    panMsg.request.value = head_pan;
    pan_tilt_client.call(panMsg);

    dynamixel_workbench_msgs::DynamixelCommand tiltMsg;
    tiltMsg.request.id = tilt_id;
    tiltMsg.request.addr_name = "Goal_Position";
    tiltMsg.request.value = head_tilt;
    pan_tilt_client.call(tiltMsg);
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

  std::uniform_real_distribution<> PAN_DIST(n.param<float>("/pan_tilt/pan_min_angle", 0.0) * n.param<float>("/pan_tilt/pan_steps_per_degree", DEFAULT_STEPS_PER_DEGREE),
                                            n.param<float>("/pan_tilt/pan_max_angle", 90.0) * n.param<float>("/pan_tilt/pan_steps_per_degree", DEFAULT_STEPS_PER_DEGREE));

  std::uniform_real_distribution<> TILT_DIST(n.param<float>("/pan_tilt/tilt_min_angle", 0.0) * n.param<float>("/pan_tilt/tilt_steps_per_degree", DEFAULT_STEPS_PER_DEGREE),
                                             n.param<float>("/pan_tilt/tilt_max_angle", 90.0) * n.param<float>("/pan_tilt/tilt_steps_per_degree", DEFAULT_STEPS_PER_DEGREE));

  pan_id = n.param<int>("/pan_tilt/pan_id", 1);
  tilt_id = n.param<int>("/pan_tilt/tilt_id", 2);

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 2);
  ros::Publisher feedback_pub = n.advertise<mainbot::HeadFeedback>("head_feedback", 2);

  ros::Subscriber sub = n.subscribe("/reward_button", 1, rewardButtonCallback);


  ros::ServiceClient pan_tilt_client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

  ros::Rate loop_rate(20);

  ros::Time recalc_means = ros::Time::now();
  ros::Duration recalc_freq = ros::Duration(nhPriv.param<float>("change_frequency_secs", 1.0));

  float forward_mean = 0, angular_mean = 0;
  float head_pan = 0, head_tilt = 0;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());

  while (ros::ok())
  {
    //Every so often, pick a new mean/variance for the movement parameters
    if (ros::Time::now() - recalc_means > recalc_freq && external_reward >= 0.0f) {
      forward_mean = FORWARD_SPEED_DIST(gen);
      angular_mean = ANGULAR_RATE_DIST(gen);
      head_pan = PAN_DIST(gen);
      head_tilt = TILT_DIST(gen);
      recalc_means = ros::Time::now();

      sendDynamixelCommands(pan_tilt_client, head_pan, head_tilt);
    } 
    else if (external_reward < 0.0f) {
      // Stop external motion if you got a negative reward
      forward_mean = 0.0f;
      angular_mean = 0.0f;

      head_pan = (PAN_DIST.a() + PAN_DIST.b()) / 2;
      head_tilt = (TILT_DIST.a() + TILT_DIST.b()) / 2;

      recalc_means = ros::Time::now();

      sendDynamixelCommands(pan_tilt_client, head_pan, head_tilt);
    }

    geometry_msgs::Twist msg;
    msg.linear.x = forward_mean;
    msg.angular.z = angular_mean;

    cmd_vel_pub.publish(msg);

    // Publish the feedback command of the pan/tilt so we can log it, otherwise ROS service parameters are not logged
    mainbot::HeadFeedback feedback_msg;
    feedback_msg.pan_command = head_pan;
    feedback_msg.tilt_command = head_tilt;
    feedback_msg.header.stamp = ros::Time::now();
    feedback_pub.publish(feedback_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}