#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include <torch/torch.h>

#include <iostream>
#include <fstream>
#include <random>
#include <math.h>

#define MAX_SPEED 0.50

#define TILT_ID 10
#define PAN_ID 11


struct Net : torch::nn::Module {
  Net() : linear(register_module("linear", torch::nn::Linear(120, 160))) {

  }

  torch::Tensor forward(torch::Tensor input) {
    return linear(input);
  }

  torch::nn::Linear linear;
};

/**
 * This code just drive the robot around randomly, for the purposes of initializing reinforcement learning training.
 */
int main(int argc, char **argv)
{
  /**
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "brain");

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

  Net net;
torch::Device device = torch::kCPU;
if (torch::cuda::is_available()) {
  std::cout << "CUDA is available! Training on GPU." << std::endl;
  device = torch::kCUDA;
}
 torch::nn::Sequential discriminator(
  // Layer 1
  torch::nn::Conv2d(
      torch::nn::Conv2dOptions(1, 64, 4).stride(2).padding(1).bias(false)),
  torch::nn::LeakyReLU(torch::nn::LeakyReLUOptions().negative_slope(0.2)),
  // Layer 2
  torch::nn::Conv2d(
      torch::nn::Conv2dOptions(64, 128, 4).stride(2).padding(1).bias(false)),
  torch::nn::BatchNorm2d(128),
  torch::nn::LeakyReLU(torch::nn::LeakyReLUOptions().negative_slope(0.2)),
  // Layer 3
  torch::nn::Conv2d(
      torch::nn::Conv2dOptions(128, 256, 4).stride(2).padding(1).bias(false)),
  torch::nn::BatchNorm2d(256),
  torch::nn::LeakyReLU(torch::nn::LeakyReLUOptions().negative_slope(0.2)),
  // Layer 4
  torch::nn::Conv2d(
      torch::nn::Conv2dOptions(256, 1, 3).stride(1).padding(0).bias(false)),
  torch::nn::Sigmoid());

 discriminator->to(device);

  // Initialize with one run
  torch::Tensor rand_input = torch::rand({1, 1, 64, 64}).to(device);
  torch::Tensor output = discriminator->forward(rand_input);

 
  // Time a thousand test runs
  ros::Time start = ros::Time::now();

  for (int i = 0; i < 1000; i++) {
    rand_input = torch::rand({1, 1, 64, 64}).to(device);
    output = discriminator->forward(rand_input);

  }

  std::cout << "Time taken (seconds) " << std::endl;
  std::cout << ros::Time::now() - start << std::endl;

  // while (ros::ok())
  // {
  //   geometry_msgs::Twist msg;

  //   msg.linear.x = forward_mean;
  //   msg.angular.z = angular_mean;

  //   cmd_vel_pub.publish(msg);

  //   ROS_INFO("Sending random direction %f, %f, %f,  Ang %f, %f, %f",
  //   msg.linear.x, msg.linear.y, msg.linear.z,
  //   msg.angular.x, msg.angular.y, msg.angular.z);

  //   ros::spinOnce();

  //   loop_rate.sleep();
  // }


  return 0;
}