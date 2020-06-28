#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include <torch/torch.h>
#include <torch/script.h>

#include <iostream>
#include <fstream>
#include <random>
#include <math.h>

#define MAX_SPEED 0.50

#define TILT_ID 10
#define PAN_ID 11

ros::Time last_image_received;


void cameraImageCallback(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO("Received camera image with encoding %s, width %d, height %d", 
  img->encoding.c_str(), img->width, img->height);


  last_image_received = ros::Time::now();
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
  ros::init(argc, argv, "brain");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::ServiceClient pan_tilt_client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

  ros::Subscriber sub = n.subscribe("/camera/infra2/image_rect_raw", 1, cameraImageCallback);

  ros::Rate loop_rate(20);


  float forward_mean = 0, angular_mean = 0;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());

  torch::Device device = torch::kCPU;
  if (torch::cuda::is_available()) {
    std::cout << "CUDA is available! Using the GPU." << std::endl;
    device = torch::kCUDA;
  }

  torch::jit::script::Module yolov5;
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    yolov5 = torch::jit::load("/home/robot/yolov5s.torchscript");
    std::cout << "Loaded the model" << std::endl;
    yolov5.to(device);
    std::cout << "Moved to device" << std::endl;
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    return -1;
  }

  torch::Tensor rand_input = torch::rand({1, 3, 640, 640}).to(device);


  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    msg.linear.x = forward_mean;
    msg.angular.z = angular_mean;

    cmd_vel_pub.publish(msg);

    ros::Time start = ros::Time::now();

    auto yolo_features = yolov5.forward({rand_input});
    std::cout << "Yolo ran in " << ros::Time::now() - start << std::endl;

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}