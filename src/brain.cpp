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

torch::Tensor image_input = torch::zeros({1, 3, 640, 640});

std::vector<std::string> yolo_class_names = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign",
  "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
  "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", 
  "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
  "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
  "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
  "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};


void cameraImageCallback(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO("Received camera image with encoding %s, width %d, height %d", 
  img->encoding.c_str(), img->width, img->height);

  // Load the data into a tensor, not that it doesn't take ownership of the ROS message
  auto temp_tensor = torch::from_blob(const_cast<uint8_t*>(&img->data[0]), 
                                      {640, 640},
                                      torch::TensorOptions().dtype(torch::kUInt8));

  // Cast the float in the range 0 to 1                                      
  temp_tensor = temp_tensor.to(torch::kFloat) / 255.0;
  
  // Set the dimensions
  temp_tensor = temp_tensor.view({1, 1, 640, 640}).expand({-1, 3, -1, -1});
  image_input = temp_tensor;

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

//  torch::Tensor rand_input = torch::rand({1, 3, 640, 640}).to(device);


  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    msg.linear.x = forward_mean;
    msg.angular.z = angular_mean;

    cmd_vel_pub.publish(msg);

    ros::Time start = ros::Time::now();

    auto yolo_features = yolov5.forward({image_input.to(device)}).toTensorVector();
    std::cout << "Yolo ran in " << ros::Time::now() - start << std::endl;

    auto all_yolo_frames = torch::cat({yolo_features[0].cpu().view({1, -1, 85}),
                                       yolo_features[1].cpu().view({1, -1, 85}),
                                       yolo_features[2].cpu().view({1, -1, 85})}, 1);

    all_yolo_frames = all_yolo_frames.cpu();

    std::cout << "shape " << all_yolo_frames.sizes() << all_yolo_frames.device() << std::endl;      

    float threshold = 0.6;
    auto frame_access = all_yolo_frames.accessor<float,3>();

    for(int32_t i = 0; i < all_yolo_frames.sizes()[1]; i++) {
       float base_confidence = frame_access[0][i][4];

       if (base_confidence >= threshold) {
          for(int32_t class_index = 0; class_index < 80; class_index++) {
            if( frame_access[0][i][class_index + 5] * base_confidence >= threshold) {
              std::cout << "Saw class " << yolo_class_names[class_index] << std::endl;
            }
          }
       }
    }                                

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}