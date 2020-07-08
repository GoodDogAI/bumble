#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include <boost/make_shared.hpp>

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

torch::Tensor image_input = torch::zeros({1, 3, 480, 640}); // N C H W format

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
                                      {img->height, img->width},
                                      torch::TensorOptions().dtype(torch::kUInt8));

  // Cast the float in the range 0 to 1                                      
  temp_tensor = temp_tensor.to(torch::kFloat) / 255.0;
  
  // Set the dimensions, NCHW format is the standard for pytorch, also expand out to three color dimensions
  temp_tensor = temp_tensor.view({1, 1, img->height, img->width}).expand({-1, 3, -1, -1});

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
  ros::Publisher debug_img_pub = n.advertise<sensor_msgs::Image>("yolo_img", 2);
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

    auto yolo_output = yolov5.forward({image_input.to(device)}).toTuple();
    auto yolo_features = yolo_output->elements()[0].toTensor().cpu();
    std::cout << "Yolo ran in " << ros::Time::now() - start << std::endl;

    std::cout << "input shape " << image_input.sizes() << std::endl;
    std::cout << "yolo shape " << yolo_features.sizes() << std::endl;      

    float threshold = 0.6;
    auto frame_access = yolo_features.accessor<float,3>();

    auto tagged_output_image = image_input.clone();

    auto frame_bboxes = torch::zeros( {yolo_features.sizes()[1], 4});
    frame_bboxes.index_put_({torch::indexing::Slice(), 0}, (yolo_features.index({0, torch::indexing::Slice(), 0}) - yolo_features.index({0, torch::indexing::Slice(), 2}) / 2.0));
    frame_bboxes.index_put_({torch::indexing::Slice(), 1}, (yolo_features.index({0, torch::indexing::Slice(), 1}) - yolo_features.index({0, torch::indexing::Slice(), 3}) / 2.0));
    frame_bboxes.index_put_({torch::indexing::Slice(), 2}, (yolo_features.index({0, torch::indexing::Slice(), 0}) + yolo_features.index({0, torch::indexing::Slice(), 2}) / 2.0));
    frame_bboxes.index_put_({torch::indexing::Slice(), 3}, (yolo_features.index({0, torch::indexing::Slice(), 1}) + yolo_features.index({0, torch::indexing::Slice(), 3}) / 2.0));
    frame_bboxes = frame_bboxes.round().to(torch::kInt32);

    auto bbox_access = frame_bboxes.accessor<int,2>();

    for(int32_t i = 0; i < yolo_features.sizes()[1]; i++) {
       float base_confidence = frame_access[0][i][4];

       if (base_confidence >= threshold) {
          for(int32_t class_index = 0; class_index < 80; class_index++) {
            if( frame_access[0][i][class_index + 5] * base_confidence >= threshold) {
              std::cout << "Saw class " << yolo_class_names[class_index] << 
                  " at " << bbox_access[i][0] << "x" << bbox_access[i][1] << " to " << bbox_access[i][2] << "x" << bbox_access[i][3] << std::endl;

              std::cout << frame_access[0][i][0] << std::endl;

              tagged_output_image.index_put_({0, 1, torch::indexing::Slice(bbox_access[i][1], bbox_access[i][3]), torch::indexing::Slice(bbox_access[i][0], bbox_access[i][2])}, 1.0);
              
              //This works to just put a green square in a constant place
              //tagged_output_image.index_put_({0, 1, torch::indexing::Slice(50, 150), torch::indexing::Slice(200, 300)}, 1.0);
            }
          }
       }
    }      


    //Copies an image from a tensor back into a ROS Image message and publishes it
    sensor_msgs::ImagePtr yolo_msg = boost::make_shared<sensor_msgs::Image>();
    yolo_msg->header = std_msgs::Header();
    yolo_msg->width = 640;
    yolo_msg->height = 480;
    yolo_msg->encoding = "rgb8";
    yolo_msg->is_bigendian = false;
    yolo_msg->step = 640 * 3;
    size_t size = yolo_msg->step * yolo_msg->height;
    yolo_msg->data.resize(size);
    memcpy((char*)(&yolo_msg->data[0]), (tagged_output_image * 255).contiguous(torch::MemoryFormat::ChannelsLast).to(torch::kUInt8).data_ptr(), size);

    debug_img_pub.publish(yolo_msg);
                          

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}