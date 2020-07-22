#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include <boost/make_shared.hpp>

#include "NvInfer.h"


#include <iostream>
#include <fstream>
#include <random>
#include <math.h>

#define INPUT_BINDING_NAME "images"
#define OUTPUT_BINDING_NAME "output"

#define MAX_SPEED 0.50

#define TILT_ID 10
#define PAN_ID 11

using namespace nvinfer1;

ros::Time last_image_received;

template <typename T>
struct TrtDestroyer
{
    void operator()(T* t)
    {
        t->destroy();
    }
};

template <typename T>
using TrtUniquePtr = std::unique_ptr<T, TrtDestroyer<T>>;

class Logger : public ILogger           
 {
     void log(Severity severity, const char* msg) override
     {
         // suppress info-level messages
         if (severity != Severity::kINFO)
             std::cout << msg << std::endl;
     }
 } gLogger;

ICudaEngine* loadEngine(const std::string& engine, int DLACore, std::ostream& err)
{
    std::ifstream engineFile(engine, std::ios::binary);
    if (!engineFile)
    {
        err << "Error opening engine file: " << engine << std::endl;
        return nullptr;
    }

    engineFile.seekg(0, engineFile.end);
    long int fsize = engineFile.tellg();
    engineFile.seekg(0, engineFile.beg);

    std::vector<char> engineData(fsize);
    engineFile.read(engineData.data(), fsize);
    if (!engineFile)
    {
        err << "Error loading engine file: " << engine << std::endl;
        return nullptr;
    }

    TrtUniquePtr<IRuntime> runtime{createInferRuntime(gLogger)};
    if (DLACore != -1)
    {
        runtime->setDLACore(DLACore);
    }

    return runtime->deserializeCudaEngine(engineData.data(), fsize, nullptr);
}

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

  std::cout << "Creating Inference engine and execution context" << std::endl;
  ICudaEngine* engine = loadEngine("/home/robot/yolov5s.tensorrt", 0, std::cout);
  IExecutionContext *context = engine->createExecutionContext();
  std::cout << "Created" << std::endl;

  for (int ib = 0; ib < engine->getNbBindings(); ib++) {
   std::cout << engine->getBindingName(ib) << " isInput: " << engine->bindingIsInput(ib) << std::endl; 
  }

  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    msg.linear.x = forward_mean;
    msg.angular.z = angular_mean;

    cmd_vel_pub.publish(msg);

    ros::Time start = ros::Time::now();

    std::cout << "A" << std::endl;

    int inputIndex = engine->getBindingIndex(INPUT_BINDING_NAME);
    int outputIndex = engine->getBindingIndex(OUTPUT_BINDING_NAME);
    void* buffers[2];
    // buffers[inputIndex] = inputbuffer;
    // buffers[outputIndex] = outputBuffer;
    // context->enqueue(batchSize, buffers, stream, nullptr);

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

    debug_img_pub.publish(yolo_msg);
                          

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}