#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include <boost/make_shared.hpp>

#include <cv_bridge/cv_bridge.h>

#include "NvInfer.h"
#include "tensorrt_common/buffers.h"

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

cv_bridge::CvImagePtr cv_ptr;
ros::Time last_image_received;

inline float Logist(float data){ return 1.0f / (1.0f + expf(-data)); };

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

std::shared_ptr<nvinfer1::ICudaEngine> loadEngine(const std::string& engine, int DLACore, std::ostream& err)
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

    return std::shared_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(engineData.data(), fsize, nullptr),
                                                  samplesCommon::InferDeleter());
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


  cv_ptr = cv_bridge::toCvCopy(img, "rgb8");

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
  std::shared_ptr<nvinfer1::ICudaEngine> mEngine = loadEngine("/home/robot/yolov5s.tensorrt", 0, std::cout);
  IExecutionContext *context = mEngine->createExecutionContext();
  std::cout << "Created" << std::endl;
  std::cout << "Implicit batch: " << mEngine->hasImplicitBatchDimension() << std::endl;

  for (int ib = 0; ib < mEngine->getNbBindings(); ib++) {
   std::cout << mEngine->getBindingName(ib) << " isInput: " << mEngine->bindingIsInput(ib) 
    << " Dims: " << mEngine->getBindingDimensions(ib) << std::endl; 
  }

  samplesCommon::BufferManager buffers(mEngine, 0);

  


  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    msg.linear.x = forward_mean;
    msg.angular.z = angular_mean;

    cmd_vel_pub.publish(msg);

    ros::Time start = ros::Time::now();

    // Skip the image processing step if there is no image
    if (cv_ptr) {
        float* hostInputBuffer = static_cast<float*>(buffers.getHostBuffer(INPUT_BINDING_NAME));

        //NCHW format is offset_nchw(n, c, h, w) = n * CHW + c * HW + h * W + w
        for(int i=0; i < cv_ptr->image.rows; i++) {
            // pointer to first pixel in row
            cv::Vec3b* pixel = cv_ptr->image.ptr<cv::Vec3b>(i);

            for(int j=0; j < cv_ptr->image.cols; j++) {
                hostInputBuffer[0 * cv_ptr->image.rows * cv_ptr->image.cols + j * cv_ptr->image.rows + i] = pixel[j][0];
                hostInputBuffer[1 * cv_ptr->image.rows * cv_ptr->image.cols + j * cv_ptr->image.rows + i] = pixel[j][1];
                hostInputBuffer[2 * cv_ptr->image.rows * cv_ptr->image.cols + j * cv_ptr->image.rows + i] = pixel[j][2];
            }
        }
    
        cudaStream_t stream;
        CHECK(cudaStreamCreate(&stream));

        // Asynchronously copy data from host input buffers to device input buffers
        buffers.copyInputToDeviceAsync(stream);

        // Asynchronously enqueue the inference work
        if (!context->enqueue(1, buffers.getDeviceBindings().data(), stream, nullptr))
        {
            return false;
        }
        // Asynchronously copy data from device output buffers to host output buffers
        buffers.copyOutputToHostAsync(stream);

        // Wait for the work in the stream to complete
        cudaStreamSynchronize(stream);

        // Release stream
        cudaStreamDestroy(stream);

        //Read back the final classifications
        const float* detectionOut = static_cast<const float*>(buffers.getHostBuffer(OUTPUT_BINDING_NAME));
        nvinfer1::Dims dims = mEngine->getBindingDimensions(mEngine->getBindingIndex(OUTPUT_BINDING_NAME));

        std::cout << dims << std::endl;

        for (int c = 0; c < 3; c++) {
            for (int col = 0; col < dims.d[2]; col++) {
                for (int row = 0; row < dims.d[3]; row++) {
                    float box_prob = Logist(detectionOut[c * dims.d[2] * dims.d[3] * dims.d[4] +
                                                         col * dims.d[3] * dims.d[4] +
                                                         row * dims.d[4] +
                                                         4]);

                    if (box_prob < 0.50) 
                        continue;

                    for (int obj_class = 0; obj_class < 80; obj_class++ ){
                        float class_prob = Logist(detectionOut[c * dims.d[2] * dims.d[3] * dims.d[4] +
                                                                col * dims.d[3] * dims.d[4] +
                                                                row * dims.d[4] +
                                                                4 + obj_class]);

                        class_prob = class_prob * box_prob;                                                                

                        if (class_prob > .80) {
                            std::cout << "Found class " << yolo_class_names[obj_class] << class_prob << std::endl;
                        }                                                                
                    }
                }
            }
            
        }
        

        //Draws the inference data back into a ROS Image message and publishes it
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));       
        debug_img_pub.publish(cv_ptr->toImageMsg());
    }
              
    std::cout << "Took " << ros::Time::now() - start << std::endl;      

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}