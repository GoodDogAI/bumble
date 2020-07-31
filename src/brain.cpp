#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include <boost/make_shared.hpp>
#include <cv_bridge/cv_bridge.h>

#include "NvInfer.h"
#include "tensorrt_common/buffers.h"

#include "hash/sha2_256.h"

#include <iostream>

#include <fstream>
#include <random>
#include <math.h>

#define TENSORRT_YOLO_PATH "/home/robot/yolov5s.tensorrt"
#define INPUT_BINDING_NAME "images"
#define OUTPUT_BINDING_NAME1 "output"
#define OUTPUT_BINDING_NAME2 "427"
#define OUTPUT_BINDING_NAME3 "446"
#define INTERMEDIATE_LAYER_BINDING_NAME "300"

#define MAX_SPEED 0.50
#define OBJECT_DETECTION_THRESHOLD 0.60

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

static constexpr int INPUT_H = 480;
static constexpr int INPUT_W = 640;
static constexpr int CLASS_NUM = 80;
static constexpr int CHECK_COUNT = 3;

struct YoloKernel
{
    int width;
    int height;
    float anchors[CHECK_COUNT*2];
};

static constexpr YoloKernel yolo1 = {
    INPUT_W / 32,
    INPUT_H / 32,
    {116,90,  156,198,  373,326}
};
static constexpr YoloKernel yolo2 = {
    INPUT_W / 16,
    INPUT_H / 16,
    {30,61,  62,45,  59,119}
};
static constexpr YoloKernel yolo3 = {
    INPUT_W / 8,
    INPUT_H / 8,
    {10,13,  16,30,  33,23}
};

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

void detectBBoxes(const float* detectionOut, Dims dims, YoloKernel kernel) {
    //NCHW format
    for (int c = 0; c < CHECK_COUNT; c++) {
        for (int row = 0; row < dims.d[2]; row++) {
            for (int col = 0; col < dims.d[3]; col++) {
                float box_prob = Logist(detectionOut[c * dims.d[2] * dims.d[3] * dims.d[4] +
                                                        row * dims.d[3] * dims.d[4] +
                                                        col * dims.d[4] +
                                                        4]);
                //std::cout << box_prob << std::endl;

                if (box_prob < OBJECT_DETECTION_THRESHOLD) 
                    continue;

                for (int obj_class = 0; obj_class < CLASS_NUM; obj_class++ ){
                    float class_prob = Logist(detectionOut[c * dims.d[2] * dims.d[3] * dims.d[4] +
                                                            row * dims.d[3] * dims.d[4] +
                                                            col * dims.d[4] +
                                                            5 + obj_class]);

                    class_prob = class_prob * box_prob;                                                                

                    if (class_prob >= OBJECT_DETECTION_THRESHOLD) {
                        std::cout << "Found class " << yolo_class_names[obj_class] << c << std::endl;
                        float x = (col - 0.5f + 2.0f * Logist(detectionOut[c * dims.d[2] * dims.d[3] * dims.d[4] +
                                                        row * dims.d[3] * dims.d[4] +
                                                        col * dims.d[4] +
                                                        0])) * INPUT_W / kernel.width;
                        float y = (row - 0.5f + 2.0f * Logist(detectionOut[c * dims.d[2] * dims.d[3] * dims.d[4] +
                                                        row * dims.d[3] * dims.d[4] +
                                                        col * dims.d[4] +
                                                        1])) * INPUT_H / kernel.height;
                        float width = 2.0f * Logist(detectionOut[c * dims.d[2] * dims.d[3] * dims.d[4] +
                                                        row * dims.d[3] * dims.d[4] +
                                                        col * dims.d[4] +
                                                        2]);
                        width = width * width * kernel.anchors[2*c];
                        float height = 2.0f * Logist(detectionOut[c * dims.d[2] * dims.d[3] * dims.d[4] +
                                                        row * dims.d[3] * dims.d[4] +
                                                        col * dims.d[4] +
                                                        3]);
                        height = height * height * kernel.anchors[2*c + 1];
                        std::cout << cv::Rect(x,y,width,height) << std::endl;
                        cv::rectangle(cv_ptr->image, cv::Rect(x - width / 2,y - height / 2,width,height), CV_RGB(255,0,0));
                    }                                                                
                }
            }
        }
    }
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
  ros::Publisher yolo_intermediate_pub = n.advertise<std_msgs::Float32MultiArray>("yolo_intermediate", 2);
  ros::Publisher version_pub = n.advertise<std_msgs::String>("onnx_version", 2);


  ros::ServiceClient pan_tilt_client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

  ros::Subscriber sub = n.subscribe("/camera/infra2/image_rect_raw", 1, cameraImageCallback);

  ros::Rate loop_rate(20);


  float forward_mean = 0, angular_mean = 0;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());

  std::cout << "Creating Inference engine and execution context" << std::endl;
  std::shared_ptr<nvinfer1::ICudaEngine> mEngine = loadEngine(TENSORRT_YOLO_PATH, 0, std::cout);
  IExecutionContext *context = mEngine->createExecutionContext();
  std::cout << "Created" << std::endl;
  std::cout << "Implicit batch: " << mEngine->hasImplicitBatchDimension() << std::endl;

  // Save off the hash of the engine used for future references
  std_msgs::String hash_msg = std_msgs::String();
  std::ifstream infile (TENSORRT_YOLO_PATH, std::ios_base::binary);
  Chocobo1::SHA2_256 sha2;
  //sha2.addData((const void *)&infileb[0], 10);
  sha2.finalize();
  hash_msg.data = sha2.toString();

  for (int ib = 0; ib < mEngine->getNbBindings(); ib++) {
   std::cout << mEngine->getBindingName(ib) << " isInput: " << mEngine->bindingIsInput(ib) 
    << " Dims: " << mEngine->getBindingDimensions(ib) << " dtype: " << (int)mEngine->getBindingDataType(ib) <<  std::endl; 
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
                hostInputBuffer[0 * cv_ptr->image.rows * cv_ptr->image.cols + i * cv_ptr->image.cols + j] = pixel[j][0] / 255.0;
                hostInputBuffer[1 * cv_ptr->image.rows * cv_ptr->image.cols + i * cv_ptr->image.cols + j] = pixel[j][1] / 255.0;
                hostInputBuffer[2 * cv_ptr->image.rows * cv_ptr->image.cols + i * cv_ptr->image.cols + j] = pixel[j][2] / 255.0;
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
        const float* detectionOut1 = static_cast<const float*>(buffers.getHostBuffer(OUTPUT_BINDING_NAME1));
        nvinfer1::Dims dims1 = mEngine->getBindingDimensions(mEngine->getBindingIndex(OUTPUT_BINDING_NAME1));
        detectBBoxes(detectionOut1, dims1, yolo1);

        const float* detectionOut2 = static_cast<const float*>(buffers.getHostBuffer(OUTPUT_BINDING_NAME2));
        nvinfer1::Dims dims2 = mEngine->getBindingDimensions(mEngine->getBindingIndex(OUTPUT_BINDING_NAME2));
        detectBBoxes(detectionOut2, dims2, yolo2);

        const float* detectionOut3 = static_cast<const float*>(buffers.getHostBuffer(OUTPUT_BINDING_NAME3));
        nvinfer1::Dims dims3 = mEngine->getBindingDimensions(mEngine->getBindingIndex(OUTPUT_BINDING_NAME3));
        detectBBoxes(detectionOut3, dims3, yolo3);
        
        //Draws the inference data back into a ROS Image message and publishes it
        debug_img_pub.publish(cv_ptr->toImageMsg());

        //Publish the intermediate yolo array
        std_msgs::Float32MultiArray yolo_intermediate;
        const float* intermediateOut = static_cast<const float*>(buffers.getHostBuffer(INTERMEDIATE_LAYER_BINDING_NAME));
        nvinfer1::Dims intermediateDims = mEngine->getBindingDimensions(mEngine->getBindingIndex(INTERMEDIATE_LAYER_BINDING_NAME));
        yolo_intermediate.data = std::vector<float>(intermediateOut, intermediateOut + intermediateDims.d[0] * intermediateDims.d[1] * intermediateDims.d[2] * intermediateDims.d[3]);
        yolo_intermediate.layout.data_offset = 0;
        yolo_intermediate.layout.dim.push_back(std_msgs::MultiArrayDimension());
        yolo_intermediate.layout.dim[0].label = "N";
        yolo_intermediate.layout.dim[0].size = intermediateDims.d[0];
        yolo_intermediate.layout.dim[0].stride = intermediateDims.d[0] * intermediateDims.d[1] * intermediateDims.d[2] * intermediateDims.d[3];

        yolo_intermediate.layout.dim.push_back(std_msgs::MultiArrayDimension());
        yolo_intermediate.layout.dim[1].label = "C";
        yolo_intermediate.layout.dim[1].size = intermediateDims.d[1];
        yolo_intermediate.layout.dim[1].stride = intermediateDims.d[1] * intermediateDims.d[2] * intermediateDims.d[3];

        yolo_intermediate.layout.dim.push_back(std_msgs::MultiArrayDimension());
        yolo_intermediate.layout.dim[2].label = "H";
        yolo_intermediate.layout.dim[2].size = intermediateDims.d[2];
        yolo_intermediate.layout.dim[2].stride = intermediateDims.d[2] * intermediateDims.d[3];

        yolo_intermediate.layout.dim.push_back(std_msgs::MultiArrayDimension());
        yolo_intermediate.layout.dim[3].label = "W";
        yolo_intermediate.layout.dim[3].size = intermediateDims.d[3];
        yolo_intermediate.layout.dim[3].stride = intermediateDims.d[3];

        yolo_intermediate_pub.publish(yolo_intermediate);
        version_pub.publish(hash_msg);
    }
              
    std::cout << "Took " << ros::Time::now() - start << std::endl;      

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}