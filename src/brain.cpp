#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "mainbot/HeadFeedback.h"

#include <boost/make_shared.hpp>
#include <cv_bridge/cv_bridge.h>

#include "NvOnnxParser.h"
#include "NvInfer.h"
#include "tensorrt_common/buffers.h"

#include <algorithm>

#include <iostream>

#include <fstream>
#include <random>
#include <math.h>

#define INPUT_BINDING_NAME "images"
#define OUTPUT_BINDING_NAME1 "output"
#define OUTPUT_BINDING_NAME2 "427"
#define OUTPUT_BINDING_NAME3 "446"
#define INTERMEDIATE_LAYER_BINDING_NAME "300"

#define MLP_INPUT_BINDING_NAME "yolo_intermediate"
#define MLP_OUTPUT_BINDING_NAME "actions"
#define ACTIONS_STDDEV_BINDING_NAME "stddev"

#define OBJECT_DETECTION_THRESHOLD 0.60

#define DEFAULT_STEPS_PER_DEGREE (1024/300.0)

const float SPEED_MIN = -0.5;
const float SPEED_MAX = 0.5;
const float ROTATION_MIN = -0.5;
const float ROTATION_MAX = 0.5;
const float PAN_MIN = -1;
const float PAN_MAX = 1;
const float TILT_MIN = -1;
const float TILT_MAX = 1;

using namespace nvinfer1;

cv_bridge::CvImagePtr cv_ptr;
ros::Time last_image_received;
float external_reward = 0.0f;

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

std::shared_ptr<nvinfer1::ICudaEngine> loadEngine(const std::string& engine_path)
{
    std::ifstream engineFile(engine_path, std::ios::binary);
    if (!engineFile)
    {
        ROS_ERROR("Error opening engine file: %s", engine_path.c_str());
        return nullptr;
    }

    engineFile.seekg(0, engineFile.end);
    long int fsize = engineFile.tellg();
    engineFile.seekg(0, engineFile.beg);

    std::vector<char> engineData(fsize);
    engineFile.read(engineData.data(), fsize);
    if (!engineFile)
    {
        ROS_ERROR("Error loading engine file: %s", engine_path.c_str());
        return nullptr;
    }

    TrtUniquePtr<nvinfer1::IRuntime> runtime{createInferRuntime(gLogger)};

    // Not using DLACores in our inference scenarios
    // if (DLACore != -1)
    // {
    //     runtime->setDLACore(DLACore);
    // }

    return std::shared_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(engineData.data(), fsize, nullptr),
                                                  samplesCommon::InferDeleter());
}

std::shared_ptr<nvinfer1::ICudaEngine> buildAndCacheEngine(const std::string& onnx_path) {
    auto builder = TrtUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(gLogger));
    if (!builder)
    {
        ROS_ERROR("Failed to create TensorRT builder");
        return nullptr;
    }

    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = TrtUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
    if (!network)
    {
        ROS_ERROR("Failed to create TensorRT network");
        return nullptr;
    }

    auto config = TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
    if (!config)
    {
        ROS_ERROR("Failed to create TensorRT builder config");
        return nullptr;
    }

    auto parser = TrtUniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, gLogger));
    if (!parser)
    {
        ROS_ERROR("Failed to create TensorRT ONNX parser");
        return nullptr;
    }

    auto parsed = parser->parseFromFile(onnx_path.c_str(), static_cast<int>(sample::gLogger.getReportableSeverity()));
    if (!parsed)
    {
        ROS_ERROR("Failed to parse ONNX network %s", onnx_path.c_str());
        return nullptr;
    }

    config->setMaxWorkspaceSize(16_MiB);

    // CUDA stream used for profiling by the builder.
    cudaStream_t profileStream;
    CHECK(cudaStreamCreate(&profileStream));
    config->setProfileStream(profileStream);

    std::shared_ptr<nvinfer1::ICudaEngine> engine(builder->buildEngineWithConfig(*network, *config),
                                                  samplesCommon::InferDeleter());
    if (!engine)
    {
        ROS_ERROR("Failed to build TensorRT engine from config");
        return nullptr;
    }

    auto serialized = TrtUniquePtr<nvinfer1::IHostMemory>(engine->serialize());
    if (!serialized)
    {
        ROS_ERROR("Failed to serialize TensorRT engine");
        return nullptr;
    }

    // Save serialized engine to disk.
    std::ofstream engineFile(onnx_path + ".engine", std::ios::binary);
    if (!engineFile)
    {
        ROS_ERROR("Failed to open engine file %s", onnx_path.c_str());
        return nullptr;
    }
    engineFile.write(reinterpret_cast<char*>(serialized->data()), serialized->size());
    engineFile.close();

    cudaStreamDestroy(profileStream);

    return engine;
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

std::normal_distribution<float> normal_dist(0.0, 1.0);

std::vector<std::string> yolo_class_names = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign",
  "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
  "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", 
  "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
  "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
  "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
  "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};


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

void write_intermediate_outputs(std_msgs::Float32MultiArray &yolo_intermediate, 
                                const float* intermediateOut, nvinfer1::Dims intermediateDims) {   
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
}

float normalize_output(float val, float low, float high) {
    return (val - low) * 2 / (high - low) - 1;
}

float output_from_normalized(float val_normalized, float low, float high) {
    return (val_normalized + 1) * (high - low) / 2 + low;
}

 
// Called when a new camera message is received.
// Moves the image into an easy to work with OpenCV format.
void cameraImageCallback(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO("Received camera image with encoding %s, width %d, height %d", 
            img->encoding.c_str(), img->width, img->height);

  cv_ptr = cv_bridge::toCvCopy(img, "rgb8");
  last_image_received = ros::Time::now();
}

// Called when you receive a reward message from an external controller
void rewardButtonCallback(const std_msgs::Float32& rew)
{
  if (rew.data != external_reward)
    ROS_INFO("Received reward button message: %f", rew.data);

  external_reward = rew.data;
}

/**
 * This code runs the YOLO backbone network, and then gets its drive commands from an MLP network that was trained offline.
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
  ros::NodeHandle nhPriv("~");

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Publisher feedback_pub = n.advertise<mainbot::HeadFeedback>("head_feedback", 5);

  // Publishes intermedia debug messages for verifying proper operation. 
  // Typically you would not enable this, because the bag recording will have trouble keeping up
  ros::Publisher yolo_intermediate_pub = n.advertise<std_msgs::Float32MultiArray>("yolo_intermediate", 2);
  ros::Publisher debug_img_pub = n.advertise<sensor_msgs::Image>("yolo_img", 2);

  ros::ServiceClient pan_tilt_client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

  ros::Subscriber camera_sub = n.subscribe("/camera/infra2/image_rect_raw", 1, cameraImageCallback);
  ros::Subscriber reward_sub = n.subscribe("/reward_button", 1, rewardButtonCallback);

  ros::Rate loop_rate(20);

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());

  float pan_min = n.param<float>("/pan_tilt/pan_min_angle", 0.0) * n.param<float>("/pan_tilt/pan_steps_per_degree", DEFAULT_STEPS_PER_DEGREE);
  float pan_max = n.param<float>("/pan_tilt/pan_max_angle", 90.0) * n.param<float>("/pan_tilt/pan_steps_per_degree", DEFAULT_STEPS_PER_DEGREE);
  float tilt_min = n.param<float>("/pan_tilt/tilt_min_angle", 0.0) * n.param<float>("/pan_tilt/tilt_steps_per_degree", DEFAULT_STEPS_PER_DEGREE);
  float tilt_max = n.param<float>("/pan_tilt/tilt_max_angle", 90.0) * n.param<float>("/pan_tilt/tilt_steps_per_degree", DEFAULT_STEPS_PER_DEGREE);

  float sampling_scale = nhPriv.param<float>("sampling_scale", 1.0f);

  // Build the inference engine if it doesn't exist yet
  std::string yolo_model_path = nhPriv.param<std::string>("tensorrt_yolo", "yolo.onnx");
  std::string brain_model_path = nhPriv.param<std::string>("tensorrt_brain", "mlpsac.onnx");

  //Load and print stats on the YOLO inference engine
  std::cout << "Creating YOLO Inference engine and execution context" << std::endl;
  std::shared_ptr<nvinfer1::ICudaEngine> mEngine = loadEngine(yolo_model_path + ".engine");

  if (mEngine == nullptr) {
    mEngine = buildAndCacheEngine(yolo_model_path);

    if (mEngine == nullptr) {
      std::cout << "Error loading YOLO engine" << std::endl;
      return 1;
    }
  }

  IExecutionContext *context = mEngine->createExecutionContext();

  for (int ib = 0; ib < mEngine->getNbBindings(); ib++) {
   std::cout << "YOLO " << mEngine->getBindingName(ib) << " isInput: " << mEngine->bindingIsInput(ib) 
    << " Dims: " << mEngine->getBindingDimensions(ib) << " dtype: " << (int)mEngine->getBindingDataType(ib) <<  std::endl; 
  }


  std::cout << "Creating SAC MLP Inference engine and execution context" << std::endl;
  std::shared_ptr<nvinfer1::ICudaEngine> mlpEngine = loadEngine(brain_model_path + ".engine");

  if (mlpEngine == nullptr) {
    mlpEngine = buildAndCacheEngine(brain_model_path);

    if (mlpEngine == nullptr) {
      std::cout << "Error loading SAC MLP engine" << std::endl;
      return 1;
    }
  }

  IExecutionContext *mlpContext = mlpEngine->createExecutionContext();

  for (int ib = 0; ib < mlpEngine->getNbBindings(); ib++) {
   std::cout << "MLPSAC " << mlpEngine->getBindingName(ib) << " isInput: " << mlpEngine->bindingIsInput(ib) 
    << " Dims: " << mlpEngine->getBindingDimensions(ib) << " dtype: " << (int)mlpEngine->getBindingDataType(ib) <<  std::endl; 
  }

  samplesCommon::BufferManager yoloBuffers(mEngine, 0);
  samplesCommon::BufferManager mlpBuffers(mlpEngine, 0);

  while (ros::ok())
  {
    // Skip the image processing step if there is no image
    if (cv_ptr) {
        ros::Time start = ros::Time::now();

        float* hostInputBuffer = static_cast<float*>(yoloBuffers.getHostBuffer(INPUT_BINDING_NAME));

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
        yoloBuffers.copyInputToDeviceAsync(stream);

        // Asynchronously enqueue the inference work
        if (!context->enqueue(1, yoloBuffers.getDeviceBindings().data(), stream, nullptr))
        {
            return false;
        }
        // Asynchronously copy data from device output buffers to host output buffers
        yoloBuffers.copyOutputToHostAsync(stream);

        // Wait for the work in the stream to complete
        cudaStreamSynchronize(stream);

        // Release stream
        cudaStreamDestroy(stream);

        //Read back the final classifications
        const float* detectionOut1 = static_cast<const float*>(yoloBuffers.getHostBuffer(OUTPUT_BINDING_NAME1));
        nvinfer1::Dims dims1 = mEngine->getBindingDimensions(mEngine->getBindingIndex(OUTPUT_BINDING_NAME1));
        detectBBoxes(detectionOut1, dims1, yolo1);

        const float* detectionOut2 = static_cast<const float*>(yoloBuffers.getHostBuffer(OUTPUT_BINDING_NAME2));
        nvinfer1::Dims dims2 = mEngine->getBindingDimensions(mEngine->getBindingIndex(OUTPUT_BINDING_NAME2));
        detectBBoxes(detectionOut2, dims2, yolo2);

        const float* detectionOut3 = static_cast<const float*>(yoloBuffers.getHostBuffer(OUTPUT_BINDING_NAME3));
        nvinfer1::Dims dims3 = mEngine->getBindingDimensions(mEngine->getBindingIndex(OUTPUT_BINDING_NAME3));
        detectBBoxes(detectionOut3, dims3, yolo3);
        
        // Prepare the intermediate outputs to use later
        const float* intermediateOut = static_cast<const float*>(yoloBuffers.getHostBuffer(INTERMEDIATE_LAYER_BINDING_NAME));
        nvinfer1::Dims intermediateDims = mEngine->getBindingDimensions(mEngine->getBindingIndex(INTERMEDIATE_LAYER_BINDING_NAME));

        //Publish the intermediate yolo array if desired, caution this takes a lot of extra cpu saving the bags
        if (nhPriv.param<bool>("log_network_outputs", false)) {
            std_msgs::Float32MultiArray yolo_intermediate;
            write_intermediate_outputs(yolo_intermediate, intermediateOut, intermediateDims);
            yolo_intermediate_pub.publish(yolo_intermediate);

            debug_img_pub.publish(cv_ptr->toImageMsg());
        }

        // Run the intermediate array through the SAC model
        float* mlpInputBuffer = static_cast<float*>(mlpBuffers.getHostBuffer(MLP_INPUT_BINDING_NAME));
       
        //Copy the full intermediate layer over into the SAC model
        //memcpy(mlpInputBuffer, intermediateOut, intermediateDims.d[0] * intermediateDims.d[1] * intermediateDims.d[2] * intermediateDims.d[3]);

        //Copy every 151st element into the SAC model
        for (int i = 0; i < 1018; i++) {
            mlpInputBuffer[i] = intermediateOut[i * 151];
        }

        cudaStream_t mlpStream;
        CHECK(cudaStreamCreate(&mlpStream));

        // Asynchronously copy data from host input buffers to device input buffers
        mlpBuffers.copyInputToDeviceAsync(mlpStream);

        // Asynchronously enqueue the inference work
        if (!mlpContext->enqueue(1, mlpBuffers.getDeviceBindings().data(), mlpStream, nullptr))
        {
            return false;
        }
        // Asynchronously copy data from device output buffers to host output buffers
        mlpBuffers.copyOutputToHostAsync(mlpStream);

        // Wait for the work in the stream to complete
        cudaStreamSynchronize(mlpStream);

        // Release stream
        cudaStreamDestroy(mlpStream);

        const float* mlpOutput = static_cast<const float*>(mlpBuffers.getHostBuffer(MLP_OUTPUT_BINDING_NAME));

        float speed = mlpOutput[0],
              ang = mlpOutput[1],
              pan = mlpOutput[2],
              tilt = mlpOutput[3];

        const float* actionsStdDev = static_cast<const float*>(mlpBuffers.getHostBuffer(ACTIONS_STDDEV_BINDING_NAME));
        speed = std::clamp(speed + normal_dist(gen) * actionsStdDev[0] * sampling_scale, SPEED_MIN, SPEED_MAX);
        ang = std::clamp(ang + normal_dist(gen) * actionsStdDev[1] * sampling_scale, ROTATION_MIN, ROTATION_MAX);
        pan = std::clamp(pan + normal_dist(gen) * actionsStdDev[2] * sampling_scale, PAN_MIN, PAN_MAX);
        tilt = std::clamp(tilt + normal_dist(gen) * actionsStdDev[3] * sampling_scale, TILT_MIN, TILT_MAX);

        pan = output_from_normalized(pan, pan_min, pan_max);
        tilt = output_from_normalized(tilt, tilt_min, tilt_max);
        
        std::cout << "speed: " << speed << "   ang: " << ang << 
                    "   pan: " << pan << "   tilt: " << tilt<<
                    "     stddevs: "  << actionsStdDev[0] << " " << actionsStdDev[1] << " " << actionsStdDev[2] << " " << actionsStdDev[3] <<  std::endl;

        if (external_reward < 0.0f) {
            speed = 0.0f;
            ang = 0.0f;

            pan = (pan_min + pan_max) / 2;
            tilt = (tilt_min + tilt_max) / 2;
        }

        geometry_msgs::Twist msg;
        msg.linear.x = speed;
        msg.angular.z = ang;
        cmd_vel_pub.publish(msg);

        dynamixel_workbench_msgs::DynamixelCommand panMsg;
        panMsg.request.id = n.param<int>("/pan_tilt/pan_id", 1);
        panMsg.request.addr_name = "Goal_Position";
        panMsg.request.value = pan;
        pan_tilt_client.call(panMsg);

        dynamixel_workbench_msgs::DynamixelCommand tiltMsg;
        tiltMsg.request.id = n.param<int>("/pan_tilt/tilt_id", 2);
        tiltMsg.request.addr_name = "Goal_Position";
        tiltMsg.request.value = tilt;
        pan_tilt_client.call(tiltMsg);

        // Publish the feedback command of the pan/tilt so we can log it, otherwise ROS service parameters are not logged
        mainbot::HeadFeedback feedback_msg;
        feedback_msg.pan_command = pan;
        feedback_msg.tilt_command = tilt;
        feedback_msg.header.stamp = ros::Time::now();
        feedback_pub.publish(feedback_msg);

        // Clear out the image pointer, so we don't reprocess this image anymore
        cv_ptr = nullptr;
        std::cout << "Took " << ros::Time::now() - start << std::endl;          
    }
    else if (last_image_received > ros::Time(0) && ros::Time::now() - last_image_received > ros::Duration(1.0)) {
        ROS_ERROR("No image received in the last second, exiting");
        ros::shutdown();
    }
              
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}