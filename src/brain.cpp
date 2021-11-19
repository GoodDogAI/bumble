#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "bumble/HeadFeedback.h"
#include "bumble/HeadCommand.h"
#include "bumble/ODriveFeedback.h"

#include <boost/make_shared.hpp>

#include "NvOnnxParser.h"
#include "NvInfer.h"
#include "tensorrt_common/buffers.h"

#include <algorithm>
#include <deque>
#include <iostream>

#include <fstream>
#include <random>
#include <math.h>

#define INPUT_BINDING_NAME "images"
#define DETECTION_BINDING_NAME "output"
#define INTERMEDIATE_LAYER_BINDING_NAME "361"

#define MLP_INPUT_BINDING_NAME "yolo_intermediate"
#define MLP_INPUT_SIZE 990
//#define MLP_INPUT_SIZE 5308
#define MLP_OUTPUT_BINDING_NAME "actions"
#define MLP_OUTPUT_STDDEV_BINDING_NAME "stddev"

#define OBJECT_DETECTION_THRESHOLD 0.60
#define DEFAULT_VBUS 14.0f

const float SPEED_MIN = -0.5;
const float SPEED_MAX = 0.5;
const float ROTATION_MIN = -0.5;
const float ROTATION_MAX = 0.5;
const float PAN_MIN = -1;
const float PAN_MAX = 1;
const float TILT_MIN = -1;
const float TILT_MAX = 1;

using namespace nvinfer1;

sensor_msgs::ImageConstPtr image_ptr;

ros::Time last_image_received;
sensor_msgs::Imu last_head_orientation;
bumble::ODriveFeedback last_odrive_feedback;
bumble::HeadFeedback last_head_feedback;
float last_vbus = DEFAULT_VBUS;

geometry_msgs::Twist last_internal_cmd_vel;
geometry_msgs::Twist last_external_cmd_vel;
geometry_msgs::Twist stopped_cmd_vel;
float last_internal_pan, last_internal_tilt;

bool external_reward_connected = false;
float external_reward = 0.0f;
bool use_external_cmd_vel = false;


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
     void log(Severity severity, const char* msg) noexcept override
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

std::shared_ptr<nvinfer1::ICudaEngine> buildAndCacheEngine(const std::string& onnx_path, int32_t mlp_input_history_size = -1) {
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

    // Set an optimiziation profile for any dynamic LSTM dimensions
    if (mlp_input_history_size > 0) {
        ROS_INFO("Adding optimization dimensions for MLP_INPUT_BINDING_NAME");

        nvinfer1::IOptimizationProfile* profile = builder->createOptimizationProfile();
        profile->setDimensions(MLP_INPUT_BINDING_NAME, OptProfileSelector::kMIN, Dims3(1, 1, MLP_INPUT_SIZE));
        profile->setDimensions(MLP_INPUT_BINDING_NAME, OptProfileSelector::kOPT, Dims3(1, mlp_input_history_size, MLP_INPUT_SIZE));
        profile->setDimensions(MLP_INPUT_BINDING_NAME, OptProfileSelector::kMAX, Dims3(1, mlp_input_history_size, MLP_INPUT_SIZE));

        config->addOptimizationProfile(profile);
    }

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


void detectBBoxes(const float* detectionOut, Dims dims) {
    // Expected dims as [1, N, 85] standard detection bbox from a yolo network
    // Format of the 85-tensor is [center x, center y, width, height,
    //                            box_probability, person_probability, bicycle_probability..., toothbrush_probability]

    for (int batch = 0; batch < dims.d[0]; batch++)
    {
        for (int row = 0; row < dims.d[1]; row++)
        {
            float box_prob = detectionOut[batch * dims.d[1] * dims.d[2] +
                                          row * dims.d[2] +
                                          4];

            if (box_prob < OBJECT_DETECTION_THRESHOLD)
                continue;

            for (int obj_class = 0; obj_class < CLASS_NUM; obj_class++)
            {
                float class_prob = detectionOut[batch * dims.d[1] * dims.d[2] +
                                                row * dims.d[2] +
                                                5 + obj_class];

                class_prob = class_prob * box_prob;

                if (class_prob >= OBJECT_DETECTION_THRESHOLD)
                {
                    std::cout << "Found class " << yolo_class_names[obj_class] << std::endl;

                    float cx = detectionOut[batch * dims.d[1] * dims.d[2] +
                                            row * dims.d[2] +
                                            0];

                    float cy = detectionOut[batch * dims.d[1] * dims.d[2] +
                                            row * dims.d[2] +
                                            1];

                    float w = detectionOut[batch * dims.d[1] * dims.d[2] +
                                           row * dims.d[2] +
                                           2];

                    float h = detectionOut[batch * dims.d[1] * dims.d[2] +
                                           row * dims.d[2] +
                                           3];

                    float x = cx - w / 2.0f;
                    float y = cy - h / 2.0f;

                    std::cout << x << ", " << y << "  dims " << w << "x" << h << std::endl;                                           
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

  image_ptr = img;

  last_image_received = ros::Time::now();
}

// Called when you receive a reward message from an external controller
void rewardButtonCallback(const std_msgs::Float32& rew)
{
  if (rew.data != external_reward)
    ROS_INFO("Received reward button message: %f", rew.data);

  external_reward = rew.data;
}

// Called when the reward button wants to override the robot's heading
void rewardButtonCmdVelCallback(const geometry_msgs::Twist& rew_cmd_vel)
{
    last_external_cmd_vel = rew_cmd_vel;
}

// Called when the reward button wants to override the robot's heading
void rewardButtonOverrideCmdVelCallback(const std_msgs::Bool& override)
{
    use_external_cmd_vel = override.data;
}

// Called when the reward button app either connects or disconnects from the robot
void rewardButtonConnectedCallback(const std_msgs::Bool& override)
{
    external_reward_connected = override.data;
}


// Called when you receive a head current state message
void headFeedbackCallback(const bumble::HeadFeedback& msg)
{
    last_head_feedback = msg;
}

// Called when you receive an accelerometer message from the real sense camera
void accelCallback(const sensor_msgs::Imu& msg) 
{
    last_head_orientation.linear_acceleration.x = msg.linear_acceleration.x;
    last_head_orientation.linear_acceleration.y = msg.linear_acceleration.y;
    last_head_orientation.linear_acceleration.z = msg.linear_acceleration.z;
}

// Called when you receive an gyro message from the real sense camera
void gyroCallback(const sensor_msgs::Imu& msg) 
{
    last_head_orientation.angular_velocity.x = msg.angular_velocity.x;
    last_head_orientation.angular_velocity.y = msg.angular_velocity.y;
    last_head_orientation.angular_velocity.z = msg.angular_velocity.z;
}

// Called you when receive informatiom about the current state of the motors via the Odrive interface
void oDriveCallback(const bumble::ODriveFeedback& msg)
{
    last_odrive_feedback = msg;
}

// Called when you get a vbus reading
void vbusCallback(const std_msgs::Float32& msg) 
{
    last_vbus = msg.data;
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
  ros::Publisher head_pub = n.advertise<bumble::HeadCommand>("head_cmd", 1);

  // Publishes intermedia debug messages for verifying proper operation. 
  // Typically you would not enable this, because the bag recording will have trouble keeping up
  ros::Publisher yolo_intermediate_pub = n.advertise<std_msgs::Float32MultiArray>("yolo_intermediate", 2);
  ros::Publisher debug_img_pub = n.advertise<sensor_msgs::Image>("yolo_img", 2);
  
  ros::Publisher brain_inputs_pub = n.advertise<std_msgs::Float32MultiArray>("brain_inputs", 2);
  ros::Publisher brain_outputs_pub = n.advertise<std_msgs::Float32MultiArray>("brain_outputs", 2);

  ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 1, cameraImageCallback);
  
  ros::Subscriber reward_sub = n.subscribe("/reward_button", 1, rewardButtonCallback);
  ros::Subscriber reward_connected_sub = n.subscribe("/reward_button_connected", 1, rewardButtonConnectedCallback);
  ros::Subscriber reward_override_cmd_vel_pub = n.subscribe("/reward_button_override_cmd_vel", 1, rewardButtonOverrideCmdVelCallback);
  ros::Subscriber reward_cmd_vel_pub = n.subscribe("/reward_button_cmd_vel", 1, rewardButtonCmdVelCallback);

  //ros::Subscriber dynamixel_state_sub = n.subscribe("/dynamixel_workbench/dynamixel_state", 1, dynamixelStateCallback);
  ros::Subscriber accel_sub = n.subscribe("/camera/accel/sample", 1, accelCallback);
  ros::Subscriber gyro_sub = n.subscribe("/camera/gyro/sample", 1, gyroCallback);
  ros::Subscriber odrive_feedback_sub = n.subscribe("/odrive_feedback", 1, oDriveCallback);
  ros::Subscriber head_feedback_sub = n.subscribe("/head_feedback", 1, headFeedbackCallback);
  ros::Subscriber vbus_sub = n.subscribe("/vbus", 1, vbusCallback);

  ros::Rate loop_rate(nhPriv.param<float>("update_rate", 8.0f));
  int32_t mlp_input_history_size = nhPriv.param<int32_t>("mlp_input_history_size", 1);

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());

  float pan_min = n.param<float>("/simplebgc/yaw_angle_min", -30.0);
  float pan_max = n.param<float>("/simplebgc/yaw_angle_max", 30.0);
  float tilt_min = n.param<float>("/simplebgc/pitch_angle_min", -30.0);
  float tilt_max = n.param<float>("/simplebgc/pitch_angle_max", 30.0);

  float sampling_scale = nhPriv.param<float>("sampling_scale", 1.0f);

  // Build the inference engine if it doesn't exist yet
  std::string yolo_model_path = nhPriv.param<std::string>("tensorrt_yolo", "yolo.onnx");
  std::string brain_model_path = nhPriv.param<std::string>("tensorrt_brain", "mlpsac.onnx");

  //Load and print stats on the YOLO inference engine
  std::cout << "Creating YOLO Inference engine and execution context" << std::endl;
  std::shared_ptr<nvinfer1::ICudaEngine> mEngine = loadEngine(yolo_model_path + ".engine");

  if (mEngine == nullptr) {
    mEngine = buildAndCacheEngine(yolo_model_path, 1);

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
    mlpEngine = buildAndCacheEngine(brain_model_path, mlp_input_history_size);

    if (mlpEngine == nullptr) {
      std::cout << "Error loading SAC MLP engine" << std::endl;
      return 1;
    }
  }

  IExecutionContext *mlpContext = mlpEngine->createExecutionContext();

  // Set the variable input time dimension to the preferred amount
  mlpContext->setBindingDimensions(mlpEngine->getBindingIndex(MLP_INPUT_BINDING_NAME),
                                   nvinfer1::Dims3(1, mlp_input_history_size, MLP_INPUT_SIZE));


  for (int ib = 0; ib < mlpEngine->getNbBindings(); ib++) {
   std::cout << "MLPSAC " << mlpEngine->getBindingName(ib) << " isInput: " << mlpEngine->bindingIsInput(ib) 
    << " Dims: " << mlpContext->getBindingDimensions(ib) << " dtype: " << (int)mlpEngine->getBindingDataType(ib) <<  std::endl; 
  }

  assert(mlpContext->getBindingDimensions(mlpEngine->getBindingIndex(MLP_INPUT_BINDING_NAME)).nbDims == 3);
  assert(mlpContext->getBindingDimensions(mlpEngine->getBindingIndex(MLP_INPUT_BINDING_NAME)).d[0] == 1);
  assert(mlpContext->getBindingDimensions(mlpEngine->getBindingIndex(MLP_INPUT_BINDING_NAME)).d[1] == mlp_input_history_size);
  assert(mlpContext->getBindingDimensions(mlpEngine->getBindingIndex(MLP_INPUT_BINDING_NAME)).d[2] == MLP_INPUT_SIZE);

  // Build a buffer to hold a history of input, in case the implementation allows for an LSTM, etc
  std::deque<std::vector<float>> mlp_input_history;

  // Prefill the buffer with zero entry vectors
  for (int32_t i = 0; i < mlp_input_history_size; i++) {
      mlp_input_history.push_back(std::vector<float>(MLP_INPUT_SIZE, 0.0));
  }

  samplesCommon::BufferManager yoloBuffers(mEngine, 0);
  samplesCommon::BufferManager mlpBuffers(mlpEngine, 0, mlpContext);  // Need to pass in context because it has the dynamic input sizes for the LSTM

  cudaStream_t stream;
  CHECK(cudaStreamCreate(&stream));

  while (ros::ok())
  {
    ros::Time start = ros::Time::now();

    // Skip the image processing step if there is no image
    if (image_ptr) {
        float* hostInputBuffer = static_cast<float*>(yoloBuffers.getHostBuffer(INPUT_BINDING_NAME));

        // Only MONO8 encoding is supported for now
        assert(image_ptr->encoding == sensor_msgs::image_encodings::RGB8);
        
        //NCHW format is offset_nchw(n, c, h, w) = n * CHW + c * HW + h * W + w
        for(int i=0; i < image_ptr->height; i++) {
            for(int j=0; j < image_ptr->width; j++) {
                hostInputBuffer[0 * image_ptr->height * image_ptr->width + i * image_ptr->width + j] = image_ptr->data[i * image_ptr->step + j * 3 + 0] / 255.0;
                hostInputBuffer[1 * image_ptr->height * image_ptr->width + i * image_ptr->width + j] = image_ptr->data[i * image_ptr->step + j * 3 + 1] / 255.0;
                hostInputBuffer[2 * image_ptr->height * image_ptr->width + i * image_ptr->width + j] = image_ptr->data[i * image_ptr->step + j * 3 + 2] / 255.0;
            }
        }
    
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

        //Read back the final classifications
        const float* detectionOut = static_cast<const float*>(yoloBuffers.getHostBuffer(DETECTION_BINDING_NAME));
        nvinfer1::Dims detectionDims = mEngine->getBindingDimensions(mEngine->getBindingIndex(DETECTION_BINDING_NAME));
        detectBBoxes(detectionOut, detectionDims);

        
        // Prepare the intermediate outputs to use later
        const float* intermediateOut = static_cast<const float*>(yoloBuffers.getHostBuffer(INTERMEDIATE_LAYER_BINDING_NAME));
        nvinfer1::Dims intermediateDims = mEngine->getBindingDimensions(mEngine->getBindingIndex(INTERMEDIATE_LAYER_BINDING_NAME));

        //Publish the intermediate yolo array if desired, caution this takes a lot of extra cpu saving the bags
        if (nhPriv.param<bool>("log_network_outputs", false)) {
            std_msgs::Float32MultiArray yolo_intermediate;
            write_intermediate_outputs(yolo_intermediate, intermediateOut, intermediateDims);
            yolo_intermediate_pub.publish(yolo_intermediate);

            //debug_img_pub.publish(cv_ptr->toImageMsg());
        }

        // Build the input observation space
        std::vector<float> mlp_input = std::vector<float>(MLP_INPUT_SIZE, 0.0);

        // Normalized pan and tilt, current orientation
        mlp_input[0] = normalize_output(last_head_feedback.cur_angle_yaw, pan_min, pan_max);
        mlp_input[1] = normalize_output(last_head_feedback.cur_angle_pitch, tilt_min, tilt_max);

        // Head gyro
        mlp_input[2] = last_head_orientation.angular_velocity.x / 10.0f;
        mlp_input[3] = last_head_orientation.angular_velocity.y / 10.0f;
        mlp_input[4] = last_head_orientation.angular_velocity.z / 10.0f;

        // Head accel
        mlp_input[5] = last_head_orientation.linear_acceleration.x / 10.0f;
        mlp_input[6] = last_head_orientation.linear_acceleration.y / 10.0f;
        mlp_input[7] = last_head_orientation.linear_acceleration.z / 10.0f;

        // ODrive feedback
        mlp_input[8] = last_odrive_feedback.motor_vel_actual_0;
        mlp_input[9] = last_odrive_feedback.motor_vel_actual_1;

        // Vbus
        mlp_input[10] = last_vbus - DEFAULT_VBUS;
    
        //Copy every 157st element into the SAC model
        #if MLP_INPUT_SIZE == 990
            for (int i = 0; i < MLP_INPUT_SIZE - 11; i++) {
                mlp_input[11 + i] = intermediateOut[i * 157];
            }
        #elif MLP_INPUT_SIZE == 5308
            for (int i = 0; i < MLP_INPUT_SIZE - 11; i++) {
                mlp_input[11 + i] = intermediateOut[i * 29];
            }
        #endif

        // Put the newly constructed observation into the mlp_input_history buffer
        mlp_input_history.push_back(mlp_input);

        while (mlp_input_history.size() > mlp_input_history_size) {
            mlp_input_history.pop_front();
        }

        // Copy the mlp_input_history buffer
        float* mlpInputBuffer = static_cast<float*>(mlpBuffers.getHostBuffer(MLP_INPUT_BINDING_NAME));
       
        for (int i = 0; i < mlp_input_history.size(); i++) {
            memcpy(mlpInputBuffer + i * MLP_INPUT_SIZE, mlp_input_history[i].data(), MLP_INPUT_SIZE * sizeof(float));
        }
    
        // Asynchronously copy data from host input buffers to device input buffers
        mlpBuffers.copyInputToDeviceAsync(stream);

        // Asynchronously enqueue the inference work
        if (!mlpContext->enqueue(1, mlpBuffers.getDeviceBindings().data(), stream, nullptr))
        {
            return false;
        }
        // Asynchronously copy data from device output buffers to host output buffers
        mlpBuffers.copyOutputToHostAsync(stream);

        // Wait for the work in the stream to complete
        cudaStreamSynchronize(stream);

        const float* mlpOutput = static_cast<const float*>(mlpBuffers.getHostBuffer(MLP_OUTPUT_BINDING_NAME));

        float speed = mlpOutput[0],
              ang = mlpOutput[1],
              pan = mlpOutput[2],
              tilt = mlpOutput[3];

        const float* actionsStdDev = static_cast<const float*>(mlpBuffers.getHostBuffer(MLP_OUTPUT_STDDEV_BINDING_NAME));
        speed = std::clamp(speed + normal_dist(gen) * actionsStdDev[0] * sampling_scale, SPEED_MIN, SPEED_MAX);
        ang = std::clamp(ang + normal_dist(gen) * actionsStdDev[1] * sampling_scale, ROTATION_MIN, ROTATION_MAX);
        pan = std::clamp(pan + normal_dist(gen) * actionsStdDev[2] * sampling_scale, PAN_MIN, PAN_MAX);
        tilt = std::clamp(tilt + normal_dist(gen) * actionsStdDev[3] * sampling_scale, TILT_MIN, TILT_MAX);

        pan = output_from_normalized(pan, pan_min, pan_max);
        tilt = output_from_normalized(tilt, tilt_min, tilt_max);
        
        std::cout << "speed: " << speed << "   ang: " << ang << 
                    "   pan: " << pan << "   tilt: " << tilt<<
                    "     stddevs: "  << actionsStdDev[0] << " " << actionsStdDev[1] << " " << actionsStdDev[2] << " " << actionsStdDev[3] <<  std::endl;



        last_internal_cmd_vel.linear.x = speed;
        last_internal_cmd_vel.angular.z = ang;
        last_internal_pan = pan;
        last_internal_tilt = tilt;

        // Publish the brain IOs so we can make sure they match up with what we are passing in during training
        std_msgs::Float32MultiArray brain_inputs_msg;
        brain_inputs_msg.data = std::vector<float>(mlpInputBuffer, mlpInputBuffer + (mlp_input_history.size() * MLP_INPUT_SIZE));
        brain_inputs_pub.publish(brain_inputs_msg);

        std_msgs::Float32MultiArray brain_outputs_msg;
        brain_outputs_msg.data = std::vector<float>(mlpOutput, 
                                                    mlpOutput + mlpContext->getBindingDimensions(mlpEngine->getBindingIndex(MLP_OUTPUT_BINDING_NAME)).d[1]);
        brain_outputs_pub.publish(brain_outputs_msg);

        // Clear out the image pointer, so we don't reprocess this image anymore
        image_ptr = nullptr;
        std::cout << "Took " << ros::Time::now() - start << std::endl;          
    }
    else if (last_image_received > ros::Time(0) && ros::Time::now() - last_image_received > ros::Duration(1.0)) {
        ROS_ERROR("No image received in the last second, exiting");
        ros::shutdown();
    }

    float pan, tilt;

    if (use_external_cmd_vel) {
        cmd_vel_pub.publish(last_external_cmd_vel);
        pan = (pan_min + pan_max) / 2;
        tilt = (tilt_min + tilt_max) / 2;
    }
    else {
        if (!external_reward_connected) {
            cmd_vel_pub.publish(stopped_cmd_vel);
            pan = (pan_min + pan_max) / 2;
            tilt = tilt_max;
            ROS_INFO("Connect the rewardbutton app on your phone to start robot");
        }
        else if (external_reward < 0.0f) {
            cmd_vel_pub.publish(stopped_cmd_vel);
            pan = (pan_min + pan_max) / 2;
            tilt = tilt_min;
            ROS_INFO("Punish button pressed, robot stopped");
        }
        else {
            cmd_vel_pub.publish(last_internal_cmd_vel);
            pan = last_internal_pan;
            tilt = last_internal_tilt;
        }
    }

    // Write the output to the head
    bumble::HeadCommand head_msg;
    head_msg.cmd_angle_pitch = tilt;
    head_msg.cmd_angle_yaw = pan;
    head_pub.publish(head_msg);

    ros::spinOnce();
    loop_rate.sleep();

    if(loop_rate.cycleTime() > ros::Time::now() - start )
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", 
                        1.0 / loop_rate.expectedCycleTime().toSec(),
                        loop_rate.cycleTime().toSec());
  }


  return 0;
}