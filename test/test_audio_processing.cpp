#include <torch/torch.h>

#include "ros/ros.h"

// Bring in my package's API, which is what I'm testing
#include "npy.hpp"

// Bring in gtest
#include <gtest/gtest.h>

#include <iostream>

using namespace std;

// Test that you can take a raw mel spectrogram
TEST(TestSuite, testCleanForwardFeatures)
{
  // Load both the input data and expected resulting features
  vector<unsigned long> input_shape;
  bool input_order;
  vector<float> input_data;
  npy::LoadArrayFromNumpy("../../src/mainbot/test/clean_forward_input.npy", input_shape, input_order, input_data);

  std::cout << "input_shape " << input_shape[0] << std::endl;

  // Load the expected output
  vector<unsigned long> output_shape;
  bool output_order;
  vector<float> output_data;
  npy::LoadArrayFromNumpy("../../src/mainbot/test/clean_forward_features.npy", output_shape, output_order, output_data);


  torch::Tensor tensor = torch::rand({2, 3});
  std::cout << tensor << std::endl;
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}