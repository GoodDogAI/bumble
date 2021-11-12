#include "ros/ros.h"

// Bring in my package's API, which is what I'm testing
#include "librosa.h"
#include "npy.hpp"

// Bring in gtest
#include <gtest/gtest.h>

#include <iostream>

using namespace std;

// Test that you can take a raw mel spectrogram
TEST(TestSuite, testCleanForwardFeatures)
{
  int sr = 16000;
  int n_fft = 512;
  int n_hop = 160;
  string window = "hann";
  bool center = true;
  string pad_mode = "reflect";
  float power = 2.f;
  int n_mel = 64;
  int fmin = 0;
  int fmax = 8000;
  int n_mfcc = 64;
  bool norm = true;
  int type = 2;

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


  // Duplicate the AmpltiudeToDb behavior of torchaudio, mode "power" with a topDB of 80
  float topDB = 80.f; 
  float multiplier = 10.0f;
  float amin = 1e-10f;
  float db_multiplier = log10(max(amin, 1.0f));
  float amax = 0.0f;

  for (int i = 0; i < input_data.size(); i++)
  {
    input_data[i] = multiplier * log10(input_data[i] < amin ? amin : input_data[i]);
    input_data[i] -= multiplier * db_multiplier;

    if (input_data[i] > amax)
    {
      amax = input_data[i];
    }
  }

  for (int i = 0; i < input_data.size(); i++)
  {
    input_data[i] = max(input_data[i], amax - topDB);
  }



  // compute mel spectrogram
  std::vector<std::vector<float>> mfccs = librosa::Feature::mfcc(input_data, sr,
                                                                 n_fft, n_hop, window, center, pad_mode,
                                                                 power, n_mel, fmin, fmax,
                                                                 n_mfcc, norm, type);

  std::cout << "result shape" << mfccs.size() << " " << mfccs[0].size() << std::endl;
  std::cout << "expected output shape" << output_shape[0] << " " << output_shape[1] << std::endl;

  for (auto i = 0; i < mfccs[0].size(); i++)
  {
    std::cout << mfccs[0][i] << std::endl;
  }

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}