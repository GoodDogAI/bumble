#include "ros/ros.h"

// Bring in my package's API, which is what I'm testing
#include "librosa.h"
// Bring in gtest
#include <gtest/gtest.h>

// Test that you can take a raw mel spectrogram
TEST(TestSuite, testCase1)
{
//       int sr = 16000;
//   int n_fft = 400;
//   int n_hop = 160;
//   std::string window = "hann";
//   bool center = false;
//   std::string pad_mode = "reflect";
//   float power = 2.f;
//   int n_mel = 40;
//   int fmin = 80;
//   int fmax = 7600;
//   int n_mfcc = 20;
//   bool norm = true;
//   int type = 2;

//     // compute mel spectrogram
//   std::vector<std::vector<float>> mels = librosa::Feature::melspectrogram(x, sr, n_fft, n_hop, window, center, pad_mode, power,n_mel, fmin, fmax);

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}