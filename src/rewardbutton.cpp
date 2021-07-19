#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#include <sys/poll.h>

/**
 * This code listens to a bluetooth device such as a fob, or camera shutter, and 
 * sends a message on the reward topic when it is triggered.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rewardbutton");


  ros::NodeHandle n;
  ros::NodeHandle nhPriv("~");

  ros::Publisher reward_pub = n.advertise<std_msgs::Float32>("reward_button", 2);
  
  ros::Rate loop_rate(2);

  // Open a file description to the input device
  std::string dev_name = nhPriv.param<std::string>("device", "/dev/input/event0"); 
  int fd = open(dev_name.c_str(), O_RDONLY);
  if (fd < 0)
  {
      ROS_ERROR("Could not open input file error %d", fd);
      return 1;
  }

  ROS_INFO("Opened event input file %s", dev_name.c_str());
  
  // Ros Params for settings rewards/penalties
  int penalty_keycode = nhPriv.param<int>("penalty_keycode", KEY_ESC);
  ros::Duration penalty_duration = ros::Duration(nhPriv.param<float>("penalty_duration_secs", 1.0));
  ros::Time last_penalty;

  // Buffers for reading keyboard IO
  struct input_event ev[16];

  // Poll over open file descriptors
  struct pollfd fds[1];
  fds[0].fd = fd;
  fds[0].events = POLLIN;

  while (ros::ok())
  {
    int ret = poll(fds, sizeof(fds), 0);

    if (fds[0].revents & POLLIN) {
      // Read keycodes from the input file
      int rd = read(fd, ev, sizeof(ev) * sizeof(input_event));

      for (int i = 0; i < rd / sizeof(input_event); i++) {
        // ROS_INFO("keycode %d %d %d", ev[i].type, ev[i].code, ev[i].value);

        if (ev[i].type == EV_KEY && ev[i].code == penalty_keycode && ev[i].value == 1) {
          ROS_INFO("Penalty keycode detected");
          last_penalty = ros::Time::now();
        }
      }
    }

    // Publish the latest value of the reward button
    std_msgs::Float32 reward;
    reward.data = (ros::Time::now() - last_penalty) > penalty_duration ? 0.0 : -1.0;
    reward_pub.publish(reward);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}