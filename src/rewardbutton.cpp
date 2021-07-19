#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/kd.h>
#include <linux/keyboard.h>
#include <sys/ioctl.h>

struct input_event {
      struct timeval time;
      unsigned short type;
      unsigned short code;
      unsigned int value; 
};

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
  
  int rd;
  struct input_event ev[16];

  while (ros::ok())
  {
    // Read keycodes from the input file
    rd = read(fd, ev, sizeof(ev) * sizeof(input_event));

    for (int i = 0; i < rd / sizeof(input_event); i++) {
      ROS_INFO("keycode %d %d %d", 		
        ev[i].type,
        ev[i].code,
        ev[i].value);
		}

    // Publish the latest value of the reward button
    std_msgs::Float32 reward;
    reward.data = 0.0;
    reward_pub.publish(reward);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}