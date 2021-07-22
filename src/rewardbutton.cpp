#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/poll.h>

#include <vector>

#define DEV_INPUT_EVENT "/dev/input"
#define EVENT_DEV_NAME "event"


static int is_event_device(const struct dirent *dir) {
	return strncmp(EVENT_DEV_NAME, dir->d_name, 5) == 0;
}


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

  // Container for all the input devices fds that we are watching
  std::vector<pollfd> input_fds;

  // Iterate over all files in /dev/input/event*, using code from evtest
  struct dirent **namelist;
	int i, ndev, devnum, match;
	char *filename;

	ndev = scandir(DEV_INPUT_EVENT, &namelist, is_event_device, alphasort);

  ROS_INFO("Available input devices: ");

	for (i = 0; i < ndev; i++)
	{
		char fname[4096];
		int fd = -1;
		char name[256] = "???";

    snprintf(fname, sizeof(fname), "%s/%s", DEV_INPUT_EVENT, namelist[i]->d_name);
		
		fd = open(fname, O_RDONLY);
		if (fd < 0) {
      ROS_WARN("Failed to open %s", fname);
      continue;
    }
    
		ioctl(fd, EVIOCGNAME(sizeof(name)), name);

    ROS_INFO("%s:  %s", fname, name);
		input_fds.push_back({fd, POLLIN, 0});

		free(namelist[i]);
	}

  
  // Ros Params for settings rewards/penalties
  int penalty_keycode = nhPriv.param<int>("penalty_keycode", KEY_ESC);
  ros::Duration penalty_duration = ros::Duration(nhPriv.param<float>("penalty_duration_secs", 1.0));
  ros::Time last_penalty;

  // Buffers for reading keyboard IO
  struct input_event ev[16];


  while (ros::ok())
  {
    int ret = poll(input_fds.data(), input_fds.size(), 0);

    for (int i = 0; i < input_fds.size(); i++) {
      if (input_fds[i].revents & POLLIN) {
        // Read keycodes from the input file
        int rd = read(input_fds[i].fd, ev, sizeof(ev) * sizeof(input_event));

        for (int i = 0; i < rd / sizeof(input_event); i++) {
          // ROS_INFO("keycode %d %d %d", ev[i].type, ev[i].code, ev[i].value);

          if (ev[i].type == EV_KEY && ev[i].code == penalty_keycode && ev[i].value == 1) {
            ROS_INFO("Penalty keycode detected");
            last_penalty = ros::Time::now();
          }
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