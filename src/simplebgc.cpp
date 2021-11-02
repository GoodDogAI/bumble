#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "bumble/ODriveFeedback.h"

#include <iostream>
#include <fstream>
#include <string>

// Linux headers for opening serial port connections to the ODRIVE
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <math.h>


ros::Time last_received;
static volatile bool motors_enabled;


/**
 * This node provides a simple interface to the ODrive module, it accepts cmd_vel messages to drive the motors,
 and publishes /vbus to report the current battery voltage
 */
int main(int argc, char **argv)
{
  /**
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "simplebgc");
  ros::NodeHandle n;
  ros::NodeHandle nhPriv("~");

  
  // Set the last message received time so we know if we stop getting messages and have to 
  // shut down the motors.
  last_received = ros::Time::now();

  ros::Rate loop_rate(10);

  int serial_port = open(nhPriv.param<std::string>("serial_port", "/dev/ttyACM0").c_str(), O_RDWR);

  if (serial_port < 0) {
      ROS_ERROR("Error %i from open: %s\n", errno, strerror(errno));
      return errno;
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      ROS_ERROR("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return errno;
  }

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return errno;
  }


  while (ros::ok())
  {
    ros::Time start = ros::Time::now();

    // If you haven't received a message in the last second, then stop the motors
    if (ros::Time::now() - last_received > ros::Duration(1)) {
      if (motors_enabled) {
        ROS_WARN("Didn't receive a message for the past second, shutting down motors");
        motors_enabled = false;

      }
    } 
    else {
      if (!motors_enabled) {
        ROS_INFO("Received message, enabling motors");
        motors_enabled = true;

      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}