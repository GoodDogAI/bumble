#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "mainbot/ODriveFeedback.h"

#include <iostream>
#include <fstream>

// Linux headers for opening serial port connections to the ODRIVE
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <math.h>

static volatile float MAX_SPEED;

static volatile bool motors_enabled;
static volatile float vel_left, vel_right;
ros::Time last_received;

std::string read_string(int fd) {
  std::string response = "";
  char buf;
  int num_read;

  while(num_read = read(fd, &buf, 1)) {
    // Make sure to read full lines of \r\n
    if (buf == '\r')
      continue;

    if (isspace(buf))
      break;

    response += buf;
  }

  //Useful for debugging
  //std::cout << "read_string: '" << response << "'" << std::endl;
  return response;
}

void send_raw_command(int fd, const std::string& command) {
  int write_res = write(fd, command.c_str(), command.length());

  if (write_res != command.length()) {
    ROS_ERROR("Error sending float command");
  }
}

int send_int_command(int fd, const std::string& command) {
  int write_res = write(fd, command.c_str(), command.length());

  if (write_res != command.length()) {
    ROS_ERROR("Error sending int command");
    return -1;
  }

  std::string response = read_string(fd);

  if (response == "unknown") {
    ROS_WARN("Received 'unknown' response to %s", command.c_str());
    return -1;
  }

  return std::stoi(response);
}

float send_float_command(int fd, const std::string& command) {
  int write_res = write(fd, command.c_str(), command.length());

  if (write_res != command.length()) {
    ROS_ERROR("Error sending float command");
    return NAN;
  }

  std::string response = read_string(fd);

  if (response == "unknown") {
    ROS_WARN("Received 'unknown' response to %s", command.c_str());
    return NAN;
  }

  return std::stof(response);
}


void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // ROS_INFO("Linear %f, %f, %f,  Ang %f, %f, %f", 
  //   msg->linear.x, msg->linear.y, msg->linear.z,
  //   msg->angular.x, msg->angular.y, msg->angular.z);

  float ang = msg->angular.z;

  vel_left = -1.0f * (msg->linear.x - ang); // -1 to flip direction
  vel_right = (msg->linear.x + ang);

  if (vel_left > MAX_SPEED)
    vel_left = MAX_SPEED;
  if (vel_left < -MAX_SPEED)
    vel_left = -MAX_SPEED;

  if (vel_right > MAX_SPEED)
    vel_right = MAX_SPEED;
  if (vel_right < -MAX_SPEED)
    vel_right = -MAX_SPEED;

  last_received = ros::Time::now();
}

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
  ros::init(argc, argv, "odrive");
  ros::NodeHandle n;
  ros::NodeHandle nhPriv("~");

  // Read in the max_speed and other parameters
  MAX_SPEED = nhPriv.param<float>("max_speed", 2.0);

  // Set the last message received time so we know if we stop getting messages and have to 
  // shut down the motors.
  last_received = ros::Time::now();

  ros::Publisher vbus_pub = n.advertise<std_msgs::Float32>("vbus", 10);
  ros::Publisher feedback_pub = n.advertise<mainbot::ODriveFeedback>("odrive_feedback", 5);

  ros::Rate loop_rate(10);

  int serial_port = open("/dev/ttyACM0", O_RDWR);

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

  ros::Subscriber sub = n.subscribe("cmd_vel", 10, twistCallback);

  ROS_INFO("Successfully started odrive communications");

  while (ros::ok())
  {
    ros::Time start = ros::Time::now();

    // If you haven't received a message in the last second, then stop the motors
    if (ros::Time::now() - last_received > ros::Duration(1)) {
      if (motors_enabled) {
        ROS_WARN("Didn't receive a message for the past second, shutting down motors");
        vel_left = vel_right = 0;

        send_raw_command(serial_port, "w axis0.requested_state 1\n");
        send_raw_command(serial_port, "w axis1.requested_state 1\n");
        motors_enabled = false;
      }
    } 
    else {
      if (!motors_enabled) {
        ROS_INFO("Received message, enabling motors");

        //Put motors into AXIS_STATE_CLOSED_LOOP_CONTROL
        send_raw_command(serial_port, "w axis0.requested_state 8\n");
        send_raw_command(serial_port, "w axis1.requested_state 8\n");
        motors_enabled = true;
      }
    }


    //ROS_INFO("Sending motor vels %f %f", vel_left, vel_right);
    std::string cmd;
    cmd = "v 0 " + std::to_string(vel_left) + "\n";
    send_raw_command(serial_port, cmd.c_str());

    cmd = "v 1 " + std::to_string(vel_right) + "\n";
    send_raw_command(serial_port, cmd.c_str());


    // Read and publish the vbus main voltage
    float vbus_voltage = send_float_command(serial_port, "r vbus_voltage\n");

    std_msgs::Float32 vbus_msg;
    vbus_msg.data = vbus_voltage;
    vbus_pub.publish(vbus_msg);

    // Read and publish the motor feedback values
    mainbot::ODriveFeedback feedback_msg;
    send_raw_command(serial_port, "f 0\n"); 
    feedback_msg.motor_pos_actual_0 = std::stof(read_string(serial_port));
    feedback_msg.motor_vel_actual_0 = std::stof(read_string(serial_port));
    feedback_msg.motor_vel_cmd_0 = vel_left;

    send_raw_command(serial_port, "f 1\n");
    feedback_msg.motor_pos_actual_1 = std::stof(read_string(serial_port));
    feedback_msg.motor_vel_actual_1 = std::stof(read_string(serial_port));
    feedback_msg.motor_vel_cmd_1 = vel_right;

    feedback_msg.header.stamp = ros::Time::now();
    feedback_pub.publish(feedback_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Disable motors when we quit the program
  send_raw_command(serial_port, "w axis0.requested_state 1\n");
  send_raw_command(serial_port, "w axis1.requested_state 1\n");

  return 0;
}