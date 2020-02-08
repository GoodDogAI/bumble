#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <fstream>

// Linux headers for opening serial port connections to the ODRIVE
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <math.h>

#define MAX_SPEED 50.0
static volatile float vel_left, vel_right;


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

  std::string response = "";
  char buf;
  int num_read;

  while(num_read = read(fd, &buf, 1)) {
    if (buf == '\n')
      break;

    response += buf;
  }

  return std::stoi(response);
}

float send_float_command(int fd, const std::string& command) {
  int write_res = write(fd, command.c_str(), command.length());

  if (write_res != command.length()) {
    ROS_ERROR("Error sending float command");
    return NAN;
  }

  std::string response = "";
  char buf;
  int num_read;

  while(num_read = read(fd, &buf, 1)) {
    if (buf == '\n')
      break;

    response += buf;
  }

  return std::stof(response);
}


void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Linear %f, %f, %f,  Ang %f, %f, %f", 
    msg->linear.x, msg->linear.y, msg->linear.z,
    msg->angular.x, msg->angular.y, msg->angular.z);

  float ang = msg->angular.z;

  vel_left = -(msg->linear.x - ang) * MAX_SPEED;
  vel_right = (msg->linear.x + ang) * MAX_SPEED;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "odrive");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher vbus_pub = n.advertise<std_msgs::Float32>("vbus", 10);

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

  //Put motors into AXIS_STATE_CLOSED_LOOP_CONTROL
  send_raw_command(serial_port, "w axis0.requested_state 8\n");
  send_raw_command(serial_port, "w axis1.requested_state 8\n");

  ros::Subscriber sub = n.subscribe("cmd_vel", 10, twistCallback);

  while (ros::ok())
  {
    // Read and publish the vbus main voltage
    float vbus_voltage = send_float_command(serial_port, "r vbus_voltage\n");

    std_msgs::Float32 vbus_msg;
    vbus_msg.data = vbus_voltage;
    //ROS_INFO("Publishing message: %f", vbus_voltage);
    vbus_pub.publish(vbus_msg);

    //ROS_INFO("Sending motor vels %f %f", vel_left, vel_right);

    std::string cmd;
    cmd = "v 0 " + std::to_string(vel_left) + "\n";
    send_raw_command(serial_port, cmd.c_str());

    cmd = "v 1 " + std::to_string(vel_right) + "\n";
    send_raw_command(serial_port, cmd.c_str());

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}