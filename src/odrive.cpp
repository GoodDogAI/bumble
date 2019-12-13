#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <fstream>

// Linux headers for opening serial port connections to the ODRIVE
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


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

  std::fstream serial_port;
  serial_port.open("/dev/ttyACM0", std::ios::in | std::ios::out);

    if (!serial_port.is_open()) {
      ROS_ERROR("Error %i from open: %s\n", errno, strerror(errno));
      return errno;
  }

  while (ros::ok())
  {
    float vbus_voltage = 0.0;

    serial_port << "r vbus_voltage" << std::endl;
    serial_port >> vbus_voltage;

        ROS_INFO("First read message: %f", vbus_voltage);


    serial_port << "r vbus_voltage" << std::endl;
    serial_port >> vbus_voltage;

    std_msgs::Float32 vbus_msg;
    vbus_msg.data = vbus_voltage;
    ROS_INFO("Publishing message: %f", vbus_voltage);

    vbus_pub.publish(vbus_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}