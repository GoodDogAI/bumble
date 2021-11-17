#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "bumble/HeadFeedback.h"
#include "bumble/HeadCommand.h"
#include "simplebgc.h"

#include <iostream>
#include <fstream>
#include <string>

// Linux headers for opening serial port connections to the ODRIVE
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/poll.h> // For reading from the serial port without blocking
#include <math.h>

ros::Time bgc_last_received;
ros::Time ros_last_received;

//fd for serial port connection
int serial_port;

static uint8_t bgc_state = BGC_WAITING_FOR_START_BYTE;
static uint8_t bgc_payload_counter = 0;
static uint8_t bgc_payload_crc[2];
static uint8_t bgc_rx_buffer[BGC_RX_BUFFER_SIZE];
static bgc_msg * const bgc_rx_msg = (bgc_msg *)bgc_rx_buffer;

void crc16_update(uint16_t length, uint8_t *data, uint8_t crc[2]) {
  uint16_t counter;
  uint16_t polynom = 0x8005;
  uint16_t crc_register = (uint16_t)crc[0] | ((uint16_t)crc[1] << 8);
  uint8_t shift_register;
  uint8_t data_bit, crc_bit;
  for (counter = 0; counter < length; counter++) {
    for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1) {
      data_bit = (data[counter] & shift_register) ? 1 : 0;
      crc_bit = crc_register >> 15;
      crc_register <<= 1;

      if (data_bit != crc_bit) crc_register ^= polynom;
    }
  }

  crc[0] = crc_register;
  crc[1] = (crc_register >> 8);
}

void crc16_calculate(uint16_t length, uint8_t *data, uint8_t crc[2]) {
  crc[0] = 0; crc[1] = 0;
  crc16_update(length, data, crc);
}

void tty_raw(struct termios *raw) {
  /* input modes - clear indicated ones giving: no break, no CR to NL, 
      no parity check, no strip char, no start/stop output (sic) control */
  raw->c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  /* output modes - clear giving: no post processing such as NL to CR+NL */
  raw->c_oflag &= ~(OPOST);

  /* control modes - set 8 bit chars */
  raw->c_cflag |= (CS8);

  /* local modes - clear giving: echoing off, canonical off (no erase with 
      backspace, ^U,...),  no extended functions, no signal chars (^Z,^C) */
  raw->c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

  /* control chars - set return condition: min number of bytes and timer */
  // raw->c_cc[VMIN] = 5; raw->c_cc[VTIME] = 8; /* after 5 bytes or .8 seconds
  //                                             after first byte seen      */
  raw->c_cc[VMIN] = 0; raw->c_cc[VTIME] = 0; /* immediate - anything       */
  // raw->c_cc[VMIN] = 2; raw->c_cc[VTIME] = 0; /* after two bytes, no timer  */
  // raw->c_cc[VMIN] = 0; raw->c_cc[VTIME] = 8; /* after a byte or .8 seconds */
}

void send_message(int fd, uint8_t cmd, uint8_t *payload, uint16_t payload_size) {
  bgc_msg *cmd_msg = (bgc_msg *)malloc(sizeof(bgc_msg) + payload_size);
  cmd_msg->command_id = cmd;
  cmd_msg->payload_size = payload_size;
  cmd_msg->header_checksum = cmd_msg->command_id + cmd_msg->payload_size;

  memcpy(cmd_msg->payload, payload, payload_size);
  
  uint8_t crc[2];
  crc16_calculate(sizeof(bgc_msg) + payload_size, (uint8_t *)cmd_msg, crc);

  write(fd, &simplebgc_start_byte, 1);
  write(fd, cmd_msg, sizeof(bgc_msg) + payload_size);
  write(fd, crc, sizeof(crc));
}

void head_cmd_callback(const bumble::HeadCommand::ConstPtr& msg)
{
  // Send a control command immediately to set the new position
  bgc_control_data control_data;
  memset(&control_data, 0, sizeof(control_data));

  control_data.control_mode_roll = CONTROL_MODE_IGNORE;
  control_data.control_mode_pitch = CONTROL_MODE_ANGLE_REL_FRAME;
  control_data.control_mode_yaw = CONTROL_MODE_ANGLE_REL_FRAME;
  control_data.angle_pitch = round(DEG_TO_INT16(msg->cmd_angle_pitch));
  control_data.angle_yaw = round(DEG_TO_INT16(msg->cmd_angle_yaw));
  send_message(serial_port, CMD_CONTROL, (uint8_t *)&control_data, sizeof(control_data));

  // ROS_INFO("Received head cmd %f %d, %f %d", 
  //   msg->cmd_angle_pitch, control_data.angle_pitch,
  //   msg->cmd_angle_yaw, control_data.angle_yaw);
  
  ros_last_received = ros::Time::now();
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
  ros::init(argc, argv, "simplebgc");
  ros::NodeHandle n;
  ros::NodeHandle nhPriv("~");

  ros::Subscriber sub = n.subscribe("head_cmd", 1, head_cmd_callback);
  ros::Publisher feedback_pub = n.advertise<bumble::HeadFeedback>("head_feedback", 1);

  // Set the last message received time so we know if we stop getting messages and have to 
  // shut down the motors.
  bgc_last_received = ros::Time::now();
  ros_last_received = ros::Time::now();

  ros::Rate loop_rate(10);

  serial_port = open(nhPriv.param<std::string>("serial_port", "/dev/ttyTHS0").c_str(), O_RDWR | O_NOCTTY);

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
  tty_raw(&tty);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return errno;
  }

  ROS_INFO("Opened SimpleBGC serial port %s", nhPriv.param<std::string>("serial_port", "/dev/ttyTHS0").c_str());

  // Recenter the YAW Axis
  uint8_t menu_cmd = SBGC_MENU_CENTER_YAW;
  send_message(serial_port, CMD_EXECUTE_MENU, &menu_cmd, 1);

  // Register a realtime data stream syncing up with the loop rate
  bgc_data_stream_interval stream_data;
  memset(&stream_data, 0, sizeof(bgc_data_stream_interval));
  stream_data.cmd_id = CMD_REALTIME_DATA_4;
  stream_data.interval_ms = loop_rate.expectedCycleTime().toSec() * 1000;
  stream_data.sync_to_data = 0;

  send_message(serial_port, CMD_DATA_STREAM_INTERVAL, (uint8_t *)&stream_data, sizeof(stream_data));
 
  pollfd serial_port_poll = {serial_port, POLLIN, 0};

  while (ros::ok())
  {
    // Exit with an error if you haven't received a message in a while
    if (ros::Time::now() - bgc_last_received > ros::Duration(1.0)) {
      ROS_ERROR("No messages received in 5 seconds, shutting down BGC subsystem");
      return 1;
    }

    int ret = poll(&serial_port_poll, 1, 5);
    
    if (serial_port_poll.revents & POLLIN) {
      uint8_t buf[1024];
      ssize_t bytes_read = read(serial_port, buf, sizeof(buf));

      if (bytes_read < 0) {
        ROS_ERROR("Error %i from read: %s\n", errno, strerror(errno));
        return errno;
      }

      for (ssize_t i = 0; i < bytes_read; i++) {
        if (bgc_state == BGC_WAITING_FOR_START_BYTE && buf[i] == simplebgc_start_byte) {
          bgc_state = BGC_READ_COMMAND_ID;
        }
        else if (bgc_state == BGC_READ_COMMAND_ID) {
          bgc_rx_msg->command_id = buf[i];
          bgc_state = BGC_READ_PAYLOAD_SIZE;
        }
        else if (bgc_state == BGC_READ_PAYLOAD_SIZE) {
          bgc_rx_msg->payload_size = buf[i];
          bgc_state = BGC_READ_HEADER_CHECKSUM;
        }
        else if (bgc_state == BGC_READ_HEADER_CHECKSUM) {
          bgc_rx_msg->header_checksum = buf[i];

          if (bgc_rx_msg->header_checksum != bgc_rx_msg->command_id + bgc_rx_msg->payload_size) {
            ROS_ERROR("Header checksum failed");
            bgc_state = BGC_WAITING_FOR_START_BYTE;
          }
          else {
            bgc_state = BGC_READ_PAYLOAD;
            bgc_payload_counter = 0;
          }
        }
        else if (bgc_state == BGC_READ_PAYLOAD) {
          bgc_rx_msg->payload[bgc_payload_counter] = buf[i];
          bgc_payload_counter++;

          if (bgc_payload_counter == bgc_rx_msg->payload_size) {
            bgc_state = BGC_READ_CRC_0;
          }
        }
        else if (bgc_state == BGC_READ_CRC_0) {
          bgc_payload_crc[0] = buf[i];
          bgc_state = BGC_READ_CRC_1;
        }
        else if (bgc_state == BGC_READ_CRC_1) {
          bgc_payload_crc[1] = buf[i];

          uint8_t crc[2];
          crc16_calculate(sizeof(bgc_msg) + bgc_rx_msg->payload_size, bgc_rx_buffer, crc);

          if (crc[0] != bgc_payload_crc[0] || crc[1] != bgc_payload_crc[1]) {
            ROS_ERROR("Payload checksum failed");
          }
          else {
            //ROS_INFO("Recieved valid message of type %d", bgc_rx_msg->command_id);
            bgc_last_received = ros::Time::now();

            if (bgc_rx_msg->command_id == CMD_REALTIME_DATA_4) {
              bgc_realtime_data_4 *realtime_data = (bgc_realtime_data_4 *)bgc_rx_msg->payload;

              if (realtime_data->system_error) {
                ROS_ERROR("BGC Error %02x", realtime_data->system_error);
                ROS_ERROR("Shutting down BGC");
                return realtime_data->system_error;
              }

            //  ROS_INFO("Pitch %0.4f %0.4f %0.4f", 
            //       INT16_TO_DEG(realtime_data->imu_angle_pitch),
            //       INT16_TO_DEG(realtime_data->target_angle_pitch),
            //       INT16_TO_DEG(realtime_data->stator_angle_pitch));

              // Publish a feedback message with the data
              bumble::HeadFeedback feedback_msg;
              feedback_msg.cur_angle_pitch = INT16_TO_DEG(realtime_data->stator_angle_pitch);
              feedback_msg.cur_angle_yaw = INT16_TO_DEG(realtime_data->stator_angle_yaw);
              feedback_msg.header.stamp = ros::Time::now();
              feedback_pub.publish(feedback_msg);
            }
            else if (bgc_rx_msg->command_id == CMD_GET_ANGLES_EXT) {
              bgc_angles_ext *angles_ext = (bgc_angles_ext *)bgc_rx_msg->payload;
              ROS_INFO("Yaw %0.4f %0.4f %0.4f", 
                  INT16_TO_DEG(angles_ext->imu_angle_yaw),
                  INT16_TO_DEG(angles_ext->target_angle_yaw),
                  INT16_TO_DEG(angles_ext->stator_angle_yaw));

               ROS_INFO("Pitch %0.4f %0.4f %0.4f", 
                  INT16_TO_DEG(angles_ext->imu_angle_pitch),
                  INT16_TO_DEG(angles_ext->target_angle_pitch),
                  INT16_TO_DEG(angles_ext->stator_angle_pitch));
            }
            else if (bgc_rx_msg->command_id == CMD_ERROR) {
               ROS_ERROR("Received CMD_ERROR from BGC");
            }
          }

          bgc_state = BGC_WAITING_FOR_START_BYTE;
        }
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}