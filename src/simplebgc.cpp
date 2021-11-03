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
#include <sys/poll.h> // For reading from the serial port without blocking
#include <math.h>


ros::Time last_received;
static volatile bool motors_enabled;

static const uint8_t simplebgc_start_byte = 0x24;

typedef struct {
  uint8_t command_id;
  uint8_t payload_size;
  uint8_t header_checksum;
  uint8_t payload[];
} bgc_msg;

#define CMD_READ_PARAMS  82
#define CMD_WRITE_PARAMS  87
#define CMD_REALTIME_DATA  68
#define CMD_BOARD_INFO  86
#define CMD_CALIB_ACC  65
#define CMD_CALIB_GYRO  103
#define CMD_CALIB_EXT_GAIN  71
#define CMD_USE_DEFAULTS  70
#define CMD_CALIB_POLES  80
#define CMD_RESET  114
#define CMD_HELPER_DATA 72
#define CMD_CALIB_OFFSET  79
#define CMD_CALIB_BAT  66
#define CMD_MOTORS_ON   77
#define CMD_MOTORS_OFF  109
#define CMD_CONTROL   67
#define CMD_TRIGGER_PIN  84
#define CMD_EXECUTE_MENU 69
#define CMD_GET_ANGLES  73
#define CMD_CONFIRM  67
#define CMD_BOARD_INFO_3  20
#define CMD_READ_PARAMS_3 21
#define CMD_WRITE_PARAMS_3 22
#define CMD_REALTIME_DATA_3  23
#define CMD_REALTIME_DATA_4  25
#define CMD_SELECT_IMU_3 24
#define CMD_READ_PROFILE_NAMES 28
#define CMD_WRITE_PROFILE_NAMES 29
#define CMD_QUEUE_PARAMS_INFO_3 30
#define CMD_SET_ADJ_VARS_VAL 31
#define CMD_SAVE_PARAMS_3 32
#define CMD_READ_PARAMS_EXT 33
#define CMD_WRITE_PARAMS_EXT 34
#define CMD_AUTO_PID 35
#define CMD_SERVO_OUT 36
#define CMD_I2C_WRITE_REG_BUF 39
#define CMD_I2C_READ_REG_BUF 40
#define CMD_WRITE_EXTERNAL_DATA 41
#define CMD_READ_EXTERNAL_DATA 42
#define CMD_READ_ADJ_VARS_CFG 43
#define CMD_WRITE_ADJ_VARS_CFG 44
#define CMD_API_VIRT_CH_CONTROL 45
#define CMD_ADJ_VARS_STATE 46
#define CMD_EEPROM_WRITE 47
#define CMD_EEPROM_READ 48
#define CMD_CALIB_INFO 49
#define CMD_SIGN_MESSAGE 50
#define CMD_BOOT_MODE_3 51
#define CMD_SYSTEM_STATE 52
#define CMD_READ_FILE 53
#define CMD_WRITE_FILE 54
#define CMD_FS_CLEAR_ALL 55
#define CMD_AHRS_HELPER 56
#define CMD_RUN_SCRIPT 57
#define CMD_SCRIPT_DEBUG 58
#define CMD_CALIB_MAG 59
#define CMD_GET_ANGLES_EXT 61
#define CMD_READ_PARAMS_EXT2 62
#define CMD_WRITE_PARAMS_EXT2 63
#define CMD_GET_ADJ_VARS_VAL 64
#define CMD_CALIB_MOTOR_MAG_LINK 74
#define CMD_GYRO_CORRECTION 75
#define CMD_DATA_STREAM_INTERVAL 85
#define CMD_REALTIME_DATA_CUSTOM 88
#define CMD_BEEP_SOUND 89
#define CMD_ENCODERS_CALIB_OFFSET_4  26
#define CMD_ENCODERS_CALIB_FLD_OFFSET_4 27
#define CMD_CONTROL_CONFIG 90
#define CMD_CALIB_ORIENT_CORR 91
#define CMD_COGGING_CALIB_INFO 92
#define CMD_CALIB_COGGING 93
#define CMD_CALIB_ACC_EXT_REF 94
#define CMD_PROFILE_SET 95
#define CMD_CAN_DEVICE_SCAN 96
#define CMD_CAN_DRV_HARD_PARAMS 97
#define CMD_CAN_DRV_STATE 98
#define CMD_CAN_DRV_CALIBRATE 99
#define CMD_READ_RC_INPUTS 100
#define CMD_REALTIME_DATA_CAN_DRV 101
#define CMD_EVENT 102
#define CMD_READ_PARAMS_EXT3 104
#define CMD_WRITE_PARAMS_EXT3 105
#define CMD_EXT_IMU_DEBUG_INFO 106
#define CMD_SET_DEVICE_ADDR 107
#define CMD_AUTO_PID2 108
#define CMD_EXT_IMU_CMD 110
#define CMD_READ_STATE_VARS 111
#define CMD_WRITE_STATE_VARS 112
#define CMD_SERIAL_PROXY 113
#define CMD_IMU_ADVANCED_CALIB 115
#define CMD_API_VIRT_CH_HIGH_RES 116
#define CMD_SET_DEBUG_PORT 249
#define CMD_MAVLINK_INFO 250
#define CMD_MAVLINK_DEBUG 251
#define CMD_DEBUG_VARS_INFO_3 253
#define CMD_DEBUG_VARS_3 254
#define CMD_ERROR  255

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

  int serial_port = open(nhPriv.param<std::string>("serial_port", "/dev/ttyTHS0").c_str(), O_RDWR | O_NOCTTY);

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

 
  pollfd serial_port_poll = {serial_port, POLLIN, 0};

  while (ros::ok())
  {
    ros::Time start = ros::Time::now();

    // Continue sending the board info command until we get a response
    uint8_t board_info_payload[2];
    board_info_payload[0] = 0x00;
    board_info_payload[1] = 0x00;
    send_message(serial_port, CMD_BOARD_INFO, board_info_payload, 2);

    int ret = poll(&serial_port_poll, 1, 500);
    
    if (serial_port_poll.revents & POLLIN) {
      ROS_INFO("POLL for serial port POLLIN");
      uint8_t buf[256];
      ssize_t bytes_read = read(serial_port, buf, sizeof(buf));

      ROS_INFO("Read %zu bytes", bytes_read);
    }

    // If you haven't received a message in the last second, then stop the motors
    // if (ros::Time::now() - last_received > ros::Duration(1)) {
    //   if (motors_enabled) {
    //     ROS_WARN("Didn't receive a message for the past second, shutting down BGC");
    //     motors_enabled = false;

    //   }
    // } 
    // else {
    //   if (!motors_enabled) {
    //     ROS_INFO("Received ROS message, enabling BGC");
    //     motors_enabled = true;

    //   }
    // }

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}