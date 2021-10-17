#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include <unistd.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <vector>

#define MAX_MISSED_INTERVALS 3

#define BUF_DISCRIMINANT_INDEX 4

#define BUF_EMPTY_MESSAGE 0x00
#define BUF_MOVE_MESSAGE 0x01
#define BUF_SCORE_MESSAGE 0x02

#define BUF_SCORE_INDEX 5
#define BUF_CMD_VEL_ACTION_INDEX 5

#define BUF_CMD_VEL_ACTION_FORWARD 1
#define BUF_CMD_VEL_ACTION_BACKWARD 2
#define BUF_CMD_VEL_ACTION_LEFT 4
#define BUF_CMD_VEL_ACTION_RIGHT 8


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rewardbutton");

    ros::NodeHandle n;
    ros::NodeHandle nhPriv("~");

    float override_linear_speed = nhPriv.param<float>("override_linear_speed", 1.0);
    float override_angular_speed = nhPriv.param<float>("override_angular_speed", 1.0);

    ros::Publisher reward_pub = n.advertise<std_msgs::Float32>("reward_button", 0);
    ros::Publisher reward_connected = n.advertise<std_msgs::Bool>("reward_button_connected", false);
    ros::Publisher reward_override_cmd_vel_pub = n.advertise<std_msgs::Bool>("reward_button_override_cmd_vel", 0);
    ros::Publisher reward_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("reward_button_cmd_vel", 0);

    ros::Publisher reward_raw_pub = n.advertise<std_msgs::Byte>("reward_button_raw", 0);

    ros::Duration penalty_duration = ros::Duration(nhPriv.param<float>("penalty_duration_secs", 1.0));
    
    ros::Time last_penalty;
    int8_t last_score_received = 0;

    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    char buf[8] = { 0 };
    int s, client, bytes_read;
    socklen_t opt = sizeof(rem_addr);

    int ret_code;

    // allocate socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // bind socket to port 1 of the first available 
    // local bluetooth adapter
    bdaddr_t ANY = {0,0,0,0,0,0};
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = ANY;
    loc_addr.rc_channel = (uint8_t) 1;
    ret_code = bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));
    ROS_INFO("bind: %d", ret_code);

    // put socket into listening mode
    ret_code = listen(s, 1);
    ROS_INFO("listen: %d", ret_code);

    // Wait for connections.
    std::vector<pollfd> input_fds;
    input_fds.push_back({s, POLLIN, 0});

    // Track time since last connection.
    int missed_intervals = 0;
    std_msgs::Bool connected_msg;
    connected_msg.data = false;

    std_msgs::Bool override_cmd_vel_msg;
    override_cmd_vel_msg.data = false;

    std_msgs::Byte data_msg;
    data_msg.data = 0;

    while(ros::ok()) {
        int ret = poll(input_fds.data(), input_fds.size(), 500);
        if (input_fds[0].revents & POLLIN) {

            // accept one connection
            client = accept(s, (struct sockaddr *)&rem_addr, &opt);

            ba2str( &rem_addr.rc_bdaddr, buf );
            ROS_INFO("accepted connection from %s (on fd %d)", buf, client);
            
            std::vector<pollfd> conn_fds;
            conn_fds.push_back({client, POLLIN, 0});
            
            while (ros::ok()) {
                int ret = poll(conn_fds.data(), conn_fds.size(), 500);
                if (conn_fds[0].revents & POLLIN) {

                    memset(buf, 0, sizeof(buf));
                    // read data from the client
                    bytes_read = read(client, buf, sizeof(buf));
                    if( bytes_read > 0 ) {
                        if (!connected_msg.data) {
                            ROS_INFO("connected");
                            connected_msg.data = true;
                            reward_connected.publish(connected_msg);
                        }
                        data_msg.data = buf[0];
                        reward_raw_pub.publish(data_msg);

                        ROS_INFO("read [%d,%d,%d,%d, %d,%d, %d,%d]", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);


                        if (buf[BUF_DISCRIMINANT_INDEX] == BUF_MOVE_MESSAGE) {
                            geometry_msgs::Twist cmd_vel_msg;

                            if (buf[BUF_CMD_VEL_ACTION_INDEX] & BUF_CMD_VEL_ACTION_FORWARD)
                                cmd_vel_msg.linear.x = override_linear_speed;
                            else if (buf[BUF_CMD_VEL_ACTION_INDEX] & BUF_CMD_VEL_ACTION_BACKWARD)
                                cmd_vel_msg.linear.x = -override_linear_speed;

                            if (buf[BUF_CMD_VEL_ACTION_INDEX] & BUF_CMD_VEL_ACTION_LEFT)
                                cmd_vel_msg.angular.z = override_angular_speed;
                            else if (buf[BUF_CMD_VEL_ACTION_INDEX] & BUF_CMD_VEL_ACTION_RIGHT)
                                cmd_vel_msg.angular.z = -override_angular_speed;
                            
                            reward_cmd_vel_pub.publish(cmd_vel_msg);
                            override_cmd_vel_msg.data = true;
                        }
                        else if (buf[BUF_DISCRIMINANT_INDEX] == BUF_SCORE_MESSAGE) {
                            last_score_received = buf[BUF_SCORE_INDEX];
                            ROS_INFO("Score %d detected from app", last_score_received);
                            
                            last_penalty = ros::Time::now();
                        }
                        else if (buf[BUF_DISCRIMINANT_INDEX] == BUF_EMPTY_MESSAGE) {
                            override_cmd_vel_msg.data = false;
                        }
                        else {
                            ROS_WARN("Unknown message recieved %x", buf[BUF_DISCRIMINANT_INDEX]);
                        }


                        reward_override_cmd_vel_pub.publish(override_cmd_vel_msg);


                        // Publish the latest value of the reward button
                        std_msgs::Float32 reward;
                        reward.data = (ros::Time::now() - last_penalty) > penalty_duration ? 0.0 : (float)last_score_received;
                        reward_pub.publish(reward);

                        missed_intervals = 0;
                    }
                } else {
                    missed_intervals++;
                    if (missed_intervals >= MAX_MISSED_INTERVALS && connected_msg.data) {
                        ROS_INFO("disconnected");
                        connected_msg.data = false;
                        reward_connected.publish(connected_msg);
                        break;
                    }
                }
                ros::spinOnce();
            }
            // close connection
            close(client);
        }
        ros::spinOnce();
    }
    
    close(s);
    return 0;
}