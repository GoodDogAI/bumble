#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Byte.h"

#include <unistd.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <vector>

#define MAX_MISSED_INTERVALS 3

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rewardbutton");


  ros::NodeHandle n;
  ros::NodeHandle nhPriv("~");

  ros::Publisher reward_pub = n.advertise<std_msgs::Byte>("reward_button", 0);
  ros::Publisher reward_connected = n.advertise<std_msgs::Bool>("reward_button_connected", false);

    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    char buf[1024] = { 0 };
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
                        for (int i=0; i<bytes_read; i++) {
                            if (buf[i]) {
                                data_msg.data = buf[i];
                                reward_pub.publish(data_msg);
                            }
                        }
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