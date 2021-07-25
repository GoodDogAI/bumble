#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <unistd.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rewardbutton");


  ros::NodeHandle n;
  ros::NodeHandle nhPriv("~");

  ros::Publisher reward_pub = n.advertise<std_msgs::Float32>("reward_button", 2);

  // Ros Params for settings rewards/penalties
  ros::Duration penalty_duration = ros::Duration(nhPriv.param<float>("penalty_duration_secs", 1.0));
  ros::Time last_penalty;

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

    while(ros::ok()) {
        int ret = poll(input_fds.data(), input_fds.size(), 500);
        if (input_fds[0].revents & POLLIN) {

            // accept one connection
            client = accept(s, (struct sockaddr *)&rem_addr, &opt);

            ba2str( &rem_addr.rc_bdaddr, buf );
            ROS_INFO("accepted connection from %s", buf);
            memset(buf, 0, sizeof(buf));

            // read data from the client
            bytes_read = read(client, buf, sizeof(buf));
            if( bytes_read > 0 ) {
                ROS_INFO("received [%s]", buf);
            }

            // close connection
            close(client);
        }
        ros::spinOnce();
    }
    
    close(s);
    return 0;

}