#include "udp_listener.h"
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ixblue_ins_driver");
    ros::NodeHandle nh("~");

    std::string ip;
    int udp_port;
    nh.param("udp_port", udp_port, 8200);
    nh.param("ip", ip, std::string("0.0.0.0"));
    ROS_INFO("UDP port : %d", udp_port);
    ROS_INFO("IP adress : %s", ip.c_str());

    if(udp_port > std::numeric_limits<uint16_t>::max())
    {
        ROS_ERROR_STREAM(
            "UDP Port can't be greater than : " << std::numeric_limits<uint16_t>::max());
        return -1;
    }

    UDPListener udpListener(ip, static_cast<uint16_t>(udp_port));

    ros::spin();
    return 0;
}
