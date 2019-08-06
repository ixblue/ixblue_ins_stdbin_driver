#include "udp_listener.h"
#include <boost/bind.hpp>
#include <ros/console.h>

using namespace boost::asio;

UDPListener::UDPListener(const std::string& ip, uint16_t port)
    : IPListener(ip, port),
      socket(service, ip::udp::endpoint(ip::address::from_string(ip), port))
{
    listenNextData();
    ROS_DEBUG_STREAM("Starting asio thread");
    asioThread = std::thread([&]() { service.run(); });
}

void UDPListener::listenNextData()
{
    socket.async_receive_from(boost::asio::buffer(datas), endpoint,
                              boost::bind(&IPListener::onNewDataReceived, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
}
