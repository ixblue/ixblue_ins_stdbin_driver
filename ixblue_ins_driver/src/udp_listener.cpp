#include "udp_listener.h"
#include <boost/bind.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace boost::asio;

UDPListener::UDPListener(const std::string& ip, uint16_t port)
    : IPListener(ip, port),
      socket(service, ip::udp::endpoint(ip::address::from_string(ip), port))
{
    listenNextData();
    //RCLCPP_DEBUG(rclcpp::get_logger("UDPListener"), "Starting asio thread");
    asioThread = std::thread([&]() { service.run(); });
}

void UDPListener::listenNextData()
{
    socket.async_receive_from(boost::asio::buffer(datas), endpoint,
                              boost::bind(&IPListener::onNewDataReceived, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
}
