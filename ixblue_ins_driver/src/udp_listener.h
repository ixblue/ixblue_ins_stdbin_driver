#pragma once

#include "ip_listener.h"
#include <boost/asio.hpp>

class UDPListener : public IPListener
{
    UDPListener() = delete;
public:
    UDPListener(const std::string& ip, uint16_t port);

protected:
    void listenNextData();
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::endpoint endpoint;
};
