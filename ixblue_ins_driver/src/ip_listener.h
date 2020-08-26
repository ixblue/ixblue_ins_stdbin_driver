#pragma once

#include "ros_publisher.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/system/error_code.hpp>
#include <inttypes.h>
#include <ixblue_stdbin_decoder/stdbin_decoder.h>
#include <string>
#include <thread>

/*!
 * \brief Contains the common part of TCP and UDP Receiver.
 * This class manage the io_service thread, and the onNewDataReceived method.
 * In both cases (UDP or TCP), we do the same thing when new data are received :
 * * We check for errors
 * * We parse the data and publish if data are parsed
 * * We start listening for next data.
 */

class IPListener : private boost::noncopyable
{
    IPListener() = delete;

public:
    IPListener(const std::string& ip, uint16_t port);
    virtual ~IPListener();

    void onNewDataReceived(const boost::system::error_code& error,
                           std::size_t bytes_transfered);

protected:
    /*!
     * This is the pro-actor pattern implemented by boost asio. Each daughter class
     * must use this abstract method to listen next data.
     */
    virtual void listenNextData(void) = 0;
    const std::string ip;
    const uint16_t port;

    ixblue_stdbin_decoder::StdBinDecoder parser;

    boost::array<uint8_t, 8192> datas;
    boost::asio::io_service service;
    std::thread asioThread;
    ROSPublisher rosPublisher;
};
