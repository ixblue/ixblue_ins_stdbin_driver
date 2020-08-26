#include "ip_listener.h"
#include <ros/console.h>

using namespace boost::asio;

IPListener::IPListener(const std::string& ip, uint16_t port) : ip(ip), port(port) {}

IPListener::~IPListener()
{
    service.stop();
    asioThread.join();
}

void IPListener::onNewDataReceived(const boost::system::error_code& error,
                                   std::size_t bytes_transfered)
{

    if(error == boost::asio::error::operation_aborted)
    {
        // Will happen when we close the socket
        return;
    }
    else if(error)
    {
        // We don't publish a diagnostics here, they will be handled in an higher level.
        // If there is an error, there is no parse, so diagnostic updater will detect it.
        ROS_WARN_STREAM("Error occurs in IP Listener : " << error.message());
    }
    else
    {
        ROS_DEBUG_STREAM("Received StdBin data");
        // No errors, we can parse it :
        std::vector<uint8_t> dataForParser;
        dataForParser.reserve(bytes_transfered);
        std::copy(std::begin(datas), std::begin(datas) + bytes_transfered,
                  std::back_inserter(dataForParser));
        // TODO : Change parser parameter to accept a buffer, that will allow to remove
        // this useless copy.
        try
        {
            if(parser.parse(dataForParser))
            {
                auto navData = parser.getLastNavData();
                auto headerData = parser.getLastHeaderData();
                rosPublisher.onNewStdBinData(navData, headerData);
            }
        }
        catch(std::runtime_error& e)
        {
            ROS_WARN_STREAM("Parse error : " << e.what());
            // TODO : Publish a diagnostic
        }
    }
    listenNextData();
}
