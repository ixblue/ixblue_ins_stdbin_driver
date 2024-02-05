#include "ip_listener.h"
//#include <ros/console.h>

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
        RCLCPP_WARN_STREAM(rosPublisher->getNode()->get_logger(),"Error occurs in IP Listener : " << error.message());
    }
    else
    {
        RCLCPP_DEBUG_STREAM(rosPublisher->getNode()->get_logger(),"Received StdBin data");
        // No errors, we can parse it :
        try
        {
            parser.addNewData(datas.data(), bytes_transfered);
            while(parser.parseNextFrame())
            {
                auto navData = parser.getLastNavData();
                auto headerData = parser.getLastHeaderData();
                rosPublisher->onNewStdBinData(navData, headerData);
            }
        }
        catch(const std::runtime_error& e)
        {
            RCLCPP_WARN_STREAM(rosPublisher->getNode()->get_logger(),"Parse error : " << e.what());
            // TODO : Publish a diagnostic
        }
    }
    listenNextData();
}

void IPListener::setPublisher(std::shared_ptr<ROSPublisher> pub)
{
  rosPublisher = pub;
}
