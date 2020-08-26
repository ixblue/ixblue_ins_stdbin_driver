#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <pcap.h>

using namespace boost::asio;
namespace po = boost::program_options;

pcap_t* pcapDescriptor = nullptr;
bool shouldRepeat = false;
// std::shared_ptr<ip::udp::socket> UDPSocket;
io_service service;
ip::udp::socket UDPSocket(service);
ip::udp::endpoint endPoint;
deadline_timer timer(service);
long period_ms;
std::string pcapFile;

void processNextDataInPcapFile(const boost::system::error_code& errocode)
{
    if(errocode != boost::asio::error::operation_aborted && errocode)
    {
        std::cout << "Unexpected timer error : " << errocode.message();
        exit(-1);
    }
    char errbuf[PCAP_ERRBUF_SIZE];
    if(!pcapDescriptor)
    {
        pcapDescriptor = pcap_open_offline(pcapFile.c_str(), errbuf);
        if(pcapDescriptor == NULL)
        {
            std::cout << "pcap_open() failed: " << errbuf << std::endl;
            exit(1);
        }
    }

    struct pcap_pkthdr* header;
    const u_char* pkt_data;
    // Here we read the next packet in the PCAP File
    auto res = pcap_next_ex(pcapDescriptor, &header, &pkt_data);
    if(res < 0)
    {
        // We are at end of file,
        pcap_close(pcapDescriptor);
        if(shouldRepeat)
        {
            // if we must repeat, then we close the file and recall this method.
            pcapDescriptor = nullptr;
            processNextDataInPcapFile(errocode);
        }
        else
        {
            exit(0);
        }
    }
    boost::system::error_code ec;
    const u_char* startOfData = pkt_data + sizeof(struct ether_header) +
                                sizeof(struct ip) + sizeof(struct udphdr);
    size_t dataLength = header->len - (sizeof(struct ether_header) + sizeof(struct ip) +
                                       sizeof(struct udphdr));
    if(*startOfData != 'I' && *(startOfData + 1) != 'X')
    {
        std::cout << "Can't send this packet, its not STDBin packet" << std::endl;
    }
    else
    {
        UDPSocket.send_to(boost::asio::buffer(startOfData, dataLength), endPoint, 0, ec);
        if(ec)
        {
            std::cout << "Problem when sending data on UDP " << ec.message() << std::endl;
        }
    }
    // We restart the timer.
    timer.expires_from_now(boost::posix_time::milliseconds(period_ms));
    timer.async_wait(processNextDataInPcapFile);
}

int main(int argc, char* argv[])
{
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "produce help message")
        ("period", po::value<long>()->default_value(10), "Publication period in ms.")
        ("repeat", "Will repeat the file infinitely")
        ("ip", po::value<std::string>()->default_value(std::string("127.0.0.1")), "Interface on which UDP Frames will be sent")
        ("port", po::value<uint16_t>()->default_value(8200), "Port on which UDP Frames will be sent")
        ("file", po::value<std::string>(), "File to replay")
    ;
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    if(vm.count("file") == 0)
    {
        std::cout << desc << "\nFile parameter is REQUIRED\n";
        return 1;
    }
    pcapFile = vm["file"].as<std::string>();
    shouldRepeat = vm.count("repeat") > 0;
    period_ms = vm["period"].as<long>();

    endPoint = ip::udp::endpoint(ip::address::from_string(vm["ip"].as<std::string>()),
                                 vm["port"].as<uint16_t>());
    UDPSocket.open(ip::udp::v4());
    socket_base::reuse_address option(true);
    UDPSocket.set_option(option);

    timer.expires_from_now(boost::posix_time::milliseconds(period_ms));
    timer.async_wait(processNextDataInPcapFile);
    service.run();
    return 0;
}
