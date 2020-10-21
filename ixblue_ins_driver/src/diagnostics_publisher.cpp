#include "diagnostics_publisher.h"

DiagnosticsPublisher::DiagnosticsPublisher(ros::NodeHandle& nh)
{
    nh.param("expected_frequency", expected_frequency, 10.0);
    nh.param("max_latency", max_latency, 1.0);
    nh.param("connection_lost_timeout", connection_lost_timeout, 10.0);

    ROS_INFO("Expected frequency for diagnostics : %.2f Hz", expected_frequency);
    ROS_INFO("Max latency acceptable for diagnostics : %.3f s", max_latency);
    ROS_INFO("Connection lost timeout : %.3f s", connection_lost_timeout);

    diagnosticsUpdater.add("status", this,
                           &DiagnosticsPublisher::produceStatusDiagnostics);
    stdImuTopicDiagnostic.reset(new diagnostic_updater::TopicDiagnostic(
        "imu topic", diagnosticsUpdater,
        diagnostic_updater::FrequencyStatusParam(&expected_frequency, &expected_frequency,
                                                 frequency_tolerance, 10),
        diagnostic_updater::TimeStampStatusParam(-max_latency, max_latency)));
    diagnosticsTimer = nh.createTimer(ros::Duration(0.1),
                                      &DiagnosticsPublisher::diagTimerCallback, this);
}

void DiagnosticsPublisher::setHardwareID(const std::string& hwId)
{
    diagnosticsUpdater.setHardwareID(hwId);
}

void DiagnosticsPublisher::stdImuTick(const ros::Time& stamp)
{
    stdImuTopicDiagnostic->tick(stamp);
}

void DiagnosticsPublisher::updateStatus(
    const boost::optional<ixblue_stdbin_decoder::Data::INSSystemStatus>& systemStatus,
    const boost::optional<ixblue_stdbin_decoder::Data::INSAlgorithmStatus>&
        algorithmStatus)
{
    lastMessageReceivedStamp = ros::SteadyTime::now();
    lastSystemStatus = systemStatus;
    lastAlgorithmStatus = algorithmStatus;
}

void DiagnosticsPublisher::diagTimerCallback(const ros::TimerEvent&)
{
    diagnosticsUpdater.update();
}

void DiagnosticsPublisher::produceStatusDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& status)
{
    if(!lastAlgorithmStatus.is_initialized() || !lastSystemStatus.is_initialized())
    {
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No data received yet");
    }
    else if((ros::SteadyTime::now() - lastMessageReceivedStamp).toSec() >
            connection_lost_timeout)
    {
        std::stringstream ss;
        ss << "No more data for more than " << connection_lost_timeout << " s";
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, ss.str());
    }
    else
    {
        const std::bitset<32> algoStatus1{lastAlgorithmStatus->status1};
        const std::bitset<32> algoStatus2{lastAlgorithmStatus->status2};
        const std::bitset<32> algoStatus3{lastAlgorithmStatus->status3};
        const std::bitset<32> algoStatus4{lastAlgorithmStatus->status4};

        const std::bitset<32> systemStatus1{lastSystemStatus->status1};
        const std::bitset<32> systemStatus2{lastSystemStatus->status2};
        const std::bitset<32> systemStatus3{lastSystemStatus->status3};

        status.add("algorithm_status_1", algoStatus1.to_string());
        status.add("algorithm_status_2", algoStatus2.to_string());
        status.add("algorithm_status_3", algoStatus3.to_string());
        status.add("algorithm_status_4", algoStatus4.to_string());

        status.add("system_status_1", systemStatus1.to_string());
        status.add("system_status_2", systemStatus2.to_string());
        status.add("system_status_3", systemStatus3.to_string());

        if(systemStatus1.test(
               ixblue_stdbin_decoder::Data::INSSystemStatus::Status1::SERIAL_IN_R_ERR))
        {
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                           "Serial input error");
        }
        else if(systemStatus1.test(
                    ixblue_stdbin_decoder::Data::INSSystemStatus::Status1::INPUT_A_ERR))
        {
            // GNNS Input error on Atlans, Input A error on other systems
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                           "GNSS or Input A error");
        }
        // TODO other system status checks
        else if(systemStatus2.test(ixblue_stdbin_decoder::Data::INSSystemStatus::Status2::
                                       WAIT_FOR_POSITION))
        {
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                           "System is waiting for position");
        }
        else if(algoStatus1.test(
                    ixblue_stdbin_decoder::Data::INSAlgorithmStatus::Status1::ALIGNMENT))
        {
            status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                           "System in alignment, do not move");
        }
        else if(algoStatus1.test(ixblue_stdbin_decoder::Data::INSAlgorithmStatus::
                                     Status1::FINE_ALIGNMENT))
        {
            status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Fine alignment");
        }
        else if(algoStatus1.test(
                    ixblue_stdbin_decoder::Data::INSAlgorithmStatus::Status1::NAVIGATION))
        {
            status.summary(diagnostic_msgs::DiagnosticStatus::OK, "System in navigation");
        }
        else
        {
            status.summary(diagnostic_msgs::DiagnosticStatus::OK, "");
        }
    }
}
