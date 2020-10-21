#pragma once

#include <memory>

#include <boost/optional.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/node_handle.h>
#include <ros/timer.h>

#include <ixblue_stdbin_decoder/data_models/navigation_data/ins_algorithm_status.h>
#include <ixblue_stdbin_decoder/data_models/navigation_data/ins_system_status.h>

class DiagnosticsPublisher
{
public:
    DiagnosticsPublisher(ros::NodeHandle& nh);
    void setHardwareID(const std::string& hwId);
    void stdImuTick(const ros::Time& stamp);
    void updateStatus(
        const boost::optional<ixblue_stdbin_decoder::Data::INSSystemStatus>& systemStatus,
        const boost::optional<ixblue_stdbin_decoder::Data::INSAlgorithmStatus>&
            algorithmStatus);

private:
    void diagTimerCallback(const ros::TimerEvent&);
    void produceStatusDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

    // Parameters
    double expected_frequency;
    const double frequency_tolerance = 0.1;
    double max_latency;
    double connection_lost_timeout;

    ros::Timer diagnosticsTimer;
    diagnostic_updater::Updater diagnosticsUpdater;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> stdImuTopicDiagnostic;
    ros::SteadyTime lastMessageReceivedStamp;
    boost::optional<ixblue_stdbin_decoder::Data::INSSystemStatus> lastSystemStatus;
    boost::optional<ixblue_stdbin_decoder::Data::INSAlgorithmStatus> lastAlgorithmStatus;
};
