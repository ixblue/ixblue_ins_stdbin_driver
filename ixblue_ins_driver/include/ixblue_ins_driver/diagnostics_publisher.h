#pragma once

#include <memory>
#include <chrono>
#include <bitset>

#include <boost/optional.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ixblue_stdbin_decoder/data_models/navigation_data/ins_algorithm_status.h>
#include <ixblue_stdbin_decoder/data_models/navigation_data/ins_system_status.h>

class DiagnosticsPublisher
{
public:
    DiagnosticsPublisher(std::shared_ptr<rclcpp::Node> nh);
    void setHardwareID(const std::string& hwId);
    void stdImuTick(const rclcpp::Time& stamp);
    void updateStatus(
        const boost::optional<ixblue_stdbin_decoder::Data::INSSystemStatus>& systemStatus,
        const boost::optional<ixblue_stdbin_decoder::Data::INSAlgorithmStatus>&
            algorithmStatus);

private:
    void diagTimerCallback();
    void produceStatusDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

    // Parameters
    double expected_frequency;
    const double frequency_tolerance = 0.1;
    double max_latency;
    double connection_lost_timeout;

    rclcpp::TimerBase::SharedPtr diagnosticsTimer;
    diagnostic_updater::Updater diagnosticsUpdater;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> stdImuTopicDiagnostic;
    rclcpp::Time lastMessageReceivedStamp;
    boost::optional<ixblue_stdbin_decoder::Data::INSSystemStatus> lastSystemStatus;
    boost::optional<ixblue_stdbin_decoder::Data::INSAlgorithmStatus> lastAlgorithmStatus;
    std::shared_ptr<rclcpp::Node> nh_;
};
