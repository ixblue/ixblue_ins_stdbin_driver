#pragma once

#include <memory>

#include <boost/optional.hpp>

#include "ixblue_ins_msgs/Ins.h"
#include <ixblue_stdbin_decoder/data_models/nav_header.h>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>

class ROSPublisher
{
public:
    ROSPublisher();
    void onNewStdBinData(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
                         const ixblue_stdbin_decoder::Data::NavHeader& headerData);

    // Standard ros msgs
    static sensor_msgs::ImuPtr
    toImuMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);
    static sensor_msgs::NavSatFixPtr
    toNavSatFixMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);
    static sensor_msgs::TimeReferencePtr
    toTimeReference(const ixblue_stdbin_decoder::Data::NavHeader& headerData);

    // iXblue ros msgs
    static ixblue_ins_msgs::InsPtr
    toiXInsMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);

protected:
    // Header
    std_msgs::Header getHeader(const ixblue_stdbin_decoder::Data::NavHeader& headerData,
                               const ixblue_stdbin_decoder::Data::BinaryNav& navData);
    // Diagnostics
    void diagTimerCallback(const ros::TimerEvent&);
    void produceStatusDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

    // Launch parameters
    std::string frame_id;
    std::string time_source;
    std::string time_origin;
    double expected_frequency;
    const double frequency_tolerance = 0.1;
    double max_latency;

    // Publishers
    ros::Publisher stdImuPublisher;
    std::unique_ptr<diagnostic_updater::DiagnosedPublisher<sensor_msgs::Imu>>
        stdImuPublisherDiag;
    ros::Publisher stdNavSatFixPublisher;
    ros::Publisher stdTimeReferencePublisher;
    ros::Publisher stdInsPublisher;

    // Utils
    bool useInsAsTimeReference = true;
    bool useUnixAsTimeOrigin = true;

    // Diagnostics
    ros::Timer diagnosticsTimer;
    diagnostic_updater::Updater diagnosticsUpdater;
    boost::optional<ixblue_stdbin_decoder::Data::INSAlgorithmStatus> lastAlgorithmStatus;
    boost::optional<ixblue_stdbin_decoder::Data::INSSystemStatus> lastSystemStatus;
};
