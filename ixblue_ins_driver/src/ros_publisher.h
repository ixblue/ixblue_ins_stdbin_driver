#pragma once

#include <ixblue_ins_msgs/Ins.h>
#include <ixblue_stdbin_decoder/data_models/nav_header.h>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>

#include "diagnostics_publisher.h"

class ROSPublisher
{
public:
    ROSPublisher();
    void onNewStdBinData(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
                         const ixblue_stdbin_decoder::Data::NavHeader& headerData);

    // Standard ros msgs
    static sensor_msgs::ImuPtr
    toImuMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
             bool use_compensated_acceleration);
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

    // Launch parameters
    std::string frame_id;
    std::string time_source;
    std::string time_origin;
    bool use_compensated_acceleration;

    ros::NodeHandle nh;

    // Publishers
    ros::Publisher stdImuPublisher;
    ros::Publisher stdNavSatFixPublisher;
    ros::Publisher stdTimeReferencePublisher;
    ros::Publisher stdInsPublisher;
    DiagnosticsPublisher diagPub;

    // Utils
    bool useInsAsTimeReference = true;
    bool useUnixAsTimeOrigin = true;
};
