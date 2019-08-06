#pragma once

#include "ixblue_ins_msgs/ins.h"
#include <iXblue_stdbin_decoder/data_models/nav_header.h>
#include <iXblue_stdbin_decoder/data_models/stdbin.h>
#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>

class ROSPublisher
{
public:
    ROSPublisher();
    void onNewStdBinData(const StdBinDecoder::Data::BinaryNav& navData,
                         const StdBinDecoder::Data::NavHeader& headerData);

protected:
    // Header
    std_msgs::Header getHeader(const StdBinDecoder::Data::NavHeader& headerData, const StdBinDecoder::Data::BinaryNav& navData);

    // Standard ros msgs
    sensor_msgs::ImuPtr toImuMsg(const StdBinDecoder::Data::BinaryNav& navData);
    sensor_msgs::NavSatFixPtr
    toNavSatFixMsg(const StdBinDecoder::Data::BinaryNav& navData);
    sensor_msgs::TimeReferencePtr
    toTimeReference(const StdBinDecoder::Data::NavHeader& headerData);

    // iXblue ros msgs
    ixblue_ins_msgs::insPtr toiXInsMsg(const StdBinDecoder::Data::BinaryNav& navData);

    // Launch parameters
    std::string frame_id;
    std::string time_source;
    std::string time_origin;

    // Publishers
    ros::Publisher stdImuPublisher;
    ros::Publisher stdNavSatFixPublisher;
    ros::Publisher stdTimeReferencePublisher;
    ros::Publisher stdInsPublisher;

    // Utils
    bool useInsAsTimeReference = true;
    bool useUnixAsTimeOrigin = true;
};
