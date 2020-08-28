#include "ros_publisher.h"
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cmath>
#include <ros/node_handle.h>

ROSPublisher::ROSPublisher()
{
    ros::NodeHandle nh("~");

    nh.param("frame_id", frame_id, std::string("ins"));
    nh.param("time_source", time_source, std::string("ins"));
    nh.param("time_origin", time_origin, std::string("unix"));

    if(time_source == std::string("ros"))
    {
        useInsAsTimeReference = false;
    }
    else if(time_source != std::string("ins"))
    {
        ROS_WARN("This timestamp source is not available. You can use ins or ros. By "
                 "default we "
                 "replace your value by ins.");
        time_source = std::string("ins");
    }

    if(time_origin == std::string("sensor_default"))
    {
        useUnixAsTimeOrigin = false;
    }
    else if(time_origin != std::string("unix"))
    {
        ROS_WARN("This timestamp origin is not available. You can use unix or "
                 "sensor_default. By default we "
                 "replace your value by unix.");
        time_origin = std::string("unix");
    }

    // Check parameters' value
    ROS_INFO("Frame ID : %s", frame_id.c_str());
    ROS_INFO("Timestamp register in the header will come from : %s", time_source.c_str());
    ROS_INFO("Timestamp register in the header will be in the base time of : %s",
             time_origin.c_str());

    stdImuPublisher = nh.advertise<sensor_msgs::Imu>("standard/imu", 1);
    stdNavSatFixPublisher = nh.advertise<sensor_msgs::NavSatFix>("standard/navsatfix", 1);
    stdTimeReferencePublisher =
        nh.advertise<sensor_msgs::TimeReference>("standard/timereference", 1);
    stdInsPublisher = nh.advertise<ixblue_ins_msgs::Ins>("ix/ins", 1);
}

void ROSPublisher::onNewStdBinData(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
                                   const ixblue_stdbin_decoder::Data::NavHeader& headerData)
{
    auto headerMsg = getHeader(headerData, navData);

    auto imuMsg = toImuMsg(navData);
    auto navsatfixMsg = toNavSatFixMsg(navData);
    auto iXinsMsg = toiXInsMsg(navData);

    if(!useInsAsTimeReference)
    {
        auto timeReferenceMsg = toTimeReference(headerData);

        if(timeReferenceMsg)
        {
            timeReferenceMsg->header = headerMsg;
            stdTimeReferencePublisher.publish(timeReferenceMsg);
        }
    }

    if(imuMsg)
    {
        imuMsg->header = headerMsg;
        stdImuPublisher.publish(imuMsg);
    }
    if(navsatfixMsg)
    {
        navsatfixMsg->header = headerMsg;
        stdNavSatFixPublisher.publish(navsatfixMsg);
    }
    if(iXinsMsg)
    {
        iXinsMsg->header = headerMsg;
        stdInsPublisher.publish(iXinsMsg);
    }
}

std_msgs::Header ROSPublisher::getHeader(const ixblue_stdbin_decoder::Data::NavHeader& headerData,
                                         const ixblue_stdbin_decoder::Data::BinaryNav& navData)
{
    // --- Initialisation
    std_msgs::Header res;

    // --- Frame ID
    res.frame_id = frame_id;

    // --- Timestamp
    if(useInsAsTimeReference)
    {

        uint32_t sec = (uint32_t)((headerData.navigationDataValidityTime_100us) / 10000);
        uint32_t nsec =
            (uint32_t)(((headerData.navigationDataValidityTime_100us) % 10000) * 100000);

        // --- Stamp origin = unix i.e. since 1st of january 1970
        if(useUnixAsTimeOrigin)
        {
            // Step 1 : to gregorian date
            boost::gregorian::date survey_day(navData.systemDate.get().year,
                                              navData.systemDate.get().month,
                                              navData.systemDate.get().day);

            // Step 2 : to unix date
            boost::gregorian::date unix_origin(1970, 1, 1);
            boost::posix_time::ptime survey_day_p = boost::posix_time::ptime(survey_day);
            boost::posix_time::ptime unix_origin_p =
                boost::posix_time::ptime(unix_origin);
            uint32_t stamp_since_origin_sec =
                (uint32_t)(survey_day_p - unix_origin_p).total_seconds();

            // Step 3 : to ROS format
            res.stamp = ros::Time(stamp_since_origin_sec + sec, nsec);
        }
        else
        {
            res.stamp = ros::Time(sec, nsec);
        }
    }
    else
    {
        res.stamp = ros::Time::now();
    }

    return res;
}

sensor_msgs::ImuPtr ROSPublisher::toImuMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData)
{

    // --- Check if there are enough data to send the message
    if(navData.rotationRateVesselFrame.is_initialized() == false ||
       navData.attitudeQuaternion.is_initialized() == false ||
       navData.accelerationVesselFrame.is_initialized() == false)
    {
        return nullptr;
    }

    // --- Initialisation
    sensor_msgs::ImuPtr res = boost::make_shared<sensor_msgs::Imu>();

    // --- Orientation
    res->orientation.x = navData.attitudeQuaternion.get().q0;
    res->orientation.y = navData.attitudeQuaternion.get().q1;
    res->orientation.z = navData.attitudeQuaternion.get().q2;
    res->orientation.w = navData.attitudeQuaternion.get().q3;

    // --- Orientation SD
    if(navData.attitudeQuaternionDeviation.is_initialized() == false)
    {
        // We don't have the velocity covariance in data, so according to the ROS standard
        // described in msg file we set the whole matrix to 0.
        res->orientation_covariance.assign(0);
    }
    else
    {
        // According to ros documentation, if the covariance is not available we only put
        // the variance in the diagonal
        res->orientation_covariance[0] =
            navData.attitudeQuaternionDeviation.get().quat_stddev_xi1 *
            navData.attitudeQuaternionDeviation.get().quat_stddev_xi1;
        res->orientation_covariance[4] =
            navData.attitudeQuaternionDeviation.get().quat_stddev_xi2 *
            navData.attitudeQuaternionDeviation.get().quat_stddev_xi2;
        res->orientation_covariance[8] =
            navData.attitudeQuaternionDeviation.get().quat_stddev_xi3 *
            navData.attitudeQuaternionDeviation.get().quat_stddev_xi3;
    }

    // --- Angular Velocity
    // According to the ros standard, the angular velocity must be in rad/sec
    res->angular_velocity.x =
        navData.rotationRateVesselFrame.get().xv1_degsec * M_PI / 180.;
    res->angular_velocity.y =
        navData.rotationRateVesselFrame.get().xv2_degsec * M_PI / 180.;
    res->angular_velocity.z =
        navData.rotationRateVesselFrame.get().xv3_degsec * M_PI / 180.;

    // --- Angular Velocity SD
    if(navData.rotationRateVesselFrameDeviation.is_initialized() == false)
    {
        // We don't have the velocity covariance in data, so according to the ROS standard
        // described in msg file we set the whole matrix to 0.
        res->angular_velocity_covariance.assign(0);
    }
    else
    {
        // According to ros documentation, if the covariance is not available we only put
        // the variance in the diagonal
        res->angular_velocity_covariance[0] =
            (navData.rotationRateVesselFrameDeviation.get().xv1_stddev_degsec * M_PI /
             180) *
            (navData.rotationRateVesselFrameDeviation.get().xv1_stddev_degsec * M_PI /
             180);
        res->angular_velocity_covariance[4] =
            (navData.rotationRateVesselFrameDeviation.get().xv2_stddev_degsec * M_PI /
             180) *
            (navData.rotationRateVesselFrameDeviation.get().xv2_stddev_degsec * M_PI /
             180);
        res->angular_velocity_covariance[8] =
            (navData.rotationRateVesselFrameDeviation.get().xv3_stddev_degsec * M_PI /
             180) *
            (navData.rotationRateVesselFrameDeviation.get().xv3_stddev_degsec * M_PI /
             180);
    }

    // --- Linear Acceleration
    res->linear_acceleration.x = navData.accelerationVesselFrame.get().xv1_msec2;
    res->linear_acceleration.y = navData.accelerationVesselFrame.get().xv2_msec2;
    res->linear_acceleration.z = navData.accelerationVesselFrame.get().xv3_msec2;

    // --- Linear Acceleration SD
    if(navData.accelerationVesselFrameDeviation.is_initialized() == false)
    {
        // We don't have the velocity covariance in data, so according to the ROS standard
        // described in msg file we set the whole matrix to 0.
        res->linear_acceleration_covariance.assign(0);
    }
    else
    {
        // According to ros documentation, if the covariance is not available we only put
        // the variance in the diagonal
        res->linear_acceleration_covariance[0] =
            navData.accelerationVesselFrameDeviation.get().xv1_stddev_msec2 *
            navData.accelerationVesselFrameDeviation.get().xv1_stddev_msec2;
        res->linear_acceleration_covariance[4] =
            navData.accelerationVesselFrameDeviation.get().xv2_stddev_msec2 *
            navData.accelerationVesselFrameDeviation.get().xv2_stddev_msec2;
        res->linear_acceleration_covariance[8] =
            navData.accelerationVesselFrameDeviation.get().xv3_stddev_msec2 *
            navData.accelerationVesselFrameDeviation.get().xv3_stddev_msec2;
    }

    return res;
}

sensor_msgs::NavSatFixPtr
ROSPublisher::toNavSatFixMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData)
{

    // --- Check if there are enough data to send the message
    if(navData.position.is_initialized() == false)
    {
        return nullptr;
    }

    // --- Initialisation
    sensor_msgs::NavSatFixPtr res = boost::make_shared<sensor_msgs::NavSatFix>();

    // --- Position
    res->latitude = navData.position.get().latitude_deg;
    res->longitude = navData.position.get().longitude_deg;
    res->altitude = navData.position.get().altitude_m;

    // --- Position SD
    if(navData.positionDeviation.is_initialized() == false)
    {
        // We don't have the velocity covariance in data, so according to the ROS standard
        // described in msg file we set the whole matrix to 0.
        res->position_covariance.assign(0);
        res->position_covariance_type = 0;
    }
    else
    {
        // According to ros documentation, if the covariance is not available we only put
        // the variance in the diagonal. ENU order.
        res->position_covariance[0] = navData.positionDeviation.get().east_stddev_m *
                                      navData.positionDeviation.get().east_stddev_m;
        res->position_covariance[1] = navData.positionDeviation.get().north_east_corr *
                                      navData.positionDeviation.get().east_stddev_m *
                                      navData.positionDeviation.get().north_stddev_m;
        res->position_covariance[3] = navData.positionDeviation.get().north_east_corr *
                                      navData.positionDeviation.get().east_stddev_m *
                                      navData.positionDeviation.get().north_stddev_m;
        res->position_covariance[4] = navData.positionDeviation.get().north_stddev_m *
                                      navData.positionDeviation.get().north_stddev_m;
        res->position_covariance[8] = navData.positionDeviation.get().altitude_stddev_m *
                                      navData.positionDeviation.get().altitude_stddev_m;
        res->position_covariance_type = 2;
    }

    return res;
}

sensor_msgs::TimeReferencePtr
ROSPublisher::toTimeReference(const ixblue_stdbin_decoder::Data::NavHeader& headerData)
{
    // --- Initialisation
    sensor_msgs::TimeReferencePtr res = boost::make_shared<sensor_msgs::TimeReference>();

    // --- System time
    res->header.stamp = ros::Time::now();

    // --- INS Timestamp
    uint32_t sec = (uint32_t)((headerData.navigationDataValidityTime_100us) / 10000);
    uint32_t nsec =
        (uint32_t)(((headerData.navigationDataValidityTime_100us) % 10000) * 100000);

    res->time_ref = ros::Time(sec, nsec);

    // --- Frame
    res->source = std::string("ins");

    return res;
}

ixblue_ins_msgs::InsPtr
ROSPublisher::toiXInsMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData)
{

    // --- Check if there are enough data to send the message
    if(navData.position.is_initialized() == false ||
       navData.attitudeHeading.is_initialized() == false ||
       navData.speedVesselFrame.is_initialized() == false ||
       navData.insUserStatus.is_initialized() == false)
    {
        return nullptr;
    }

    // --- Initialisation
    ixblue_ins_msgs::InsPtr res = boost::make_shared<ixblue_ins_msgs::Ins>();

    // --- Position
    res->latitude = navData.position.get().latitude_deg;
    res->longitude = navData.position.get().longitude_deg;
    res->altitude_ref = navData.position.get().altitude_ref;
    res->altitude = navData.position.get().altitude_m;

    // --- Position SD
    if(navData.positionDeviation.is_initialized() == false)
    {
        // We don't have the velocity covariance in data, so according to the ROS standard
        // described in msg file we set the whole matrix to 0.
        res->position_covariance.assign(0);
    }
    else
    {
        // According to ros documentation, if the covariance is not available we only put
        // the variance in the diagonal. ENU order.
        res->position_covariance[0] = navData.positionDeviation.get().east_stddev_m *
                                      navData.positionDeviation.get().east_stddev_m;
        res->position_covariance[1] = navData.positionDeviation.get().north_east_corr *
                                      navData.positionDeviation.get().east_stddev_m *
                                      navData.positionDeviation.get().north_stddev_m;
        res->position_covariance[3] = navData.positionDeviation.get().north_east_corr *
                                      navData.positionDeviation.get().east_stddev_m *
                                      navData.positionDeviation.get().north_stddev_m;
        res->position_covariance[4] = navData.positionDeviation.get().north_stddev_m *
                                      navData.positionDeviation.get().north_stddev_m;
        res->position_covariance[8] = navData.positionDeviation.get().altitude_stddev_m *
                                      navData.positionDeviation.get().altitude_stddev_m;
    }

    // --- Attitude
    res->heading = navData.attitudeHeading.get().heading_deg;
    res->roll = navData.attitudeHeading.get().roll_deg;
    res->pitch = navData.attitudeHeading.get().pitch_deg;

    // --- Attitude SD
    if(navData.attitudeHeadingDeviation.is_initialized() == false)
    {
        // We don't have the velocity covariance in data, so according to the ROS standard
        // described in msg file we set the whole matrix to 0.
        res->attitude_covariance.assign(0);
    }
    else
    {
        // According to ros documentation, if the covariance is not available we only put
        // the variance in the diagonal. ENU order.
        res->attitude_covariance[0] =
            navData.attitudeHeadingDeviation.get().heading_stdDev_deg *
            navData.attitudeHeadingDeviation.get().heading_stdDev_deg;
        res->attitude_covariance[4] =
            navData.attitudeHeadingDeviation.get().roll_stdDev_deg *
            navData.attitudeHeadingDeviation.get().roll_stdDev_deg;
        res->attitude_covariance[8] =
            navData.attitudeHeadingDeviation.get().pitch_stdDev_deg *
            navData.attitudeHeadingDeviation.get().pitch_stdDev_deg;
    }

    // --- Speed Vessel Frame
    res->speed_vessel_frame.x = navData.speedVesselFrame.get().xv1_msec;
    res->speed_vessel_frame.y = navData.speedVesselFrame.get().xv2_msec;
    res->speed_vessel_frame.z = navData.speedVesselFrame.get().xv3_msec;

    // --- Speed ENU SD
    if(navData.speedGeographicFrameDeviation.is_initialized() == false)
    {
        // We don't have the velocity covariance in data, so according to the ROS standard
        // described in msg file we set the whole matrix to 0.
        res->speed_vessel_frame_covariance.assign(0);
    }
    else
    {
        // According to ros documentation, if the covariance is not available we only put
        // the variance in the diagonal. ENU order.
        res->speed_vessel_frame_covariance[0] =
            navData.speedGeographicFrameDeviation.get().east_stddev_msec *
            navData.speedGeographicFrameDeviation.get().east_stddev_msec;
        res->speed_vessel_frame_covariance[4] =
            navData.speedGeographicFrameDeviation.get().north_stddev_msec *
            navData.speedGeographicFrameDeviation.get().north_stddev_msec;
        res->speed_vessel_frame_covariance[8] =
            navData.speedGeographicFrameDeviation.get().up_stddev_msec *
            navData.speedGeographicFrameDeviation.get().up_stddev_msec;
    }

    return res;
}
