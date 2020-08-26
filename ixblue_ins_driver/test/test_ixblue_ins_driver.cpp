#include <gtest/gtest.h>

#include "../src/ros_publisher.h"

TEST(ROSPublisherTester, CanFillANavSatFixMsg)
{
    ixblue_stdbin_decoder::Data::Position pos;
    pos.latitude_deg = 1.2345;
    pos.longitude_deg = 9.8745;
    pos.altitude_m = 123.456;

    ixblue_stdbin_decoder::Data::PositionDeviation posDev;
    posDev.north_stddev_m = 0.3;
    posDev.east_stddev_m = 1.2;
    posDev.north_east_corr = 2.7;
    posDev.altitude_stddev_m = 4.5;

    ixblue_stdbin_decoder::Data::BinaryNav nav;
    nav.position = pos;
    nav.positionDeviation = posDev;

    const sensor_msgs::NavSatFixPtr msg = ROSPublisher::toNavSatFixMsg(nav);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->latitude, pos.latitude_deg);
    EXPECT_EQ(msg->longitude, pos.longitude_deg);
    EXPECT_EQ(msg->altitude, pos.altitude_m);
    EXPECT_EQ(msg->position_covariance_type, sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
    EXPECT_EQ(msg->position_covariance[0], posDev.east_stddev_m * posDev.east_stddev_m);
    EXPECT_EQ(msg->position_covariance[4], posDev.north_stddev_m * posDev.north_stddev_m);
    EXPECT_EQ(msg->position_covariance[8], posDev.altitude_stddev_m * posDev.altitude_stddev_m);
}

TEST(ROSPublisherTester, CannotFillANavSatFixMsgIfNoPositionInNav)
{
    ixblue_stdbin_decoder::Data::BinaryNav nav;
    const sensor_msgs::NavSatFixPtr msg = ROSPublisher::toNavSatFixMsg(nav);
    EXPECT_EQ(msg, nullptr);
}

TEST(ROSPublisherTester, CanFillANavSatFixMsgButNoCovarianceIfNoPositionDeviationInNav)
{
    ixblue_stdbin_decoder::Data::Position pos;
    pos.latitude_deg = 1.2345;
    pos.longitude_deg = 9.8745;
    pos.altitude_m = 123.456;

    ixblue_stdbin_decoder::Data::BinaryNav nav;
    nav.position = pos;

    const sensor_msgs::NavSatFixPtr msg = ROSPublisher::toNavSatFixMsg(nav);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->latitude, pos.latitude_deg);
    EXPECT_EQ(msg->longitude, pos.longitude_deg);
    EXPECT_EQ(msg->altitude, pos.altitude_m);
    EXPECT_EQ(msg->position_covariance_type, sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
