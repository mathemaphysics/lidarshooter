/**
 * @file LidarDevice_test.cpp
 * @author Ryan P. Daly (mathemaphysics@gmail.com)
 * @brief Unit tests related to the LidarDevice class
 * @version 0.1
 * @date 2023-01-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>

#include "../LidarDevice.hpp"

#include <ros/ros.h>
#include <spdlog/spdlog.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <filesystem>

#include "LidarShooterTesting.hpp"

namespace
{

class LidarDeviceTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Silence the logger for testing
        spdlog::set_level(spdlog::level::off);

        // Create the device to work with
        auto configPath = std::filesystem::path(LIDARSHOOTER_TESTING_DATA_DIR);
        lidarDevice = lidarshooter::LidarDevice::create(
            (configPath / "config/hesai-pandar-XT-32-lidar_0000.json").string()
        );
    }

    void TearDown() override
    {
        lidarDevice.reset();
    }

    lidarshooter::LidarDevice::Ptr lidarDevice;
};

TEST_F(LidarDeviceTest, ReadConfigParameters)
{
    int verticalIndex, horizontalIndex;
    EXPECT_EQ(lidarDevice->getSensorUid(), "lidar_0000");
    EXPECT_NO_THROW(lidarDevice->getCurrentIndex(&verticalIndex, &horizontalIndex));
    EXPECT_EQ(verticalIndex, 0);
    EXPECT_EQ(horizontalIndex, 0);
    EXPECT_EQ(lidarDevice->getTotalRays(), 150 * 32);
}

TEST_F(LidarDeviceTest, InitializeMessage)
{
    // Need to do this to be able to work with messages requiring ROS operation
    ros::Time::init();

    auto cloud = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());
    lidarDevice->initMessage(cloud, 100);
    EXPECT_EQ(cloud->fields.size(), 5);
    EXPECT_EQ(cloud->height, 1);
    EXPECT_EQ(cloud->width, 0);
    EXPECT_EQ(cloud->point_step, 32);
    EXPECT_EQ(cloud->row_step, 0 * 32);
    EXPECT_EQ(cloud->is_bigendian, false);
    EXPECT_EQ(cloud->is_dense, true);
    EXPECT_EQ(cloud->header.seq, 100);
}

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return 0;
}
