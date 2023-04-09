/**
 * @file LidarDevice_test.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
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
        lidarDevice = std::make_shared<lidarshooter::LidarDevice>(
            (configPath / "config/hesai-pandar-XT-32-lidar_0000.json").string()
        );
    }

    void TearDown() override
    {
        lidarDevice.reset();
    }

    std::shared_ptr<lidarshooter::LidarDevice> lidarDevice;
};

TEST_F(LidarDeviceTest, ReadConfigParameters1)
{
    int verticalIndex, horizontalIndex;
    EXPECT_EQ(lidarDevice->getSensorUid(), "lidar_0000");
    EXPECT_NO_THROW(lidarDevice->getCurrentIndex(&verticalIndex, &horizontalIndex));
    EXPECT_EQ(verticalIndex, 0);
    EXPECT_EQ(horizontalIndex, 0);
    EXPECT_EQ(lidarDevice->getTotalRays(), 150 * 32);
}

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return 0;
}
