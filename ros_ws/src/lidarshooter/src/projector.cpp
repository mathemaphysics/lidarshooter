/**
 * @file lidarshooter.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Tracing backend
 * @version 0.1
 * @date 2022-08-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "LidarShooter.hpp"
#include "MeshProjector.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS to eat the --ros-args flags first
    ros::init(argc, argv, "sensoruid");

    // Set up the logger
    auto logger = spdlog::get(LIDARSHOOTER_APPLICATION_NAME);
    if (logger == nullptr)
        logger = spdlog::stdout_color_mt(LIDARSHOOTER_APPLICATION_NAME);

    // Start up the mesh projector
    logger->info("Starting up LiDARShooter");
    lidarshooter::MeshProjector meshProjector;
    ros::spin();

    return 0;
}
