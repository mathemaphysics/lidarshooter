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

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "LidarShooter.hpp"
#include "MeshProjector.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS to eat the --ros-args flags first
    ros::init(argc, argv, "pandar");

    // Handle command line input
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Display help")
        ("version", "Returns the version of test executive")
        ("ros-args", "ROS arguments")
        ("r", "ROS arguments")
        ("config", boost::program_options::value<std::string>(), "Configuration file to load")
    ;

    // Parse and store command line variables for later
    boost::program_options::variables_map variables;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), variables);
    boost::program_options::notify(variables);

    // Stop if help requested
    if (variables.count("help") > 0)
    {
        std::cout << desc << std::endl;
        return 0;
    }

    // Stop if version requested
    if (variables.count("version") > 0)
    {
        std::cout << "LiDARShooter" << std::endl;
        return 0;
    }

    // Grab the config file name
    std::string configFile;
    if (variables.count("config") > 0)
        configFile = variables["config"].as<std::string>();
    else
    {
        std::cout << "Config file not given but is required" << std::endl;
        return -1;
    }

    // Set up the logger
    auto logger = spdlog::get(APPLICATION_NAME);
    if (logger == nullptr)
        logger = spdlog::stdout_color_mt(APPLICATION_NAME);

    // Start up the mesh projector
    lidarshooter::MeshProjector meshProjector(configFile);
    ros::spin();

    return 0;
}
