/**
 * @file objtracker.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Object mesh broadcast node
 * @version 0.1
 * @date 2022-08-18
 * 
 * @copyright Copyright (c) 2022
 */

#include <ros/ros.h>

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

#include <boost/program_options.hpp>

#include "LidarShooter.hpp"
#include "XYZIRBytes.hpp"
#include "XYZIRPoint.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS to eat the --ros-args flags first
    ros::init(argc, argv, "objtracker");

    // Handle command line input
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Display help")
        ("version", "Returns the version of test executive")
        ("ros-args", "ROS arguments")
        ("-r", "ROS arguments")
        ("mesh", boost::program_options::value<std::string>(), "Path to the STL mesh to load")
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
    std::string meshFile;
    if (variables.count("mesh") > 0)
        meshFile = variables["mesh"].as<std::string>();
    else
    {
        std::cout << "Mesh file not given but is required" << std::endl;
        return -1;
    }

    // Publisher node needs to be global; no namespace
    ros::NodeHandle nodeHandle;
    ros::Publisher meshPublisher = nodeHandle.advertise<pcl_msgs::PolygonMesh>("objtracker", 20);

    // Load the objects to track
    pcl::PolygonMesh trackObject;
    pcl::io::loadPolygonFileSTL(meshFile, trackObject);
    std::cout << "Points in tracked object      : " << trackObject.cloud.width * trackObject.cloud.height << std::endl;
    std::cout << "Triangles in tracked object   : " << trackObject.polygons.size() << std::endl;

    // The main loop; each iteration produces a new point cloud
    ros::Rate rate(10);
    int frameIndex = 200;
    while (ros::ok())
    {
        // Convert mesh to pcl_msgs::PolygonMesh
        pcl_msgs::PolygonMesh trackObjectMsg;
        for (int jdx = 0; jdx < trackObject.cloud.width * trackObject.cloud.height; ++jdx)
        {
            float x, y, z, intensity;
            int ring;
            auto rawData = trackObject.cloud.data.data() + jdx * trackObject.cloud.point_step;
            auto point = lidarshooter::XYZIRPoint(rawData);
            point.getPoint(&x, &y, &z, &intensity, &ring);
            point.setPoint(x + 0.05, y + 0.05, z, intensity, ring);
            point.writePoint(rawData);
        }
        pcl_conversions::fromPCL(trackObject, trackObjectMsg);

        meshPublisher.publish(trackObjectMsg); // Eventually this will be a path and update each send
        ros::spinOnce();
        rate.sleep();
        ++frameIndex;
    }

    return 0;
}
