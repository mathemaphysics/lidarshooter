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

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "LidarShooter.hpp"
#include "XYZIRBytes.hpp"
#include "XYZIRPoint.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS to eat the --ros-args flags first
    ros::init(argc, argv, "objtracker");

    // Set up the logger
    auto logger = spdlog::get(APPLICATION_NAME);
    if (logger == nullptr)
        logger = spdlog::stdout_color_mt(APPLICATION_NAME);

    // Publisher node needs to be global; no namespace
    ros::NodeHandle nodeHandle("~");

    // Advertise the mesh state
    ros::Publisher meshPublisher = nodeHandle.advertise<pcl_msgs::PolygonMesh>("meshstate", 20);

    // Get value from the publisher node
    std::string meshFile = "mesh.stl";
    nodeHandle.param("/objtracker/meshfile", meshFile, meshFile);
    logger->info("Loading mesh file {}", meshFile);

    // Load the objects to track
    pcl::PolygonMesh trackObject;
    pcl::io::loadPolygonFileSTL(meshFile, trackObject);
    logger->info("Points in tracked object      : {}", trackObject.cloud.width * trackObject.cloud.height);
    logger->info("Triangles in tracked object   : {}", trackObject.polygons.size());

    // The main loop; each iteration produces a new point cloud
    ros::Rate rate(10);
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
    }

    return 0;
}
