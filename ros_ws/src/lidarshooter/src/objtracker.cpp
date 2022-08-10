#include <ros/ros.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <iostream>
#include "XYZIRBytes.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "objtracker");

    ros::NodeHandle nodeHandle;
    ros::Publisher meshPublisher = nodeHandle.advertise<pcl_msgs::PolygonMesh>("objtracker", 20);

    // Load the objects to track
    pcl::PolygonMesh trackObject;
    pcl::io::loadPolygonFileSTL("./sphere.stl", trackObject);
    std::cout << "Points in tracked object      : " << trackObject.cloud.width * trackObject.cloud.height << std::endl;
    std::cout << "Triangles in tracked object   : " << trackObject.polygons.size() << std::endl;


    // The main loop; each iteration produces a new point cloud
    ros::Rate rate(2);
    int frameIndex = 200;
    while (ros::ok())
    {
        // Convert mesh to pcl_msgs::PolygonMesh
        pcl_msgs::PolygonMesh trackObjectMsg;
        for (int jdx = 0; jdx < trackObject.cloud.width * trackObject.cloud.height; ++jdx)
        {
            auto rawData = trackObject.cloud.data.data() + jdx * trackObject.cloud.point_step;
            auto bytes = lidarshooter::XYZIRBytes(rawData, rawData + 4, rawData + 8, rawData + 16, rawData + 20);
            bytes.xPos.asFloat = bytes.xPos.asFloat + 0.05;
            bytes.yPos.asFloat = bytes.yPos.asFloat + 0.05;
            bytes.zPos.asFloat = bytes.zPos.asFloat;
            for (int i = 0; i < 4; i++)
                rawData[i] = bytes.xPos.byteArray[i];
            for (int i = 0; i < 4; i++)
                rawData[i + 4] = bytes.yPos.byteArray[i];
            for (int i = 0; i < 4; i++)
                rawData[i + 8] = bytes.zPos.byteArray[i];
        }
        pcl_conversions::fromPCL(trackObject, trackObjectMsg);

        meshPublisher.publish(trackObjectMsg); // Eventually this will be a path and update each send
        ros::spinOnce();
        rate.sleep();
        ++frameIndex;
    }

    return 0;
}
