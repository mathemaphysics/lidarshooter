#include <ros/ros.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include "XYZIRBytes.hpp"
#include "XYZIRPoint.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "objtracker");

    // Publisher node needs to be global; no namespace
    ros::NodeHandle nodeHandle;
    ros::Publisher meshPublisher = nodeHandle.advertise<pcl_msgs::PolygonMesh>("objtracker", 20);

    // Load the objects to track
    pcl::PolygonMesh trackObject;
    pcl::io::loadPolygonFileSTL("./sphere.stl", trackObject);
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
