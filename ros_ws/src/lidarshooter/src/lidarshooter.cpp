#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree.h>
#include <curl/curl.h>

#include <cstdint>
#include <cmath>
#include <vector>
#include <cstring>

#include "IntBytes.hpp"
#include "FloatBytes.hpp"
#include "XYZIRBytes.hpp"
#include "PointCloudXYZIR.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pandar");

    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("pandar", 20);

    // Load the objects to track
    pcl::PolygonMesh trackObject;
    sensor_msgs::PointCloud2 trackObjectROS;
    pcl::io::loadPolygonFileSTL("./boxcar.stl", trackObject);
    pcl_conversions::fromPCL(trackObject.cloud, trackObjectROS);
    
    std::cout << "Points in tracked object: " << trackObject.cloud.width * trackObject.cloud.height << std::endl;
    for (auto field : trackObject.cloud.fields)
    {
        std::cout << "Field: " << field << std::endl;
    }

    // Setting the publish frequency
    ros::Rate rate(10);
    std::uint32_t frameIndex = 0;
    while (ros::ok())
    {
        // Build the message and its header
        sensor_msgs::PointCloud2 msg;
        lidarshooter::PointCloudXYZIR config;

        const int xsteps = 32;
        const int ysteps = 150;
        config.initMessage(msg, xsteps * ysteps, ++frameIndex);

        // Trace out the Hesai configuration for now
        const float xstart = M_PI_2 + 0.01;
        const float xstop = M_PI_2 + (M_PI_4 / 3.0);
        const float dx = (xstop - xstart) / (float)(xsteps - 1);
        const float ystart = 0.0; // Phi goes all around
        const float ystop = 2.0 * M_PI;
        const float dy = (ystop - ystart) / (float)(ysteps - 1); // Make step depend on theta; fewer for angles closer to 0 and pi
        const float deviceHeight = 4.6; // Assume 5 m

        msg.data.clear();
        for (int ix = 0; ix < xsteps; ++ix)
        {
            for (int iy = 0; iy < ysteps; ++iy)
            {
                // Set the angular coordinates
                float theta = xstart + static_cast<float>(ix) * dx;
                float phi = ystart + static_cast<float>(iy) * dy;
                float rad = deviceHeight / std::cos(theta);

                // Transform to cartesian coordinates
                float px = rad * std::sin(theta) * std::cos(phi);
                float py = rad * std::sin(theta) * std::sin(phi);
                float pz = -1.0 * deviceHeight;
                
                float pxo = std::sin(theta) * std::cos(phi);
                float pyo = std::sin(theta) * std::sin(phi);
                float pzo = std::cos(theta);

                // This is where to figure out where the ray intersects
                for (auto poly : trackObject.polygons)
                {
                    std::uint32_t vert1 = poly.vertices[0];
                    std::uint32_t vert2 = poly.vertices[1];
                    std::uint32_t vert3 = poly.vertices[2];

                    auto rawData1 = trackObject.cloud.data.data() + vert1 * trackObject.cloud.point_step;
                    auto rawData2 = trackObject.cloud.data.data() + vert2 * trackObject.cloud.point_step;
                    auto rawData3 = trackObject.cloud.data.data() + vert3 * trackObject.cloud.point_step;

                    auto bytes1 = lidarshooter::XYZIRBytes(rawData1, rawData1 + 4, rawData1 + 8, nullptr, nullptr);
                    auto bytes2 = lidarshooter::XYZIRBytes(rawData2, rawData2 + 4, rawData2 + 8, nullptr, nullptr);
                    auto bytes3 = lidarshooter::XYZIRBytes(rawData3, rawData3 + 4, rawData3 + 8, nullptr, nullptr);

                    float p1x = bytes1.xPos.asFloat - frameIndex * 0.05;
                    float p1y = bytes1.yPos.asFloat - frameIndex * 0.05;
                    float p1z = bytes1.zPos.asFloat;
                    
                    float p2x = bytes2.xPos.asFloat - frameIndex * 0.05;
                    float p2y = bytes2.yPos.asFloat - frameIndex * 0.05;
                    float p2z = bytes2.zPos.asFloat;

                    float p3x = bytes3.xPos.asFloat - frameIndex * 0.05;
                    float p3y = bytes3.yPos.asFloat - frameIndex * 0.05;
                    float p3z = bytes3.zPos.asFloat;

                    Eigen::Matrix4f M;
                    M << p1x, p2x, p3x, -1.0 * pxo,
                         p1y, p2y, p3y, -1.0 * pyo,
                         p1z, p2z, p3z, -1.0 * pzo,
                         1.0, 1.0, 1.0,        0.0;
                    Eigen::Vector4f origin;
                    origin << 0.0, 0.0, 0.0, 1.0;
                    Eigen::Vector4f R = M.inverse() * origin;

                    if (std::fabs(R(0) + R(1) + R(2) - 1.0) < 0.02
                        && 0.0 <= R(0) && R(0) <= 1.0
                        && 0.0 <= R(1) && R(1) <= 1.0
                        && 0.0 <= R(2) && R(2) <= 1.0)
                    {
                        px = R(3) * pxo;
                        py = R(3) * pyo;
                        pz = R(3) * pzo;
                        break;
                    }
                }

                lidarshooter::XYZIRBytes cloudBytes(px, py, pz, 64.0, ix);
                cloudBytes.AddToCloud(msg);
            }
        }

        //// Add the tracking object
        //for (int i = 0; i < trackObject.cloud.width * trackObject.cloud.height; ++i)
        //{
        //    auto rawData = trackObject.cloud.data.data() + i * trackObject.cloud.point_step;
        //    auto bytes = lidarshooter::XYZIRBytes(rawData, rawData + 4, rawData + 8, nullptr, nullptr);
        //    bytes.xPos.asFloat -= frameIndex * 0.05;
        //    bytes.yPos.asFloat -= frameIndex * 0.05;
        //    bytes.intensity.asFloat = 64.0;
        //    bytes.ring.asInteger = 0;
        //    bytes.AddToCloud(msg);
        //}

        //pcl::PCLPointCloud2 pco1, pco2;
        //pcl_conversions::toPCL(msg, pco1);
        //pcl::concatenatePointCloud(pco1, trackObject.cloud, pco2);
        //pcl_conversions::fromPCL(pco2, msg);

        publisher.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}
