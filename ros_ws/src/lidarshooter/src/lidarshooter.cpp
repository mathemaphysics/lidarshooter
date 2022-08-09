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
#include <embree3/rtcore.h>

#include <cstdint>
#include <cmath>
#include <vector>
#include <cstring>

#include "IntBytes.hpp"
#include "FloatBytes.hpp"
#include "XYZIRBytes.hpp"
#include "LidarDevice.hpp"

void addMeshPolygons(pcl::PolygonMesh mesh, float *vertices, unsigned *triangles, int frameIndex)
{
    std::size_t idx = 0;
    for (auto poly : mesh.polygons)
    {
        std::uint32_t vert1 = poly.vertices[0];
        std::uint32_t vert2 = poly.vertices[1];
        std::uint32_t vert3 = poly.vertices[2];

        triangles[3 * idx + 0] = vert1; triangles[3 * idx + 1] = vert2; triangles[3 * idx + 2] = vert3; // Mesh triangle
        ++idx;
    }

    for (std::size_t jdx = 0; jdx < mesh.cloud.width * mesh.cloud.height; ++jdx)
    {
        auto rawData = mesh.cloud.data.data() + jdx * mesh.cloud.point_step;
        auto bytes = lidarshooter::XYZIRBytes(rawData, rawData + 4, rawData + 8, nullptr, nullptr);

        float px = bytes.xPos.asFloat - frameIndex * 0.05;
        float py = bytes.yPos.asFloat - frameIndex * 0.05;
        float pz = bytes.zPos.asFloat;

        vertices[3 * jdx + 0] = px; vertices[3 * jdx + 1] = py; vertices[3 * jdx + 2] = pz; // 1st vertex
    }
}

void getMeshIntersect1(float ox, float oy, float oz, float dx, float dy, float dz, RTCScene scene, RTCRayHit *rayhit)
{
    rayhit->ray.org_x  = ox; rayhit->ray.org_y = oy; rayhit->ray.org_z = oz;
    rayhit->ray.dir_x  = dx; rayhit->ray.dir_y = dy; rayhit->ray.dir_z = dz;
    rayhit->ray.tnear  = 0.f;
    rayhit->ray.tfar   = std::numeric_limits<float>::infinity();
    rayhit->hit.geomID = RTC_INVALID_GEOMETRY_ID;
    
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    // If rayhit.ray.geomID != RTC_INVALID_GEOMETRY_ID then you have a solid hit
    // at a distance of rayhit.ray.tfar
    rtcIntersect1(scene, &context, rayhit);
}

void getMeshIntersectNp(RTCScene scene, RTCRayHitNp *rayhit, unsigned int numRays)
{
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    // If rayhit.ray.geomID != RTC_INVALID_GEOMETRY_ID then you have a solid hit
    // at a distance of rayhit.ray.tfar
    rtcIntersectNp(scene, &context, rayhit, numRays);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pandar");

    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("pandar", 20);

    // Load the objects to track
    pcl::PolygonMesh trackObject;
    pcl::io::loadPolygonFileSTL("./boxcar.stl", trackObject);
    
    std::cout << "Points in tracked object: " << trackObject.cloud.width * trackObject.cloud.height << std::endl;
    std::cout << "Tracked object has " << trackObject.polygons.size() << " elements" << std::endl;
    for (auto field : trackObject.cloud.fields)
    {
        std::cout << "Field: " << field << std::endl;
    }

    // Setting the publish frequency
    ros::Rate rate(10);
    std::uint32_t frameIndex = 0;
    RTCDevice device = rtcNewDevice(nullptr);
    RTCScene scene = rtcNewScene(device);
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcAttachGeometry(scene, geom);

    // This is where to figure out where the ray intersects
    float *vertices = (float*) rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_VERTEX, 0,
        RTC_FORMAT_FLOAT3, 3*sizeof(float),
        trackObject.cloud.width * trackObject.cloud.height
    );
    unsigned *triangles = (unsigned*) rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_INDEX, 0,
        RTC_FORMAT_UINT3, 3*sizeof(unsigned),
        trackObject.polygons.size()
    );

    while (ros::ok())
    {
        // Build the message and its header
        sensor_msgs::PointCloud2 msg;
        lidarshooter::LidarDevice config;

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

        addMeshPolygons(trackObject, vertices, triangles, frameIndex);
        rtcCommitGeometry(geom);
        rtcCommitScene(scene);

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
                
                // The normalized direction to trace
                float pxo = std::sin(theta) * std::cos(phi);
                float pyo = std::sin(theta) * std::sin(phi);
                float pzo = std::cos(theta);

                RTCRayHit rayhit;
                getMeshIntersect1(0.0f, 0.0f, 0.0f, pxo, pyo, pzo, scene, &rayhit);

                if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
                {
                    lidarshooter::XYZIRBytes cloudBytes(rayhit.ray.tfar * pxo, rayhit.ray.tfar * pyo, rayhit.ray.tfar * pzo, 64.0, ix);
                    cloudBytes.AddToCloud(msg);
                }
                else
                {
                    lidarshooter::XYZIRBytes cloudBytes(px, py, pz, 64.0, ix);
                    cloudBytes.AddToCloud(msg);
                }
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

    // Clean up the geometry data
    rtcReleaseGeometry(geom);
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
}
