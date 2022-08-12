#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <embree3/rtcore.h>

#include <cstdint>
#include <cmath>
#include <vector>
#include <cstring>
#include <fstream>
#include <sstream>

#include "IntBytes.hpp"
#include "FloatBytes.hpp"
#include "XYZIRBytes.hpp"
#include "LidarDevice.hpp"

#undef USE_RAY_PACKETS

namespace lidarshooter
{
class MeshProjector
{
public:
    MeshProjector()
    {
        // Create the pubsub situation
        _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>("pandar", 20);
        _meshSubscriber = _nodeHandle.subscribe<pcl_msgs::PolygonMesh>("/objtracker", 1, &MeshProjector::meshCallback, this);

        // When object is created we start at frame index 0
        _frameIndex = 0;
    }

    ~MeshProjector()
    {
        // Clean up the geometry data
    }

    void meshCallback(const pcl_msgs::PolygonMesh::ConstPtr& _mesh)
    {
        // Announce until unneeded
        std::cout << "Received a frame" << std::endl;

        // Load the objects to track
        pcl_conversions::toPCL(*_mesh, _trackObject);
        std::cout << "Points in tracked object      : " << _trackObject.cloud.width * _trackObject.cloud.height << std::endl;
        std::cout << "Triangles in tracked object   : " << _trackObject.polygons.size() << std::endl;

        // Make the output location for the cloud
        sensor_msgs::PointCloud2 msg;
        lidarshooter::LidarDevice config;

        // Trace out the Hesai configuration for now
        const int xsteps = 32;
        const int ysteps = 150;
        const float xstart = M_PI_2 + 0.01;
        const float xstop = M_PI_2 + (M_PI_4 / 3.0);
        const float dx = (xstop - xstart) / (float)(xsteps - 1);
        const float ystart = 0.0; // Phi goes all around
        const float ystop = 2.0 * M_PI;
        const float dy = (ystop - ystart) / (float)(ysteps - 1); // Make step depend on theta; fewer for angles closer to 0 and pi
        const float deviceHeight = 4.6; // Assume 5 m
        config.initMessage(msg, xsteps * ysteps, ++_frameIndex);

        msg.data.clear();

        // For the time being we *must* initialize the scene here; make _device and _scene local variables?
        _device = rtcNewDevice(nullptr);
        _scene = rtcNewScene(_device);

        // Create the geometry buffers for tracked object's vertices and indexes
        _objectGeometry = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_TRIANGLE);
        _objectVertices = (float*) rtcSetNewGeometryBuffer(
            _objectGeometry, RTC_BUFFER_TYPE_VERTEX, 0,
            RTC_FORMAT_FLOAT3, 3 * sizeof(float),
            _trackObject.cloud.width * _trackObject.cloud.height
        );
        _objectTriangles = (unsigned*) rtcSetNewGeometryBuffer(
            _objectGeometry, RTC_BUFFER_TYPE_INDEX, 0,
            RTC_FORMAT_UINT3, 3 * sizeof(unsigned),
            _trackObject.polygons.size()
        );

        // Create the geometry buffers for the ground vertices and indexes
        _groundGeometry = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_QUAD);
        _groundVertices = (float*) rtcSetNewGeometryBuffer(
            _groundGeometry, RTC_BUFFER_TYPE_VERTEX, 0,
            RTC_FORMAT_FLOAT3, 3 * sizeof(float),
            4
        );
        _groundQuadrilaterals = (unsigned*) rtcSetNewGeometryBuffer(
            _groundGeometry, RTC_BUFFER_TYPE_INDEX, 0,
            RTC_FORMAT_UINT4, 4 * sizeof(unsigned),
            1
        );

        // Update mesh with new locations and possibly structure
        updateGround();
        updateMeshPolygons(_frameIndex);
        rtcCommitGeometry(_objectGeometry);
        rtcCommitGeometry(_groundGeometry);
        rtcAttachGeometry(_scene, _objectGeometry);
        rtcAttachGeometry(_scene, _groundGeometry);
        rtcReleaseGeometry(_objectGeometry);
        rtcReleaseGeometry(_groundGeometry);
        rtcCommitScene(_scene);

#ifdef USE_RAY_PACKETS
        int rayCount = 0;
        int validRays = { -1, -1, -1, -1};
        RTCRayHit4 rayhit4;
#endif
        for (int ix = 0; ix < xsteps; ++ix)
        {
            for (int iy = 0; iy < ysteps; ++iy)
            {
                // Set the angular coordinates
                float theta = xstart + static_cast<float>(ix) * dx;
                float phi = ystart + static_cast<float>(iy) * dy;
                float rad = deviceHeight / std::cos(theta);
                
                // The normalized direction to trace
                float pxo = std::sin(theta) * std::cos(phi);
                float pyo = std::sin(theta) * std::sin(phi);
                float pzo = std::cos(theta);

#ifndef USE_RAY_PACKETS
                RTCRayHit rayhit;
                getMeshIntersect1(0.0f, 0.0f, 0.0f, pxo, pyo, pzo, &rayhit);

                if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
                {
                    lidarshooter::XYZIRBytes cloudBytes(rayhit.ray.tfar * pxo, rayhit.ray.tfar * pyo, rayhit.ray.tfar * pzo, 64.0, ix);
                    cloudBytes.AddToCloud(msg);
                }
#else
                rayhit4.ray.org_x[rayCount % 4] = 0.0; rayhit4.ray.org_y[rayCount % 4] = 0.0; rayhit4.ray.org_z[rayCount % 4] = 0.0;
                rayhit4.ray.dir_x[rayCount % 4] = pxo; rayhit4.ray.dir_y[rayCount % 4] = pyo; rayhit4.ray.dir_z[rayCount % 4] = pzo;
                ++rayCount;
                if (rayCount % 4 == 0)
                {
                    getMeshIntersect4(_scene, validRays, rayhit4);
                    for (int i = 0; i < 4; ++i)
                    {
                        if (rayhit4.hit.geomID != RTC_INVALID_GEOMETRY_ID)
                        {
                            
                        }
                    }
                }
#endif
            }
        }

        // Spoof the LiDAR device
        _cloudPublisher.publish(msg);
        rtcReleaseScene(_scene);
        rtcReleaseDevice(_device);
    }

private:
    // Setting the publish frequency
    std::uint32_t _frameIndex;
    float *_objectVertices;
    unsigned *_objectTriangles;
    float *_groundVertices;
    unsigned *_groundQuadrilaterals;
    pcl::PolygonMesh _trackObject;
    RTCDevice _device;
    RTCScene _scene;
    RTCGeometry _objectGeometry;
    RTCGeometry _groundGeometry;
    ros::NodeHandle _nodeHandle;
    ros::Publisher _cloudPublisher;
    ros::Subscriber _meshSubscriber;

    void updateGround()
    {
        // Set the ground; eventually make this its own function
        _groundVertices[0] = -50.0; _groundVertices[1]  = -50.0; _groundVertices[2]  = -5.0;
        _groundVertices[3] = -50.0; _groundVertices[4]  =  50.0; _groundVertices[5]  = -5.0;
        _groundVertices[6] =  50.0; _groundVertices[7]  =  50.0; _groundVertices[8]  = -5.0;
        _groundVertices[9] =  50.0; _groundVertices[10] = -50.0; _groundVertices[11] = -5.0;
        _groundQuadrilaterals[0] = 0;
        _groundQuadrilaterals[1] = 1;
        _groundQuadrilaterals[2] = 2;
        _groundQuadrilaterals[3] = 3;
    }

    void updateMeshPolygons(int frameIndex)
    {
        // Set the triangle element indexes 
        std::size_t idx = 0;
        for (auto poly : _trackObject.polygons)
        {
            std::uint32_t vert1 = poly.vertices[0];
            std::uint32_t vert2 = poly.vertices[1];
            std::uint32_t vert3 = poly.vertices[2];

            _objectTriangles[3 * idx + 0] = vert1; _objectTriangles[3 * idx + 1] = vert2; _objectTriangles[3 * idx + 2] = vert3; // Mesh triangle
            ++idx;
        }

        // Set the actual vertex positions
        for (std::size_t jdx = 0; jdx < _trackObject.cloud.width * _trackObject.cloud.height; ++jdx)
        {
            auto rawData = _trackObject.cloud.data.data() + jdx * _trackObject.cloud.point_step;
            auto bytes = lidarshooter::XYZIRBytes(rawData, rawData + 4, rawData + 8, nullptr, nullptr);

            float px = bytes.xPos.asFloat;
            float py = bytes.yPos.asFloat;
            float pz = bytes.zPos.asFloat;

            _objectVertices[3 * jdx + 0] = px; _objectVertices[3 * jdx + 1] = py; _objectVertices[3 * jdx + 2] = pz; // Mesh vertex
        }
    }

    void getMeshIntersect1(float ox, float oy, float oz, float dx, float dy, float dz, RTCRayHit *rayhit)
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
        rtcIntersect1(_scene, &context, rayhit);
    }

    void getMeshIntersect4(RTCScene scene, const int *validRays, RTCRayHit4 rayhit)
    {
        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        // If rayhit.ray.geomID != RTC_INVALID_GEOMETRY_ID then you have a solid hit
        // at a distance of rayhit.ray.tfar
        rtcIntersect4(validRays, _scene, &context, &rayhit);
    }
};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pandar");
    lidarshooter::MeshProjector meshProjector;
    ros::spin();

    return 0;
}
