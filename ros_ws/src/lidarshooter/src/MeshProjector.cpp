/**
 * @file MeshProjector.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief MeshProjector class which traces objects
 * @version 0.1
 * @date 2022-08-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "MeshProjector.hpp"

#include <ros/ros.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

lidarshooter::MeshProjector::MeshProjector()
    : _nodeHandle("~")
{
    // Set up the logger
    _logger = spdlog::get(_applicationName);
    if (_logger == nullptr)
        _logger = spdlog::stdout_color_mt(_applicationName);

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");

    // Get value from the publisher node
    std::string configFile = "config.json";
    _nodeHandle.param("configfile", configFile, configFile);

    // Initializing the LiDAR device
    _logger->info("Loading config file {}", configFile);
    _config.initialize(configFile);

    // When object is created we start at frame index 0
    _frameIndex = 0;

    // Create the pubsub situation
    _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>("pandar", 20);
    _meshSubscriber = _nodeHandle.subscribe<pcl_msgs::PolygonMesh>("/objtracker/objtracker", 1, &MeshProjector::meshCallback, this);
}

lidarshooter::MeshProjector::MeshProjector(const std::string& _configFile)
    : _nodeHandle("~")
{
    // Set up the logger
    _logger = spdlog::get(_applicationName);
    if (_logger == nullptr)
        _logger = spdlog::stdout_color_mt(_applicationName);

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");

    // Initializing the LiDAR device
    _logger->info("Loading device configuration from {}", _configFile);
    _config.initialize(_configFile);

    // When object is created we start at frame index 0
    _frameIndex = 0;

    // Create the pubsub situation
    _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>("pandar", 20);
    _meshSubscriber = _nodeHandle.subscribe<pcl_msgs::PolygonMesh>("/objtracker/objtracker", 1, &MeshProjector::meshCallback, this);
}

lidarshooter::MeshProjector::~MeshProjector()
{
    // Probably some geometry cleanup if possible here when making the geometry
    // buffers persistent gets sorted
}

void lidarshooter::MeshProjector::meshCallback(const pcl_msgs::PolygonMesh::ConstPtr& _mesh)
{
    // Announce until unneeded
    _logger->info("Received a frame");

    // Load the objects to track
    pcl_conversions::toPCL(*_mesh, _trackObject);
    _logger->info("Points in tracked object      : {}", _trackObject.cloud.width * _trackObject.cloud.height);
    _logger->info("Triangles in tracked object   : {}", _trackObject.polygons.size());

    // Make the output location for the cloud
    sensor_msgs::PointCloud2 msg;

    // Trace out the Hesai configuration for now
    _config.initMessage(msg, ++_frameIndex);

    // Just do this for the sake of surety
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

    // Set up packet processing
    int validRays[RAY_PACKET_SIZE]; // Initialize all invalid
    int rayRings[RAY_PACKET_SIZE]; // Ring indexes will need to be stored for output
    for (int i = 0; i < RAY_PACKET_SIZE; ++i)
        validRays[i] = 0;
    for (int i = 0; i < RAY_PACKET_SIZE; ++i)
        rayRings[i] = -1;
    RayHitType rayhitn;

    // Initialize ray state for batch processing
    int rayState = 0;
    _config.reset();
    while (rayState == 0)
    {
        // Fill up the next ray in the buffer
        rayState = _config.nextRay(rayhitn, validRays);
        for (int idx = 0; idx < RAY_PACKET_SIZE; ++idx)
            rayRings[idx] = 0;

        // Execute when the buffer is full
        getMeshIntersect(validRays, &rayhitn);
        for (int ri = 0; ri < RAY_PACKET_SIZE; ++ri)
        {
            if (rayhitn.hit.geomID[ri] != RTC_INVALID_GEOMETRY_ID)
            {
                lidarshooter::XYZIRBytes cloudBytes(
                    rayhitn.ray.tfar[ri] * rayhitn.ray.dir_x[ri],
                    rayhitn.ray.tfar[ri] * rayhitn.ray.dir_y[ri],
                    rayhitn.ray.tfar[ri] * rayhitn.ray.dir_z[ri],
                    64.0, rayRings[ri]
                );
                cloudBytes.AddToCloud(msg);
            }
            validRays[ri] = 0; // Reset ray validity to invalid/off/don't compute
        }
    }

    // Spoof the LiDAR device
    _cloudPublisher.publish(msg);
    rtcReleaseScene(_scene);
    rtcReleaseDevice(_device);
}

void lidarshooter::MeshProjector::updateGround()
{
    // Set the ground; eventually make this its own function
    Eigen::Vector3f corner1(-50.0, -50.0, 0.0); _config.originToSensor(corner1);
    Eigen::Vector3f corner2(-50.0,  50.0, 0.0); _config.originToSensor(corner2);
    Eigen::Vector3f corner3( 50.0,  50.0, 0.0); _config.originToSensor(corner3);
    Eigen::Vector3f corner4( 50.0, -50.0, 0.0); _config.originToSensor(corner4);

    _groundVertices[0] = corner1.x(); _groundVertices[1]  = corner1.y(); _groundVertices[2]  = corner1.z();
    _groundVertices[3] = corner2.x(); _groundVertices[4]  = corner2.y(); _groundVertices[5]  = corner2.z();
    _groundVertices[6] = corner3.x(); _groundVertices[7]  = corner3.y(); _groundVertices[8]  = corner3.z();
    _groundVertices[9] = corner4.x(); _groundVertices[10] = corner4.y(); _groundVertices[11] = corner4.z();

    _groundQuadrilaterals[0] = 0;
    _groundQuadrilaterals[1] = 1;
    _groundQuadrilaterals[2] = 2;
    _groundQuadrilaterals[3] = 3;
}

void lidarshooter::MeshProjector::updateMeshPolygons(int frameIndex)
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
        
        float px, py, pz;
        auto point = lidarshooter::XYZIRPoint(rawData);
        point.getPoint(&px, &py, &pz, nullptr, nullptr);

        Eigen::Vector3f ptrans(px, py, pz);
        _config.originToSensor(ptrans);

        _objectVertices[3 * jdx + 0] = ptrans.x(); _objectVertices[3 * jdx + 1] = ptrans.y(); _objectVertices[3 * jdx + 2] = ptrans.z(); // Mesh vertex
    }
}

void lidarshooter::MeshProjector::getMeshIntersect(int *_valid, RayHitType *_rayhit)
{
    GET_MESH_INTERSECT(_valid, _rayhit);
}

void lidarshooter::MeshProjector::getMeshIntersect1(float ox, float oy, float oz, float dx, float dy, float dz, RTCRayHit *rayhit)
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

void lidarshooter::MeshProjector::getMeshIntersect8(const int *validRays, RTCRayHit8 *rayhit)
{
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    // If rayhit.ray.geomID != RTC_INVALID_GEOMETRY_ID then you have a solid hit
    // at a distance of rayhit.ray.tfar
    rtcIntersect8(validRays, _scene, &context, rayhit);
}

void lidarshooter::MeshProjector::getMeshIntersect16(const int *validRays, RTCRayHit16 *rayhit)
{
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    // If rayhit.ray.geomID != RTC_INVALID_GEOMETRY_ID then you have a solid hit
    // at a distance of rayhit.ray.tfar
    rtcIntersect16(validRays, _scene, &context, rayhit);
}