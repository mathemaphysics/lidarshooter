#include "MeshProjector.hpp"

#include <ros/ros.h>


lidarshooter::MeshProjector::MeshProjector()
{
    // Create the pubsub situation
    _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>("pandar", 20);
    _meshSubscriber = _nodeHandle.subscribe<pcl_msgs::PolygonMesh>("/objtracker", 1, &MeshProjector::meshCallback, this);

    // When object is created we start at frame index 0
    _frameIndex = 0;
}

lidarshooter::MeshProjector::~MeshProjector()
{
    // Probably some geometry cleanup if possible here when making the geometry
    // buffers persistent gets sorted
}

void lidarshooter::MeshProjector::meshCallback(const pcl_msgs::PolygonMesh::ConstPtr& _mesh)
{
    // Announce until unneeded
    std::cout << "Received a frame" << std::endl;

    // Load the objects to track
    pcl_conversions::toPCL(*_mesh, _trackObject);
    std::cout << "Points in tracked object      : " << _trackObject.cloud.width * _trackObject.cloud.height << std::endl;
    std::cout << "Triangles in tracked object   : " << _trackObject.polygons.size() << std::endl;

    // Make the output location for the cloud
    sensor_msgs::PointCloud2 msg;
    _config.initialize("/workspaces/yolo3d/ros_ws/hesai-pandar-XT-32.json");

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
    RTCRayHit8 rayhitn;

    int rayState = 0;
    _config.reset();
    while (rayState == 0)
    {
        // Fill up the next ray in the buffer
        rayState = _config.nextRay8(rayhitn, validRays);
        for (int idx = 0; idx < RAY_PACKET_SIZE; ++idx)
            rayRings[idx] = 0;

        // Execute when the buffer is full
        getMeshIntersect8(validRays, &rayhitn);
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
    Eigen::Vector3f corner3( 50.0, -50.0, 0.0); _config.originToSensor(corner3);
    Eigen::Vector3f corner4( 50.0,  50.0, 0.0); _config.originToSensor(corner4);

    _groundVertices[0] = corner1[0]; _groundVertices[1]  = corner1[1]; _groundVertices[2]  = corner1[2];
    _groundVertices[3] = corner2[0]; _groundVertices[4]  = corner2[1]; _groundVertices[5]  = corner2[2];
    _groundVertices[6] = corner3[0]; _groundVertices[7]  = corner3[1]; _groundVertices[8]  = corner3[2];
    _groundVertices[9] = corner4[0]; _groundVertices[10] = corner4[1]; _groundVertices[11] = corner4[2];

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
        auto bytes = lidarshooter::XYZIRBytes(rawData, rawData + 4, rawData + 8, nullptr, nullptr);

        float px = bytes.xPos.asFloat;
        float py = bytes.yPos.asFloat;
        float pz = bytes.zPos.asFloat;

        _objectVertices[3 * jdx + 0] = px; _objectVertices[3 * jdx + 1] = py; _objectVertices[3 * jdx + 2] = pz; // Mesh vertex
    }
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