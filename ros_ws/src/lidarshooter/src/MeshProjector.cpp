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

#include <regex>
#include <sstream>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

lidarshooter::MeshProjector::MeshProjector(ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger)
    : _nodeHandle("~"), _publishPeriod(__publishPeriod), _tracePeriod(__tracePeriod),
      _device(rtcNewDevice(nullptr)), _scene(rtcNewScene(_device)),
      _meshWasUpdated(false), _meshWasUpdatedPublic(true),
      _stateWasUpdated(false), _stateWasUpdatedPublic(false),
      _shouldPublishCloud(true)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");

    // Initialize to zero buffer size; dynamic allocation is okay
    _objectVerticesBufferSize = 0;
    _objectElementsBufferSize = 0;
    _groundVerticesBufferSize = 0;
    _groundElementsBufferSize = 0;
    
    // Set up geometry
    setupObjectGeometryBuffers(3000, 6000); // TODO: Fix this immediately
    setupGroundGeometryBuffers(8, 2);       // TODO: You need automatic buffer sizing

    // Get the sensorUid we want to run
    std::string nodeNamespace = _nodeHandle.getNamespace();
    std::regex slashRegex("/");
    auto strippedSensorUid = std::ostringstream();
    std::regex_replace(
        std::ostreambuf_iterator<char>(strippedSensorUid),
        nodeNamespace.begin(), nodeNamespace.end(), slashRegex, ""
    );
    _sensorUid = strippedSensorUid.str();
    _logger->info("SensorUID from namespace is {}", _sensorUid);

    // Get value from the publisher node
    std::string configFile = "config.json";
    _nodeHandle.param("configfile", configFile, configFile); // Get config file from ROS params

    // Initializing the LiDAR device
    _logger->info("Loading config file {} specified via configfile ROS parameter", configFile);
    _config.reset(new LidarDevice(configFile, _sensorUid, __logger));

    // The current state cloud is now a shared pointer and needs alloc'ed
    _currentState = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());
    
    // Setup for the contextless tracing space
    _traceData = TraceData::create(_config, _currentState);

    // When object is created we start at frame index 0
    _frameIndex = 0;

    /**
     * This is critical because without initialization of the header of the
     * current cloud state, publishing this to SENSR will cause some memory
     * strangeness and permanently mangles plotting.
     * 
     * TODO: Need to have an empty initializer that sets the container claim
     * it contains zero points; it's rendering some garbage that doesn't go
     * away still, if only a couple points.
     */
    _config->initMessage(_currentState, _frameIndex);

    // Check that they match
    if (_sensorUid != _config->getSensorUid())
        _logger->warn("SensorUID in config ({}) does not match namespace ({})", _config->getSensorUid(), _sensorUid);

    // Set velocities to zero
    _linearDisplacement.setZero();
    _angularDisplacement.setZero();

    // Create the pubsub situation; in this constructor cloud advertises on /[namespace]/pandar
    _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>("pandar", 20); // TODO: Make this queue size and "pandar" parameters
    _joystickSubscriber = _nodeHandle.subscribe<geometry_msgs::Twist>("/joystick/cmd_vel", LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &MeshProjector::joystickCallback, this);
    _multiJoystickSubscriber = _nodeHandle.subscribe<lidarshooter::NamedTwist>("/joystick/all/cmd_vel", LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &MeshProjector::multiJoystickCallback, this);
    _publishTimer = _nodeHandle.createTimer(_publishPeriod, std::bind(&MeshProjector::publishCloud, this));
    _traceTimer = _nodeHandle.createTimer(_tracePeriod, std::bind(&MeshProjector::traceMeshWrapper, this));

    // Admit we updated the mesh because this is the first iteration
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);

    // False because it hasn't bee traced yet
    _stateWasUpdated.store(false);
    _stateWasUpdatedPublic.store(false);
}

lidarshooter::MeshProjector::MeshProjector(const std::string& _configFile, ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger)
    : _nodeHandle("~"), _publishPeriod(__publishPeriod), _tracePeriod(__tracePeriod),
      _device(rtcNewDevice(nullptr)), _scene(rtcNewScene(_device)),
      _meshWasUpdated(false), _meshWasUpdatedPublic(false),
      _stateWasUpdated(false), _stateWasUpdatedPublic(false),
      _shouldPublishCloud(true)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");
    
    // Initialize to zero buffer size; dynamic allocation is okay
    _objectVerticesBufferSize = 0;
    _objectElementsBufferSize = 0;
    _groundVerticesBufferSize = 0;
    _groundElementsBufferSize = 0;

    // Set up geometry
    setupObjectGeometryBuffers(3000, 6000);
    setupGroundGeometryBuffers(8, 2);

    // Get the sensorUid we want to run
    std::string nodeNamespace = _nodeHandle.getNamespace();
    std::regex slashRegex("/");
    auto strippedSensorUid = std::ostringstream();
    std::regex_replace(
        std::ostreambuf_iterator<char>(strippedSensorUid),
        nodeNamespace.begin(), nodeNamespace.end(), slashRegex, ""
    );
    _sensorUid = strippedSensorUid.str();
    _logger->info("SensorUID from namespace is {}", _sensorUid);

    // Initializing the LiDAR device
    _logger->info("Loading device configuration from {}", _configFile);
    _config.reset(new LidarDevice(_configFile, _sensorUid, __logger));

    // The current state cloud is now a shared pointer and needs alloc'ed
    _currentState = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());

    // Setup for the contextless tracing space
    _traceData = TraceData::create(_config, _currentState);

    // When object is created we start at frame index 0
    _frameIndex = 0;

    /**
     * This is critical because without initialization of the header of the
     * current cloud state, publishing this to SENSR will cause some memory
     * strangeness and permanently mangles plotting.
     * 
     * TODO: Need to have an empty initializer that sets the container claim
     * it contains zero points; it's rendering some garbage that doesn't go
     * away still, if only a couple points.
     */
    _config->initMessage(_currentState, _frameIndex);

    // Check that they match
    if (_sensorUid != _config->getSensorUid())
        _logger->warn("SensorUID in config ({}) does not match namespace ({})", _config->getSensorUid(), _sensorUid);

    // Set velocities to zero
    _linearDisplacement.setZero();
    _angularDisplacement.setZero();

    // Create the pubsub situation
    _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>(fmt::format("/{}/pandar", _config->getSensorUid()), 20);
    _meshSubscriber = _nodeHandle.subscribe<pcl_msgs::PolygonMesh>("/objtracker/meshstate", LIDARSHOOTER_MESH_SUB_QUEUE_SIZE, &MeshProjector::meshCallback, this);
    _multiMeshSubscriber = _nodeHandle.subscribe<lidarshooter::NamedPolygonMesh>("/objtracker/all/meshstate", LIDARSHOOTER_MESH_SUB_QUEUE_SIZE, &MeshProjector::multiMeshCallback, this);
    _joystickSubscriber = _nodeHandle.subscribe<geometry_msgs::Twist>("/joystick/cmd_vel", LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &MeshProjector::joystickCallback, this);
    _multiJoystickSubscriber = _nodeHandle.subscribe<lidarshooter::NamedTwist>("/joystick/all/cmd_vel", LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &MeshProjector::multiJoystickCallback, this);
    _publishTimer = _nodeHandle.createTimer(_publishPeriod, std::bind(&MeshProjector::publishCloud, this));
    _traceTimer = _nodeHandle.createTimer(_tracePeriod, std::bind(&MeshProjector::traceMeshWrapper, this));

    // Admit we updated the mesh because this is the first iteration
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
}

lidarshooter::MeshProjector::MeshProjector(std::shared_ptr<LidarDevice> _configDevice, ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger)
    : _nodeHandle("~"), _publishPeriod(__publishPeriod), _tracePeriod(__tracePeriod),
      _device(rtcNewDevice(nullptr)), _scene(rtcNewScene(_device)),
      _meshWasUpdated(false), _meshWasUpdatedPublic(false),
      _stateWasUpdated(false), _stateWasUpdatedPublic(false),
      _shouldPublishCloud(true)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");
    
    // Initialize to zero buffer size; dynamic allocation is okay
    _objectVerticesBufferSize = 0;
    _objectElementsBufferSize = 0;
    _groundVerticesBufferSize = 0;
    _groundElementsBufferSize = 0;

    // Set up geometry
    setupObjectGeometryBuffers(3000, 6000);
    setupGroundGeometryBuffers(8, 2);

    // Get the sensorUid we want to run
    std::string nodeNamespace = _nodeHandle.getNamespace();
    std::regex slashRegex("/");
    auto strippedSensorUid = std::ostringstream();
    std::regex_replace(
        std::ostreambuf_iterator<char>(strippedSensorUid),
        nodeNamespace.begin(), nodeNamespace.end(), slashRegex, ""
    );
    _sensorUid = strippedSensorUid.str();
    _logger->info("SensorUID from namespace is {}", _sensorUid);

    // Initializing the LiDAR device
    _logger->info("Loaded device {} from preloaded device object", _configDevice->getSensorUid());
    _config = _configDevice;

    // The current state cloud is now a shared pointer and needs alloc'ed
    _currentState = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());

    // Setup for the contextless tracing space
    _traceData = TraceData::create(_config, _currentState);

    // When object is created we start at frame index 0
    _frameIndex = 0;

    /**
     * This is critical because without initialization of the header of the
     * current cloud state, publishing this to SENSR will cause some memory
     * strangeness and permanently mangles plotting.
     * 
     * TODO: Need to have an empty initializer that sets the container claim
     * it contains zero points; it's rendering some garbage that doesn't go
     * away still, if only a couple points.
     */
    _config->initMessage(_currentState, _frameIndex);

    // Check that they match
    if (_sensorUid != _config->getSensorUid())
        _logger->warn("SensorUID in config ({}) does not match namespace ({})", _config->getSensorUid(), _sensorUid);

    // Set velocities to zero
    _linearDisplacement.setZero();
    _angularDisplacement.setZero();

    // Create the pubsub situation
    _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>(fmt::format("/{}/pandar", _config->getSensorUid()), 20);
    _meshSubscriber = _nodeHandle.subscribe<pcl_msgs::PolygonMesh>("/objtracker/meshstate", LIDARSHOOTER_MESH_SUB_QUEUE_SIZE, &MeshProjector::meshCallback, this);
    _multiMeshSubscriber = _nodeHandle.subscribe<lidarshooter::NamedPolygonMesh>("/objtracker/all/meshstate", LIDARSHOOTER_MESH_SUB_QUEUE_SIZE, &MeshProjector::multiMeshCallback, this);
    _joystickSubscriber = _nodeHandle.subscribe<geometry_msgs::Twist>("/joystick/cmd_vel", LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &MeshProjector::joystickCallback, this);
    _multiJoystickSubscriber = _nodeHandle.subscribe<lidarshooter::NamedTwist>("/joystick/all/cmd_vel", LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &MeshProjector::multiJoystickCallback, this);
    _publishTimer = _nodeHandle.createTimer(_publishPeriod, std::bind(&MeshProjector::publishCloud, this));
    _traceTimer = _nodeHandle.createTimer(_tracePeriod, std::bind(&MeshProjector::traceMeshWrapper, this));

    // Admit we updated the mesh because this is the first iteration
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
}

lidarshooter::MeshProjector::~MeshProjector()
{
    // Obtain mutex locks before destruction so we don't interrupt publishing
    _cloudMutex.lock();
    _meshMutex.lock();
    for (auto& [name, mesh] : _meshMutexes)
        mesh.lock();
    _joystickMutex.lock();

    // Probably some geometry cleanup if possible here when making the geometry
    // buffers persistent gets sorted
    releaseObjectGeometryBuffers();
    releaseGroundGeometryBuffers();
}

void lidarshooter::MeshProjector::shutdown()
{
    // Now sure if we need to shut down the node manually
    _nodeHandle.shutdown();
}

void lidarshooter::MeshProjector::meshCallback(const pcl_msgs::PolygonMesh::ConstPtr& _mesh)
{
    // Announce until unneeded
    _logger->debug("Received a frame");

    // Load the objects to track
    _meshMutex.lock();
    pcl_conversions::toPCL(*_mesh, _trackObject);
    _logger->debug("Points in tracked object      : {}", _trackObject.cloud.width * _trackObject.cloud.height);
    _logger->debug("Triangles in tracked object   : {}", _trackObject.polygons.size());

    // Admit that we changed the mesh and it needs to be retraced
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
    _meshMutex.unlock();
}

void lidarshooter::MeshProjector::multiMeshCallback(const lidarshooter::NamedPolygonMeshConstPtr& _mesh)
{
    // Reject any mesh with a non-existent key
    auto meshIterator = _trackObjects.find(_mesh->name);
    if (meshIterator == _trackObjects.end())
    {
        _logger->warn("Received a frame for {} (key does not exist)", _mesh->name);
        return;
    }

    // Announce until unneeded
    _logger->debug("Received a frame for {}", _mesh->name);

    // Load the objects to track
    _meshMutexes[_mesh->name].lock();
    pcl_conversions::toPCL(_mesh->mesh, *(_trackObjects[_mesh->name]));
    _logger->debug("Points in tracked object      : {}", _trackObjects[_mesh->name]->cloud.width * _trackObjects[_mesh->name]->cloud.height);
    _logger->debug("Triangles in tracked object   : {}", _trackObjects[_mesh->name]->polygons.size());

    // Admit that we changed the mesh and it needs to be retraced
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
    _meshMutexes[_mesh->name].unlock();
}

// Important: This is only for initializing the mesh; you shouldn't use this
// function to make updates to the pointcloud geometry when rigid body rotations
// and translations are all you've done.
void lidarshooter::MeshProjector::addMeshToScene(const std::string& _meshName, const pcl::PolygonMesh::ConstPtr& _mesh)
{
    // This may not be what we want; make sure this is efficient
    _meshMutex.lock(); // TODO: Remove this when done with multi-mesh case
    _trackObject = *_mesh; // TODO: Remove this when done with _trackObjects (plural)

    // Emplace new copy of _mesh into _trackObjects
    _trackObjects.emplace(
        _meshName,
        pcl::PolygonMesh::Ptr(new pcl::PolygonMesh(*_mesh)) // Make a copy
    );

    // Emplace a new mesh mutex for each object
    _meshMutexes.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple() // Create mutex with no arguments
    );
    
    // Emplace a new joystick mutex for each object
    _joystickMutexes.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple() // Create mutex with no arguments
    );

    // Initialize to zero linear displacement
    _linearDisplacements.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple()
    );
    _linearDisplacements[_meshName].setZero();

    // Initialize to zero angular displacement
    _angularDisplacements.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple()
    );
    _angularDisplacements[_meshName].setZero();

    // Ado the geometry
    int geomId = _traceData->addGeometry(_meshName, RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE, _mesh->cloud.width * _mesh->cloud.height, _mesh->polygons.size());
    _logger->debug("Added geometric ID {}", geomId);

    // Indicate the mesh was updated so we get a retrace
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
    _meshMutex.unlock();
}

sensor_msgs::PointCloud2ConstPtr lidarshooter::MeshProjector::getCurrentStatePtr() const
{
    return _currentState;
}

void lidarshooter::MeshProjector::getCurrentStateCopy(pcl::PCLPointCloud2::Ptr& _output)
{
    // Don't interrupt a write with a read
    _cloudMutex.lock();
    auto tempConversion = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*_currentState, *tempConversion);
    pcl::copyPointCloud(*tempConversion, *_output);
    _cloudMutex.unlock();
}

void lidarshooter::MeshProjector::traceMeshWrapper()
{
    if (_meshWasUpdated.load() == true)
    {
        // Update _currentState
        traceMesh();

        // Turn off mesh updated flag
        _meshWasUpdated.store(false);
    }
}

void lidarshooter::MeshProjector::joystickCallback(const geometry_msgs::Twist::ConstPtr& _vel)
{
    // TODO: Move this into its own function and replace everywhere
    Eigen::Vector3f globalDisplacement = transformToGlobal(Eigen::Vector3f(_vel->linear.x, _vel->linear.y, _vel->linear.z));
    
    // Output actual displacement applied after rotation to local coordinates
    // TODO: Set both of these info() calls to debug() as soon as settled
    _logger->debug("Joystick signal: {}, {}, {}, {}, {}, {}",
                  _vel->linear.x, _vel->linear.y, _vel->linear.z,
                  _vel->angular.x, _vel->angular.y, _vel->angular.z);
    _logger->debug("Global displacement: {}, {}, {}, {}, {}, {}",
                  globalDisplacement.x(), globalDisplacement.y(), globalDisplacement.z(),
                  _vel->angular.x, _vel->angular.y, _vel->angular.z);

    // Update the linear total linear and angular displacement
    _joystickMutex.lock();
    _linearDisplacement += globalDisplacement;
    _angularDisplacement += Eigen::Vector3f(_vel->angular.x, _vel->angular.y, _vel->angular.z);

    // Hint to the tracer that it needs to run again
    _meshWasUpdated.store(true); // TODO: Don't update when the signal is (0, 0, 0, 0, 0, 0)
    _meshWasUpdatedPublic.store(true);
    _joystickMutex.unlock();
}

void lidarshooter::MeshProjector::multiJoystickCallback(const lidarshooter::NamedTwist::ConstPtr& _vel)
{
    // TODO: Move this into its own function and replace everywhere
    Eigen::Vector3f globalDisplacement = transformToGlobal(Eigen::Vector3f(_vel->twist.linear.x, _vel->twist.linear.y, _vel->twist.linear.z));
    
    // Output actual displacement applied after rotation to local coordinates
    // TODO: Set both of these info() calls to debug() as soon as settled
    _logger->debug("Joystick signal: {}, {}, {}, {}, {}, {}",
                  _vel->twist.linear.x, _vel->twist.linear.y, _vel->twist.linear.z,
                  _vel->twist.angular.x, _vel->twist.angular.y, _vel->twist.angular.z);
    _logger->debug("Global displacement: {}, {}, {}, {}, {}, {}",
                  globalDisplacement.x(), globalDisplacement.y(), globalDisplacement.z(),
                  _vel->twist.angular.x, _vel->twist.angular.y, _vel->twist.angular.z);

    // Update the linear total linear and angular displacement
    _joystickMutexes[_vel->name].lock();
    _linearDisplacements[_vel->name] += globalDisplacement;
    _angularDisplacements[_vel->name] += Eigen::Vector3f(_vel->twist.angular.x, _vel->twist.angular.y, _vel->twist.angular.z);

    // Hint to the tracer that it needs to run again
    _meshWasUpdated.store(true); // TODO: Don't update when the signal is (0, 0, 0, 0, 0, 0)
    _meshWasUpdatedPublic.store(true);
    _joystickMutexes[_vel->name].unlock();
}

void lidarshooter::MeshProjector::publishCloud()
{
    // This runs whether the cloud was updated or not; constant stream
    if (_shouldPublishCloud.load() == true)
    {
        _cloudMutex.lock();
        _cloudPublisher.publish(_currentState);
        _logger->debug("Published sequence ID {} at time {}", _currentState->header.seq, _currentState->header.stamp.toSec());
        _cloudMutex.unlock();
    }
}

void lidarshooter::MeshProjector::setCloudPublishState(bool __shouldPublishCloud)
{
    _shouldPublishCloud.store(__shouldPublishCloud);
    _logger->debug("Set publish state to {}", __shouldPublishCloud);
}

bool lidarshooter::MeshProjector::meshWasUpdated()
{
    if (_meshWasUpdatedPublic.load() == true)
    {
        _meshWasUpdatedPublic.store(false);
        return true;
    }
    else
        return false;
}

bool lidarshooter::MeshProjector::cloudWasUpdated()
{
    if (_stateWasUpdatedPublic.load() == true)
    {
        _stateWasUpdatedPublic.store(false);
        return true;
    }
    else
        return false;
}

inline Eigen::Vector3f lidarshooter::MeshProjector::transformToGlobal(Eigen::Vector3f _displacement)
{
    // Just for an Affine3f transform using an empty translation
    Eigen::AngleAxisf xRotation(_angularDisplacement.x(), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yRotation(_angularDisplacement.y(), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zRotation(_angularDisplacement.z(), Eigen::Vector3f::UnitZ());
    Eigen::Affine3f localRotation = Eigen::Translation3f(Eigen::Vector3f::Zero()) * zRotation * yRotation * xRotation;
    Eigen::Vector3f localDisplacement = localRotation * _displacement;
    return localDisplacement;
}

void lidarshooter::MeshProjector::updateGround()
{
    // Set the ground; eventually make this its own function
    Eigen::Vector3f corner1(-50.0, -50.0, 0.0); _config->originToSensor(corner1); // TODO: Allow configuration of the ground in JSON format
    Eigen::Vector3f corner2(-50.0,  50.0, 0.0); _config->originToSensor(corner2);
    Eigen::Vector3f corner3( 50.0,  50.0, 0.0); _config->originToSensor(corner3);
    Eigen::Vector3f corner4( 50.0, -50.0, 0.0); _config->originToSensor(corner4);

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

        _objectTriangles[3 * idx + 0] = vert1;
        _objectTriangles[3 * idx + 1] = vert2;
        _objectTriangles[3 * idx + 2] = vert3; // Mesh triangle
        ++idx;
    }

    // Set the actual vertex positions
    _logger->debug("Current net displacement: {}, {}, {}, {}, {}, {}",
                  _linearDisplacement.x(), _linearDisplacement.y(), _linearDisplacement.z(),
                  _angularDisplacement.x(), _angularDisplacement.y(), _angularDisplacement.z());

    // TODO: Replace this whole section with CloudTransformer::applyTransform
    for (std::size_t jdx = 0; jdx < _trackObject.cloud.width * _trackObject.cloud.height; ++jdx)
    {
        auto rawData = _trackObject.cloud.data.data() + jdx * _trackObject.cloud.point_step;
        
        float px, py, pz;
        auto point = lidarshooter::XYZIRPoint(rawData);
        point.getPoint(&px, &py, &pz, nullptr, nullptr);

        // Rotate into the local coordinate frame for this device
        Eigen::Vector3f ptrans(px, py, pz);

        // Build the transformation according to the present position in
        // _linearDisplacement and _angularDisplacement; TODO: Encapsulate this
        // into a function
        Eigen::Translation3f translation(_linearDisplacement);
        Eigen::AngleAxisf xRotation(_angularDisplacement.x(), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf yRotation(_angularDisplacement.y(), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf zRotation(_angularDisplacement.z(), Eigen::Vector3f::UnitZ());

        // Rotate first, then translate; remember, right-to-left operation order means rightmost goes first
        Eigen::Affine3f transform = translation * zRotation * yRotation * xRotation;
        
        // Apply the affine transformation and then transf
        ptrans = transform * ptrans;
        _config->originToSensor(ptrans);

        // Linear position update here
        _joystickMutex.lock(); // TODO: This is not needed at all now
        _objectVertices[3 * jdx + 0] = ptrans.x();
        _objectVertices[3 * jdx + 1] = ptrans.y();
        _objectVertices[3 * jdx + 2] = ptrans.z(); // Mesh vertex
        _joystickMutex.unlock();
    }
}

void lidarshooter::MeshProjector::traceMesh()
{
    /**
     * NEW METHOD RAYTRACING 
     */

    for (auto& [name, mesh] : _trackObjects)
    {
        // Make sure this mesh doesn't change during read
        _meshMutexes[name].lock();

        // Copy vertex and elemet data from mesh into buffers
        _traceData->updateGeometry(
            name,
            _linearDisplacement,
            _angularDisplacement,
            mesh
        );

        // Release just this mesh
        _meshMutexes[name].unlock();
    }
    _traceData->commitScene();
    _cloudMutex.lock(); // Locks _currentState
    _traceData->traceScene(++_frameIndex);
    _cloudMutex.unlock(); // Unlocks _currentState

    /*
     * NEW METHOD RAYTRACING 
     **/

    /**
     * OLD METHOD GEOMETRY UPDATE
     */

    // Update mesh with new locations and possibly structure
    updateGround();

    // Block changes to _trackObject->polygons until done
    _meshMutex.lock();
    updateMeshPolygons(_frameIndex);
    _meshMutex.unlock();

    // Commit the new geometry
    rtcCommitGeometry(_objectGeometry);
    rtcCommitGeometry(_groundGeometry);
    rtcCommitScene(_scene);

    /* 
     * OLD METHOD GEOMETRY UPDATE
     **/

    /**
     * OLD METHOD RAYTRACING
     */

    // Initialize ray state for batch processing
    _cloudMutex.lock();
    _config->initMessage(_currentState, ++_frameIndex);
    _currentState->data.clear();
    _config->reset();

    // Count the total iterations because the limits are needed for threading
    unsigned int numTotalRays = _config->getTotalRays();
    unsigned int numIterations = numTotalRays / LIDARSHOOTER_RAY_PACKET_SIZE + (numTotalRays % LIDARSHOOTER_RAY_PACKET_SIZE > 0 ? 1 : 0);
    unsigned int numThreads = 4; // TODO: Make this a parameter
    unsigned int numChunks = numIterations / numThreads + (numIterations % numThreads > 0 ? 1 : 0);

    std::mutex sharedCloudMutex;
    std::vector<std::thread> threads;
    std::atomic<int> totalPointCount;
    totalPointCount.store(0);
    for (int rayChunk = 0; rayChunk < numThreads; ++rayChunk)
    {
        //unsigned int startPosition = rayChunk * numChunks;
        // TODO: Convert the contents of the thread into a "chunk" function to simplify
        threads.emplace_back(
            [this, &sharedCloudMutex, numChunks, &totalPointCount](){
                for (int ix = 0; ix < numChunks; ++ix)
                {
                    // Set up packet processing
                    int rayState = 0;
                    int validRays[LIDARSHOOTER_RAY_PACKET_SIZE]; // Initialize all invalid
                    int rayRings[LIDARSHOOTER_RAY_PACKET_SIZE]; // Ring indexes will need to be stored for output
                    for (int i = 0; i < LIDARSHOOTER_RAY_PACKET_SIZE; ++i)
                        validRays[i] = 0;
                    for (int i = 0; i < LIDARSHOOTER_RAY_PACKET_SIZE; ++i)
                        rayRings[i] = -1;
                    RayHitType rayhitn;

                    // Fill up the next ray in the buffer
                    rayState = this->_config->nextRay(rayhitn, validRays);
                    for (int idx = 0; idx < LIDARSHOOTER_RAY_PACKET_SIZE; ++idx)
                        rayRings[idx] = 0;

                    // Execute when the buffer is full
                    this->getMeshIntersect(validRays, &rayhitn); // TODO: Make sure meshMutex doesn't need set?
                    for (int ri = 0; ri < LIDARSHOOTER_RAY_PACKET_SIZE; ++ri)
                    {
                        if (rayhitn.hit.geomID[ri] != RTC_INVALID_GEOMETRY_ID)
                        {
                            lidarshooter::XYZIRBytes cloudBytes(
                                rayhitn.ray.tfar[ri] * rayhitn.ray.dir_x[ri],
                                rayhitn.ray.tfar[ri] * rayhitn.ray.dir_y[ri],
                                rayhitn.ray.tfar[ri] * rayhitn.ray.dir_z[ri],
                                64.0, rayRings[ri]
                            );
                            sharedCloudMutex.lock();
                            cloudBytes.addToCloud(this->_currentState);
                            totalPointCount.store(totalPointCount.load() + 1);
                            sharedCloudMutex.unlock();
                        }
                        validRays[ri] = 0; // Reset ray validity to invalid/off/don't compute
                    }
                }
            }
        );
    }
    for (auto th = threads.begin(); th != threads.end(); ++th)
        th->join();

    // Set the point count to the value that made it back from tracing
    _currentState->width = totalPointCount.load();

    // Indicate that we just retraced and you can come and get it
    _stateWasUpdated.store(true);
    _stateWasUpdatedPublic.store(true);
    _cloudMutex.unlock();

    /*
     * OLD METHOD RAYTRACING
     **/
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

void lidarshooter::MeshProjector::setupObjectGeometryBuffers(int _numVertices, int _numElements)
{
    // Create the geometry itself
    _objectGeometry = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_TRIANGLE);

    // Now create the actual storage space for the vertices and set up
    _objectVertices = new float[_numVertices * 3 * sizeof(float)];
    _objectVerticesBuffer = rtcNewSharedBuffer(_device, _objectVertices, _numVertices * 3 * sizeof(float));
    rtcSetSharedGeometryBuffer(_objectGeometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, _objectVertices, 0, 3 * sizeof(float), _numVertices);

    // Now create the actual storage space for the elements and set up
    _objectTriangles = new unsigned[3 * sizeof(unsigned) * _numElements];
    _objectElementsBuffer = rtcNewSharedBuffer(_device, _objectTriangles, _numElements * 3 * sizeof(unsigned));
    rtcSetSharedGeometryBuffer(_objectGeometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, _objectTriangles, 0, 3 * sizeof(unsigned), _numElements);

    // Attach this geometry to the global scene
    rtcAttachGeometry(_scene, _objectGeometry);

    // Save in case we need to check for insufficient allocated space
    _objectVerticesBufferSize = _numVertices;
    _objectElementsBufferSize = _numElements;
}

void lidarshooter::MeshProjector::setupGroundGeometryBuffers(int _numVertices, int _numElements)
{
    // Create the geometry buffers for the ground vertices; just a quadrilateral
    _groundGeometry = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_QUAD);

    // Do the allocation
    _groundVertices = new float[_numVertices * 3 * sizeof(float)];
    _groundVerticesBuffer = rtcNewSharedBuffer(_device, _groundVertices, _numVertices * 3 * sizeof(float));
    rtcSetSharedGeometryBuffer(_groundGeometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, _groundVertices, 0, 3 * sizeof(float), _numVertices);
    
    // Repeat for the elements
    _groundQuadrilaterals = new unsigned[_numElements * 4 * sizeof(unsigned)];
    _groundElementsBuffer = rtcNewSharedBuffer(_device, _groundQuadrilaterals, _numElements * 4 * sizeof(unsigned));
    rtcSetSharedGeometryBuffer(_groundGeometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, _groundQuadrilaterals, 0, 4 * sizeof(unsigned), _numElements);

    // Attach the ground geometry to the global scene for tracing
    rtcAttachGeometry(_scene, _groundGeometry);

    // Save in case we need to check for insufficient allocated space
    _groundVerticesBufferSize = _numVertices;
    _groundElementsBufferSize = _numElements;
}

void lidarshooter::MeshProjector::releaseObjectGeometryBuffers()
{
    // Allow embree to do its thing
    rtcReleaseBuffer(_objectVerticesBuffer);
    rtcReleaseBuffer(_objectElementsBuffer);

    // Release the memory
    delete [] _objectVertices;
    delete [] _objectTriangles;
}

void lidarshooter::MeshProjector::releaseGroundGeometryBuffers()
{
    // Allow embree to do its thing
    rtcReleaseBuffer(_groundVerticesBuffer);
    rtcReleaseBuffer(_groundElementsBuffer);

    // Release the memory
    delete [] _groundVertices;
    delete [] _groundQuadrilaterals;
}
