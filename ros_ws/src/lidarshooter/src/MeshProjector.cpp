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
    for (auto& [name, mesh] : _joystickMutexes)
        mesh.lock();
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
    Eigen::Vector3f globalDisplacement = transformToGlobal(_vel->name, Eigen::Vector3f(_vel->twist.linear.x, _vel->twist.linear.y, _vel->twist.linear.z));
    
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

inline Eigen::Vector3f lidarshooter::MeshProjector::transformToGlobal(const std::string& _meshName, Eigen::Vector3f _displacement)
{
    // Just for an Affine3f transform using an empty translation
    Eigen::AngleAxisf xRotation(_angularDisplacements[_meshName].x(), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yRotation(_angularDisplacements[_meshName].y(), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zRotation(_angularDisplacements[_meshName].z(), Eigen::Vector3f::UnitZ());
    Eigen::Affine3f localRotation = Eigen::Translation3f(Eigen::Vector3f::Zero()) * zRotation * yRotation * xRotation;
    Eigen::Vector3f localDisplacement = localRotation * _displacement;
    return localDisplacement;
}

void lidarshooter::MeshProjector::traceMesh()
{
    for (auto& [name, mesh] : _trackObjects)
    {
        // Make sure this mesh doesn't change during read
        _meshMutexes[name].lock();

        // Copy vertex and elemet data from mesh into buffers
        _traceData->updateGeometry(
            name,
            _linearDisplacements[name],
            _angularDisplacements[name],
            mesh
        );

        // Debugging information
        _logger->debug("Updated {} with {} points and {} elements", name, mesh->cloud.width * mesh->cloud.height, mesh->polygons.size());

        // Release just this mesh
        _meshMutexes[name].unlock();
    }
    _traceData->commitScene();
    _cloudMutex.lock(); // Locks _currentState
    _traceData->traceScene(++_frameIndex);
    _cloudMutex.unlock(); // Unlocks _currentState

    _logger->debug("Trace cloud has {} points in it", _traceData->getTraceCloud()->width * _traceData->getTraceCloud()->height);

    // Indicate that we just retraced and you can come and get it
    _stateWasUpdated.store(true);
    _stateWasUpdatedPublic.store(true);
}
